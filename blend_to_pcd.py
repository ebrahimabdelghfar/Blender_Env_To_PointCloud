"""Convert a Blender scene (map/environment) into a colored point cloud (binary PLY).

Surfaces of every visible mesh/curve/text object (modifiers + instances applied) are
sampled randomly; each point takes the color of its material: the image texture
feeding Principled BSDF Base Color (via UVs), else the Base Color value, else the
material viewport color.

Density per triangle = its texture resolution (texels/m^2; more points would only
repeat texel colors), clamped to [--min-density, --max-density]. Untextured surfaces
get --min-density. Set both equal for uniform density.
Default max 100 pts/m^2 = 10 cm spacing (Autoware map guidance: 0.1-0.2 m voxels).

Usage:
  blender -b map.blend --python blend_to_pcd.py -- --out map.ply [--max-density 100] [--min-density 4]
"""
import argparse
import sys
import time

import bpy
import numpy as np
from mathutils import Euler

TYPES = {'MESH', 'CURVE', 'SURFACE', 'FONT', 'META'}
GREY = np.array([128, 128, 128], np.uint8)
_img_cache = {}


def lin2srgb(c):
    c = np.clip(c, 0.0, 1.0)
    return np.where(c <= 0.0031308, c * 12.92, 1.055 * c ** (1 / 2.4) - 0.055)


def srgb8(lin_rgb):
    return (lin2srgb(np.asarray(lin_rgb, np.float32)) * 255 + 0.5).astype(np.uint8)


def image_rgb(img):
    """HxWx3 uint8 sRGB array for an image, None if it has no pixels (missing file)."""
    if img.name_full not in _img_cache:
        w, h = img.size
        rgb = None
        if w * h:
            c = img.channels
            px = np.empty(w * h * c, np.float32)
            img.pixels.foreach_get(px)
            px = px.reshape(h, w, c)
            px = px[..., :3] if c >= 3 else np.repeat(px[..., :1], 3, axis=2)
            if img.is_float:  # float images are stored linear, byte images already sRGB
                px = lin2srgb(px)
            rgb = (np.clip(px, 0, 1) * 255 + 0.5).astype(np.uint8)
        _img_cache[img.name_full] = rgb
    return _img_cache[img.name_full]


def upstream_image(sock):
    """First Image Texture node feeding a socket (through Mix/HSV/etc. nodes)."""
    stack, seen = [l.from_node for l in sock.links], set()
    while stack:
        n = stack.pop()
        if n.name in seen:
            continue
        seen.add(n.name)
        if n.type == 'TEX_IMAGE' and n.image:
            return n
        stack += [l.from_node for i in n.inputs for l in i.links]
    return None


def tex_uv_transform(tex):
    """(uv layer name, '' = active; A (2,2); b (2,)) so texture coord = A @ uv + b.
    Follows the Image Texture's Vector input through Reroute and Mapping (Point) nodes to a UV source."""
    A, b, uv_name = np.eye(2), np.zeros(2), ''
    sock = tex.inputs['Vector']
    while sock.is_linked:
        n = sock.links[0].from_node
        if n.type == 'REROUTE':
            sock = n.inputs[0]
        elif n.type == 'MAPPING' and n.vector_type == 'POINT':  # out = Rot(in * Scale) + Loc
            m = np.array(Euler(n.inputs['Rotation'].default_value).to_matrix()) @ np.diag(n.inputs['Scale'].default_value)
            b = A @ np.array(n.inputs['Location'].default_value[:2]) + b  # this node runs before the ones seen so far
            A = A @ m[:2, :2]
            sock = n.inputs['Vector']
        else:  # UV Map node names its layer; Texture Coordinate.UV and anything else -> active UV
            uv_name = n.uv_map if n.type == 'UVMAP' else ''
            break
    return uv_name, A, b


def material_source(mat):
    """Either a (3,) uint8 color or a texture tuple (HxWx3 uint8, uv layer name, A, b)."""
    if mat is None:
        return GREY
    nt = mat.node_tree
    if nt:
        bsdf = next((n for n in nt.nodes if n.type == 'BSDF_PRINCIPLED'), None)
        if bsdf:
            sock = bsdf.inputs['Base Color']
            tex = upstream_image(sock)
            if tex is not None and image_rgb(tex.image) is not None:
                return (image_rgb(tex.image), *tex_uv_transform(tex))
            if not sock.is_linked:
                return srgb8(sock.default_value[:3])
        else:  # e.g. Emission/Diffuse-only materials: first color image in the tree
            for n in nt.nodes:
                if n.type == 'TEX_IMAGE' and n.image and n.image.colorspace_settings.name != 'Non-Color':
                    if image_rgb(n.image) is not None:
                        return (image_rgb(n.image), *tex_uv_transform(n))
    return srgb8(mat.diffuse_color[:3])


def mesh_data(obj):
    """Local-space triangles (T,3,3), {uv layer name: per-corner UVs (T,3,2)} ('' = active),
    material idx (T,), material sources."""
    me = obj.to_mesh()
    try:
        me.calc_loop_triangles()
        nt = len(me.loop_triangles)
        if nt == 0:
            return None
        co = np.empty(len(me.vertices) * 3, np.float32)
        me.vertices.foreach_get('co', co)
        tv = np.empty(nt * 3, np.int32)
        me.loop_triangles.foreach_get('vertices', tv)
        tl = np.empty(nt * 3, np.int32)
        me.loop_triangles.foreach_get('loops', tl)
        mi = np.empty(nt, np.int32)
        me.loop_triangles.foreach_get('material_index', mi)
        tri = co.reshape(-1, 3)[tv].reshape(nt, 3, 3)
        uvs = {}
        for layer in me.uv_layers:
            luv = np.empty(len(me.loops) * 2, np.float32)
            layer.uv.foreach_get('vector', luv)
            uvs[layer.name] = luv.reshape(-1, 2)[tl].reshape(nt, 3, 2)
        if me.uv_layers.active:
            uvs[''] = uvs[me.uv_layers.active.name]
        mats = [material_source(s.material) for s in obj.material_slots] or [GREY]
        return tri, uvs, np.clip(mi, 0, len(mats) - 1), mats
    finally:
        obj.to_mesh_clear()


def sample_instance(data, matrix, dmin, dmax, rng):
    tri, uvs, mi, mats = data
    m = np.array(matrix, np.float32)
    w = tri @ m[:3, :3].T + m[:3, 3]
    area = 0.5 * np.linalg.norm(np.cross(w[:, 1] - w[:, 0], w[:, 2] - w[:, 0]), axis=1)
    lam = area * dmin
    for k, src in enumerate(mats):
        if isinstance(src, tuple) and src[1] in uvs:
            img, uv_name, A, _ = src
            sel = mi == k
            d1, d2 = (uvs[uv_name][sel, j] - uvs[uv_name][sel, 0] for j in (1, 2))
            uv_area = 0.5 * np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0])
            texels = uv_area * abs(np.linalg.det(A)) * img.shape[0] * img.shape[1]  # texels on triangle
            lam[sel] = np.clip(texels, area[sel] * dmin, area[sel] * dmax)
    idx = np.repeat(np.arange(len(w)), rng.poisson(lam))
    if not len(idx):
        return None
    s = np.sqrt(rng.random(len(idx), np.float32))
    r = rng.random(len(idx), np.float32)
    bary = np.stack([1 - s, s * (1 - r), s * r], 1)
    xyz = np.einsum('ij,ijk->ik', bary, w[idx])

    rgb = np.empty((len(idx), 3), np.uint8)
    pm = mi[idx]
    for k, src in enumerate(mats):
        sel = pm == k
        if not sel.any():
            continue
        if not isinstance(src, tuple):
            rgb[sel] = src
        elif src[1] not in uvs:  # textured material but mesh lacks that UV layer
            rgb[sel] = src[0].reshape(-1, 3).mean(0).astype(np.uint8)
        else:
            # ponytail: nearest texel, Repeat extension only, no UDIMs/Generated/Object coords; add if a map uses them
            img, uv_name, A, b = src
            p = np.einsum('ij,ijk->ik', bary[sel], uvs[uv_name][idx[sel]]) @ A.T + b
            h, wd = img.shape[:2]
            x = ((p[:, 0] % 1.0) * wd).astype(np.int64) % wd
            y = ((p[:, 1] % 1.0) * h).astype(np.int64) % h
            rgb[sel] = img[y, x]
    return xyz, rgb


PLY_DTYPE = [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')]


def to_ply_rows(xyz, rgb):
    v = np.empty(len(xyz), PLY_DTYPE)
    v['x'], v['y'], v['z'] = xyz.T
    v['red'], v['green'], v['blue'] = rgb.T
    return v


PCD_DTYPE = [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')]


def write_cloud(path, chunks):
    """Binary .pcd (PCL PointXYZRGB layout, rgb packed 0x00RRGGBB) if path ends in .pcd, else binary .ply."""
    n = sum(map(len, chunks))
    pcd = path.lower().endswith('.pcd')
    with open(path, 'wb') as f:
        if pcd:
            f.write((f'# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z rgb\n'
                     f'SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {n}\nHEIGHT 1\n'
                     f'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n').encode())
        else:
            f.write((f'ply\nformat binary_little_endian 1.0\nelement vertex {n}\n'
                     'property float x\nproperty float y\nproperty float z\n'
                     'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n').encode())
        for v in chunks:
            if pcd:
                p = np.empty(len(v), PCD_DTYPE)
                p['x'], p['y'], p['z'] = v['x'], v['y'], v['z']
                p['rgb'] = (v['red'].astype(np.uint32) << 16) | (v['green'].astype(np.uint32) << 8) | v['blue']
                v = p
            v.tofile(f)


def main():
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True, help='output .pcd or .ply path')
    ap.add_argument('--max-density', type=float, default=100.0, help='points/m^2 cap on textured surfaces')
    ap.add_argument('--min-density', type=float, default=4.0, help='points/m^2 floor, and untextured surfaces')
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args(argv)

    t0 = time.time()
    rng = np.random.default_rng(a.seed)
    dg = bpy.context.evaluated_depsgraph_get()
    cache, chunks = {}, []
    # ponytail: all points held in RAM (15 B/pt) before writing; stream to disk if a map exceeds memory
    for inst in dg.object_instances:
        obj = inst.object
        if obj.type not in TYPES:
            continue
        key = obj.name_full
        if key not in cache:
            cache[key] = mesh_data(obj)
        if cache[key] is None:
            continue
        out = sample_instance(cache[key], inst.matrix_world, a.min_density, a.max_density, rng)
        if out:
            chunks.append(to_ply_rows(*out))

    if not chunks:
        sys.exit('no surfaces found in scene')
    write_cloud(a.out, chunks)
    lo = np.min([[c[k].min() for k in 'xyz'] for c in chunks], 0).round(2)
    hi = np.max([[c[k].max() for k in 'xyz'] for c in chunks], 0).round(2)
    print(f'[blend_to_pcd] {sum(map(len, chunks)):,} points from {len(cache)} objects -> {a.out} '
          f'bbox min {lo} max {hi} ({time.time() - t0:.1f}s)')


main()
