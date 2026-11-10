# Top-down color preview of a .pcd/.ply written by blend_to_pcd.py (highest point wins per pixel).
# blender -b --python preview_pcd.py -- in.pcd out.png [--res 0.2] [--crop cx cy r]
#   --res: meters per pixel (default 0.2; with ~100 pts/m^2 below ~0.15 leaves empty pixels)
#   --crop: square of half-size r meters around (cx, cy)
import argparse, os, sys, bpy, numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('src')
ap.add_argument('dst')
ap.add_argument('--res', type=float, default=0.2)
ap.add_argument('--crop', type=float, nargs=3)
a = ap.parse_args(sys.argv[sys.argv.index('--') + 1:])

V = [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', 'u1'), ('g', 'u1'), ('b', 'u1')]
with open(a.src, 'rb') as f:
    if a.src.lower().endswith('.pcd'):  # x y z f4 + rgb packed u4
        while not f.readline().startswith(b'DATA'):
            pass
        p = np.fromfile(f, [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
        v = np.empty(len(p), V)
        v['x'], v['y'], v['z'] = p['x'], p['y'], p['z']
        v['r'], v['g'], v['b'] = p['rgb'] >> 16, p['rgb'] >> 8, p['rgb']
        del p
    else:
        while f.readline().strip() != b'end_header':
            pass
        v = np.fromfile(f, V)
if a.crop:
    cx, cy, r = a.crop
    v = v[(abs(v['x'] - cx) < r) & (abs(v['y'] - cy) < r)]
print('points', len(v))

x, y = v['x'], v['y']
ix = ((x - x.min()) / a.res).astype(np.int64)
iy = ((y - y.min()) / a.res).astype(np.int64)
W, H = ix.max() + 1, iy.max() + 1
o = np.argsort(v['z'])  # later (higher) points overwrite lower ones
img = np.zeros((H, W, 4), np.float32)
img[..., 3] = 1
img[iy[o], ix[o], :3] = np.stack([v['r'], v['g'], v['b']], 1)[o] / 255.0
print(f'image {W}x{H} px at {a.res} m/px, empty pixels {np.mean(img[..., :3].sum(2) == 0):.1%}')
im = bpy.data.images.new('preview', int(W), int(H))
im.colorspace_settings.name = 'Non-Color'
im.pixels.foreach_set(img.ravel())
im.filepath_raw = os.path.abspath(a.dst)
im.file_format = 'PNG'
im.save()
