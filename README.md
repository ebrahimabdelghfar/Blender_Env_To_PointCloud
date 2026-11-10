# blender_design_to_pcd

Converts a Blender map/environment (`.blend`) into a colored point cloud (binary PLY) for drawing HD maps (Lanelet2 / Vector Map Builder) or for visual checks.

## Files

- `blend_to_pcd.py`: the converter
- `preview_pcd.py`: renders a top-down PNG of a PLY, to check colors

## Requirements

- Blender 4.x or newer (tested with 5.2.2, `/snap/bin/blender`). Nothing else: the script uses Blender's bundled Python and numpy.

## Usage

```bash
blender -b map.blend --python blend_to_pcd.py -- --out map.pcd [--max-density 100] [--min-density 4] [--seed 0]
```

| Option | Default | Meaning |
|---|---|---|
| `--out` | (required) | Output `.pcd` or `.ply` path |
| `--max-density` | 100 | Most points per m² on textured surfaces (100 means about 10 cm spacing) |
| `--min-density` | 4 | Fewest points per m², also used for untextured surfaces (about 0.5 m spacing) |
| `--seed` | 0 | Random seed, so repeated runs give the same cloud |

For one uniform density everywhere, set both to the same value, e.g. `--min-density 20 --max-density 20`.

Output format follows the `--out` extension:
- `.pcd`: binary PCD v0.7, fields `x y z rgb`, the same layout as PCL's `PointXYZRGB` (rgb packed as `0x00RRGGBB`). Autoware, PCL and CloudCompare load it directly.
- `.ply`: binary little-endian PLY, `x y z` (float) plus `red green blue` (uint8).

Coordinates are in meters in the Blender world frame (right-handed, Z up), so the cloud's (0, 0, 0) is the .blend's world origin. Colors are sRGB.

## How it works

1. Goes through every visible object in the evaluated scene (modifiers, collection instances and Geometry Nodes instances applied). Object types: mesh, curve, surface, text, metaball.
2. Splits each object into triangles and places random points on them, in proportion to area.
3. **Density per triangle = the texture's resolution on it (texture pixels per m²), clamped to `[min, max]`.** More points than texture pixels would only repeat the same colors. Ground with a coarse aerial photo gets few points, while asphalt and kerbs with fine textures get the full `max`.
4. Point color comes from the triangle's material, in this order:
   - the image texture feeding Principled BSDF **Base Color**, looked up at the nearest texture pixel. Mix and Hue/Saturation nodes in between are followed. The texture's UV input is honored: Mapping nodes (Point type: scale, rotation, location, i.e. tiling) and UV Map nodes that pick a named layer, otherwise the active UV map.
   - otherwise, the Base Color value
   - otherwise, the material's viewport color (no material: grey)

## Choosing density for HD-map drawing

Silverstone (`silverstone.blend`, about 7.4 km² of surface), 32-core / 62 GB machine:

| `--max-density` | Spacing | Points | File size | Notes |
|---|---|---|---|---|
| 2 (uniform) | ~70 cm | 15M | 0.2 GB | Overview only |
| **100 (default)** | ~10 cm | 222M | 3.3 GB | Lane edges and kerbs readable. 28 s, 7.8 GB RAM |
| 400 | ~5 cm | ~629M (est.) | ~9.4 GB | Mostly repeated grass-texture pixels, too heavy |

Each point takes 15 bytes in the file and in RAM. Autoware recommends 0.1–0.2 m voxels for point-cloud maps and downsampling above about 300 MB, so tile or crop large maps before loading them in a web editor.

## Limitations

- Textures always repeat (Repeat extension). Multi-tile (UDIM) textures, and Generated/Object texture coordinates, are not supported: they fall back to the UV map.
- Only the first image texture behind Base Color is used. Procedural textures (Noise, Voronoi…) are not evaluated.
- Points are random (Poisson) samples, not a simulated lidar scan. There is no intensity channel, and the cloud includes surfaces a sensor could not see.
- The whole cloud is held in RAM before writing (15 bytes per point).
- Uses the scene's viewport visibility: hide anything you don't want in the map (e.g. vehicles) before running.

## Checking the output

Top-down color render of a `.pcd` or `.ply` (the highest point wins each pixel):

```bash
blender -b --python preview_pcd.py -- map.pcd out.png [--res 0.2] [--crop cx cy r]
```

- `--res`: meters per pixel (default 0.2). At about 100 points/m², values below 0.15 leave empty pixels.
- `--crop`: square of half-size `r` meters around `(cx, cy)`.

Or open it in CloudCompare: `cloudcompare map.pcd`.
