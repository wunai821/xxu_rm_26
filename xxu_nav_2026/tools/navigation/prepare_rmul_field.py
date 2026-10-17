#!/usr/bin/env python3
"""Convert the supplied RMUL STL to metres/Z-up and a conservative Nav2 map.

The source's large plane at Y=420 mm is the playing surface; Y=520 mm is
the underside. Use a proper rotation, never a reflection: (X,Z,420-Y)/1000.
The input is read-only. The output preserves all collision triangles.
"""

import argparse
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml

from generate_rmuc_map import clip_height


STL_DTYPE = np.dtype([
    ('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])


def prepare(source, output, collision_top=0.50):
    if not np.isfinite(collision_top) or collision_top <= 0.05:
        raise ValueError('collision_top must be finite and above the 0.05 m lower band')
    raw = source.read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0]
    if len(raw) != 84 + count * 50 or not count:
        raise ValueError('Expected a complete, nonempty binary STL')
    records = np.frombuffer(raw, dtype=STL_DTYPE, offset=84)
    original = records['vertices'].astype(float)
    if not np.isfinite(original).all():
        raise ValueError('Nonfinite source coordinates')
    if not np.allclose([original.min((0, 1)), original.max((0, 1))],
                       [[0, 0, 0], [12600, 520, 8600]], atol=0.01):
        raise ValueError('This transform is specific to the supplied rmul_2027.stl')
    triangles = original[:, :, [0, 2, 1]] * 0.001
    triangles[:, :, 2] = 0.42 - triangles[:, :, 2]
    normal = np.cross(triangles[:, 1] - triangles[:, 0],
                      triangles[:, 2] - triangles[:, 0])
    length = np.linalg.norm(normal, axis=1)
    normal /= np.maximum(length[:, None], 1e-20)
    converted = np.zeros(count, dtype=STL_DTYPE)
    converted['vertices'], converted['normal'] = triangles, normal
    meshes = output / 'meshes'
    meshes.mkdir(parents=True, exist_ok=True)
    mesh_path = meshes / 'rmul_2027_m_zup.stl'
    mesh_path.write_bytes(b'RMUL2027 metres Z-up; ground=0'.ljust(80, b'\0') +
                          struct.pack('<I', count) + converted.tobytes())

    sdf = ET.fromstring('''<sdf version="1.9"><model name="rmul_2027">
      <static>true</static><link name="field">
        <visual name="field"><geometry><mesh><uri>meshes/rmul_2027_m_zup.stl</uri>
          </mesh></geometry><material><ambient>0.32 0.35 0.39 1</ambient>
          <diffuse>0.55 0.59 0.64 1</diffuse><specular>0.03 0.03 0.03 1</specular>
          </material></visual>
        <collision name="field"><geometry><mesh><uri>meshes/rmul_2027_m_zup.stl</uri>
          </mesh></geometry></collision>
      </link></model></sdf>''')
    ET.ElementTree(sdf).write(output / 'model.sdf', encoding='utf-8', xml_declaration=True)
    (output / 'model.config').write_text('''<?xml version="1.0"?>
<model><name>rmul_2027</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>
<description>User-supplied field; metres, Z-up, playing surface at z=0.</description></model>
''')

    resolution = 0.05
    origin = np.array([-0.5, -0.5])
    size = np.array([272, 192])
    image = np.full((size[1], size[0]), 205, dtype=np.uint8)
    x = origin[0] + (np.arange(size[0]) + 0.5) * resolution
    y = origin[1] + (np.arange(size[1]) + 0.5) * resolution
    inside = ((x >= 0) & (x < 12.6))[None, :] & (
        (y >= 0) & (y < 8.6))[::-1, None]
    image[inside] = 254
    selected = triangles[(triangles[:, :, 2].max(1) >= 0.05) &
                         (triangles[:, :, 2].min(1) <= collision_top)]
    for triangle in selected:
        polygon = clip_height(list(triangle), 0.05, True)
        polygon = clip_height(polygon, collision_top, False)
        if polygon:
            pixels = np.rint((np.array(polygon)[:, :2] - origin) / resolution).astype(np.int32)
            pixels[:, 1] = size[1] - 1 - pixels[:, 1]
            cv2.fillPoly(image, [pixels], 0)
    maps = output / 'navigation'
    maps.mkdir(exist_ok=True)
    if not cv2.imwrite(str(maps / 'rmul_2027.pgm'), image):
        raise RuntimeError('Map image write failed')
    (maps / 'rmul_2027.yaml').write_text(yaml.safe_dump({
        'image': 'rmul_2027.pgm', 'mode': 'trinary', 'resolution': resolution,
        'origin': [*origin.tolist(), 0.0], 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.196}, sort_keys=False))
    result = {
        'source_file': source.name, 'source_sha256': hashlib.sha256(raw).hexdigest(),
        'triangles': count, 'source_bounds': [original.min((0, 1)).tolist(), original.max((0, 1)).tolist()],
        'transform': 'x=X/1000; y=Z/1000; z=(420-Y)/1000',
        'rotation_determinant': 1, 'units': 'metre', 'ground_z': 0.0,
        'bounds_m': [triangles.min((0, 1)).tolist(), triangles.max((0, 1)).tolist()],
        'map_resolution': resolution, 'collision_height_band_m': [0.05, collision_top],
        'map_type': 'conservative geometry projection, not measured SLAM',
    }
    (output / 'conversion.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--collision-top', type=float, default=0.50,
                        help='Projection band top in metres, including vehicle height margin; '
                             'a lower candidate map does not validate vehicle clearance')
    args = parser.parse_args()
    prepare(args.source, args.output, args.collision_top)
