#!/usr/bin/env python3
"""Rasterize rm_sim_26's field collision mesh for a planar navigation map.

This is a simulation geometry map, not a measured SLAM map. Clip triangles to
the chassis collision band before projecting, so low barriers remain blocked
while the floor and overhead mesh do not fill the traversable field.
"""

import argparse
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml


def clip_height(polygon, height, keep_above):
    result = []
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        inside_start = start[2] >= height if keep_above else start[2] <= height
        inside_end = end[2] >= height if keep_above else end[2] <= height
        if inside_start:
            result.append(start)
        if inside_start != inside_end:
            result.append(start + (end-start) * ((height-start[2]) / (end[2]-start[2])))
    return result


def generate(simulator, output, resolution=0.05, ground_height=0.2):
    field = simulator / 'models/rmuc_2025'
    root = ET.parse(field / 'model.sdf').getroot()
    link = root.find('model/link')
    pose = np.array([float(x) for x in link.findtext('pose').split()])
    if np.any(pose[3:] != 0):
        raise ValueError('Rotated field pose requires an explicit transform')
    mesh = link.find('collision/geometry/mesh')
    uri = mesh.findtext('uri')
    path = simulator / 'models' / uri.removeprefix('model://')
    raw = path.read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0]
    if len(raw) != 84 + count*50:
        raise ValueError('Expected a complete binary STL')
    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    triangles = np.frombuffer(raw, dtype=dtype, offset=84)['vertices'].astype(float)
    triangles *= np.array([float(x) for x in mesh.findtext('scale', '1 1 1').split()])
    triangles += pose[:3]
    origin = np.floor(triangles.min(axis=(0, 1))[:2]) - 1
    extent = np.ceil(triangles.max(axis=(0, 1))[:2]) + 1
    size = np.ceil((extent-origin)/resolution).astype(int)
    image = np.full((size[1], size[0]), 254, dtype=np.uint8)
    bounds_min = triangles.min(axis=(0, 1))[:2]
    bounds_max = triangles.max(axis=(0, 1))[:2]
    x = origin[0] + (np.arange(size[0])+0.5)*resolution
    y = origin[1] + (np.arange(size[1])+0.5)*resolution
    outside = ((x < bounds_min[0]) | (x > bounds_max[0]))[None, :] | (
        (y < bounds_min[1]) | (y > bounds_max[1]))[::-1, None]
    image[outside] = 205
    # The playable floor is world z=0.2 in this pinned field (the robot
    # settles at base_link z=0.296). Include low physical barriers, not just
    # obstacles intersecting the old robot's elevated scan slice.
    lower, upper = ground_height + 0.05, ground_height + 0.50
    selected = triangles[(triangles[:, :, 2].max(axis=1) >= lower) &
                         (triangles[:, :, 2].min(axis=1) <= upper)]
    for triangle in selected:
        polygon = clip_height(list(triangle), lower, True)
        polygon = clip_height(polygon, upper, False)
        if polygon:
            pixels = np.rint((np.array(polygon)[:, :2]-origin)/resolution).astype(np.int32)
            pixels[:, 1] = size[1] - 1 - pixels[:, 1]
            cv2.fillPoly(image, [pixels], 0)
    image[[0, -1], :] = 0
    image[:, [0, -1]] = 0
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output.with_suffix('.pgm')), image):
        raise RuntimeError('Map image write failed')
    output.with_suffix('.yaml').write_text(yaml.safe_dump({
        'image': output.with_suffix('.pgm').name, 'mode': 'trinary',
        'resolution': resolution, 'origin': [float(origin[0]), float(origin[1]), 0.0],
        'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196,
    }, sort_keys=False))
    print(f'{count} collision triangles, {len(selected)} in scan band; '
          f'map {size[0]}x{size[1]}, origin {origin.tolist()}, world z={lower}..{upper}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('simulator', type=Path)
    parser.add_argument('output', type=Path, help='Output basename without extension')
    args = parser.parse_args()
    generate(args.simulator, args.output)
