#!/usr/bin/env python3
"""Generate a deterministic PCD by sampling collision geometry from an SDF."""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ElementTree


def vector(text, length):
    values = [float(value) for value in (text or '').split()]
    if len(values) != length:
        raise ValueError(f'expected {length} values, got {len(values)}')
    return values


def pose(element):
    return vector(element.findtext('pose', '0 0 0 0 0 0'), 6) if element is not None else [0.0] * 6


def compose(parent, child):
    px, py, pz, _, _, pyaw = parent
    cx, cy, cz, _, _, cyaw = child
    cos_yaw, sin_yaw = math.cos(pyaw), math.sin(pyaw)
    return [px + cos_yaw * cx - sin_yaw * cy,
            py + sin_yaw * cx + cos_yaw * cy, pz + cz, 0.0, 0.0, pyaw + cyaw]


def transform(point, origin, spawn_pose):
    x, y, z = point
    ox, oy, oz, _, _, yaw = origin
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    world_x = ox + cos_yaw * x - sin_yaw * y
    world_y = oy + sin_yaw * x + cos_yaw * y
    world_z = oz + z
    spawn_x, spawn_y, _, _, _, spawn_yaw = spawn_pose
    cos_inverse, sin_inverse = math.cos(-spawn_yaw), math.sin(-spawn_yaw)
    map_x = cos_inverse * (world_x - spawn_x) - sin_inverse * (world_y - spawn_y)
    map_y = sin_inverse * (world_x - spawn_x) + cos_inverse * (world_y - spawn_y)
    return (round(map_x, 4), round(map_y, 4), round(world_z, 4))


def samples(lo, hi, spacing):
    count = max(1, int(math.ceil((hi - lo) / spacing)))
    return [lo + (hi - lo) * index / count for index in range(count + 1)]


def add_box(points, origin, size, spacing, spawn_pose):
    sx, sy, sz = size
    x_values = samples(-sx / 2.0, sx / 2.0, spacing)
    y_values = samples(-sy / 2.0, sy / 2.0, spacing)
    z_values = samples(-sz / 2.0, sz / 2.0, spacing)
    for x in x_values:
        for y in y_values:
            for z in (-sz / 2.0, sz / 2.0):
                points.add(transform((x, y, z), origin, spawn_pose))
    for x in x_values:
        for z in z_values:
            for y in (-sy / 2.0, sy / 2.0):
                points.add(transform((x, y, z), origin, spawn_pose))
    for y in y_values:
        for z in z_values:
            for x in (-sx / 2.0, sx / 2.0):
                points.add(transform((x, y, z), origin, spawn_pose))


def add_cylinder(points, origin, radius, length, spacing, spawn_pose):
    circumference_count = max(12, int(math.ceil(2.0 * math.pi * radius / spacing)))
    z_values = samples(-length / 2.0, length / 2.0, spacing)
    for z in z_values:
        for index in range(circumference_count):
            angle = 2.0 * math.pi * index / circumference_count
            points.add(transform(
                (radius * math.cos(angle), radius * math.sin(angle), z), origin, spawn_pose))
    for z in (-length / 2.0, length / 2.0):
        for radius_value in samples(0.0, radius, spacing):
            for index in range(circumference_count):
                angle = 2.0 * math.pi * index / circumference_count
                points.add(transform(
                    (radius_value * math.cos(angle), radius_value * math.sin(angle), z),
                    origin, spawn_pose))


def generate(sdf_path, spacing, spawn_pose):
    root = ElementTree.parse(sdf_path).getroot()
    points = set()
    geometry_count = 0
    for model in root.findall('.//world/model'):
        model_pose = pose(model)
        for link in model.findall('link'):
            link_pose = compose(model_pose, pose(link))
            for collision in link.findall('collision'):
                collision_pose = compose(link_pose, pose(collision))
                geometry = collision.find('geometry')
                if geometry is None:
                    continue
                box = geometry.find('box/size')
                cylinder = geometry.find('cylinder')
                if box is not None:
                    add_box(points, collision_pose, vector(box.text, 3), spacing, spawn_pose)
                    geometry_count += 1
                elif cylinder is not None:
                    radius = float(cylinder.findtext('radius'))
                    length = float(cylinder.findtext('length'))
                    add_cylinder(points, collision_pose, radius, length, spacing, spawn_pose)
                    geometry_count += 1
    if geometry_count == 0 or not points:
        raise ValueError(f'no box/cylinder collision geometry found in {sdf_path}')
    return sorted(points), geometry_count


def main():
    parser = argparse.ArgumentParser(
        description='Sample box/cylinder collision geometry from complex_mapping.sdf into PCD')
    parser.add_argument('--sdf', type=Path, required=True,
                        help='complex_mapping.sdf used as the simulation source of truth')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--spacing', type=float, default=0.10)
    parser.add_argument('--spawn-x', type=float, default=1.75)
    parser.add_argument('--spawn-y', type=float, default=0.0)
    parser.add_argument('--spawn-yaw', type=float, default=math.pi)
    args = parser.parse_args()
    if args.spacing <= 0.0:
        parser.error('--spacing must be positive')
    spawn_pose = [args.spawn_x, args.spawn_y, 0.0, 0.0, 0.0, args.spawn_yaw]
    ordered, geometry_count = generate(args.sdf, args.spacing, spawn_pose)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', encoding='ascii') as stream:
        stream.write('# .PCD v0.7 - Point Cloud Data file format\n')
        stream.write('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
        stream.write(f'WIDTH {len(ordered)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n')
        stream.write(f'POINTS {len(ordered)}\nDATA ascii\n')
        for x, y, z in ordered:
            stream.write(f'{x:.4f} {y:.4f} {z:.4f}\n')
    print(
        f'wrote {len(ordered)} points from {geometry_count} SDF collision geometries '
        f'to {args.output}')


if __name__ == '__main__':
    main()
