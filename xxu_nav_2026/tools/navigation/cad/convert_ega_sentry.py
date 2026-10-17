#!/usr/bin/env python3
"""Export the EGA sentry's saved display geometry, with explicit coverage.

Requires the pinned sldprt-export checkout with sldprt-export-ega.patch applied.
This produces a visual model, not a dynamics model or a material measurement.
Source CAD is read-only. No network access or external CAD service is used.
"""

import argparse
import dataclasses
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import trimesh


def material_for(name):
    # Display styling only. These names do not establish material density.
    if any(word in name.lower() for word in ('mid360', '裁判', '装甲', '滚子')):
        return 'ivory', [210, 216, 218, 255], 0.1
    if any(word in name for word in ('螺钉', '螺栓', '螺母', '垫片', '轴承', '转接')):
        return 'steel', [122, 132, 143, 255], 0.75
    if any(word in name for word in ('轮胎', '橡胶')):
        return 'rubber', [24, 27, 31, 255], 0.0
    return 'black', [52, 58, 68, 255], 0.3


def convert(source, reader, output):
    sys.path.insert(0, str(reader / 'src'))
    from sldprt.blocks import read_blocks
    from sldprt.assembly import read_assembly

    output.mkdir(parents=True, exist_ok=True)
    main = next(source.rglob('底盘总装V2-哨ver-0517.SLDASM'))
    index = {p.name.lower(): p for p in source.rglob('*')
             if p.suffix.lower() in ('.sldasm', '.sldprt')}
    data = main.read_bytes()
    blocks = read_blocks(data)
    crc = {b.name: b.verify() for b in blocks
           if 'FaceTessellations' in b.name or 'COMPINSTANCETREE' in b.name}
    if not all(crc.values()):
        raise ValueError('Assembly display data failed CRC verification')
    combined, components, report = read_assembly(
        blocks, resolve=lambda n: index[n.lower()].read_bytes() if n.lower() in index else None)
    vertices = np.asarray(combined.vertices)
    bounds = np.array([vertices.min(0), vertices.max(0)])
    if np.max(np.ptp(vertices, axis=0)) > 900:
        raise ValueError('Implausible robot size; inspect cached mesh extraction')
    ground_y = bounds[0, 1]
    del combined, vertices
    scene = trimesh.Scene()
    parts = []
    raw_meshes = {}
    for i, component in enumerate(components):
        if component.hidden:
            continue
        points = np.asarray(component.mesh.vertices, dtype=np.float32)
        faces = np.asarray(component.mesh.triangles, dtype=np.uint32)
        raw_meshes[f'v{i}'], raw_meshes[f'f{i}'] = points, faces
        # SW Y-up -> ROS Z-up, with a right-handed rotation and ground at 0.
        points = points[:, [0, 2, 1]].copy() * 0.001
        points[:, 1] *= -1
        points[:, 2] -= ground_y * 0.001
        mesh = trimesh.Trimesh(points, faces, process=False)
        excluded_reference_triangles = 0
        if component.name.rsplit('/', 1)[-1].startswith('定位模块UW01-'):
            # Its own SW thumbnail shows a large sphere and plane around the
            # receiver: installation/reference envelopes, not robot surfaces.
            # Keep the small receiver bodies; retain raw CAD data in the NPZ.
            bodies = mesh.split(only_watertight=False)
            physical = [body for body in bodies if np.max(body.extents) < 0.1]
            if not physical:
                raise ValueError('UWB reference-envelope filtering removed the receiver')
            mesh = trimesh.util.concatenate(physical)
            excluded_reference_triangles = len(faces) - len(mesh.faces)
        # Keep the saved triangles. Aggressive decimation damaged thin slotted
        # sheet parts in the visual comparison and is unsuitable for this CAD.
        # Preserve hard CAD edges; averaging normals across sheet edges makes
        # flat panels look dented even when their coordinates are unchanged.
        mesh = trimesh.graph.smooth_shade(mesh, angle=np.radians(20))
        label, color, metallic = material_for(component.name)
        mesh.visual = trimesh.visual.TextureVisuals(
            material=trimesh.visual.material.PBRMaterial(
                name=label, baseColorFactor=color, metallicFactor=metallic,
                roughnessFactor=0.55, doubleSided=True))
        scene.add_geometry(mesh, node_name=component.name, geom_name=f'part_{i}')
        parts.append({'index': i, 'name': component.name, 'mirrored': component.mirrored,
                      'source_triangles': len(faces), 'visual_triangles': len(mesh.faces),
                      'excluded_reference_triangles': excluded_reference_triangles,
                      'display_material': label})
    np.savez_compressed(output / 'sentry_components_mm.npz', **raw_meshes)
    model_path = output / 'ega_sentry_recolored_draft.glb'
    scene.export(model_path)
    result = {
        'source_assembly': main.name, 'source_sha256': hashlib.sha256(data).hexdigest(),
        'coverage': dataclasses.asdict(report), 'display_crc': crc,
        'raw_display_bounds_mm': bounds.tolist(), 'visual_bounds_m': scene.bounds.tolist(),
        'units': 'metre',
        'transform': f'x=X/1000; y=-Z/1000; z=(Y-({ground_y}))/1000',
        'source_triangles': sum(p['source_triangles'] for p in parts),
        'visual_triangles': sum(p['visual_triangles'] for p in parts),
        'visual_parts': len(parts), 'glb_bytes': model_path.stat().st_size,
        'status': 'visual draft; unresolved parts listed; no joint/inertia validation',
        'material_basis': 'name-based display style; no density or mass inferred',
        'reference_geometry': 'UWB sphere/plane excluded using its embedded SW preview; raw mesh retained',
        'reader': {
            'repository': 'https://github.com/XRTC5/sldprt-export',
            'commit': 'ebde7524b3876c5d78c7e2248f2eba9b5725a6c0',
            'patch_sha256': hashlib.sha256(Path(__file__).with_name(
                'sldprt-export-ega.patch').read_bytes()).hexdigest(),
        },
        'parts': parts,
    }
    (output / 'sentry_decode.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps({k: v for k, v in result.items() if k not in ('parts', 'display_crc')},
                     ensure_ascii=False, indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path, help='Extracted CAD directory')
    parser.add_argument('reader', type=Path, help='Patched sldprt-export checkout')
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    convert(args.source, args.reader, args.output)
