#!/usr/bin/env python3
"""Export exact STEP BRep moments for catalog component mass replacement.

Offline build utility, requiring cadquery-ocp and numpy in an isolated Python
CAD environment. Runtime simulation only reads the generated numeric table.
The STEP and downloaded CAD assets are never modified by this script.
"""
from __future__ import annotations

import argparse
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer


def parallel_axis(center):
    return float(center @ center) * np.eye(3) - np.outer(center, center)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--step', required=True, type=Path)
    parser.add_argument('--assets', default=Path('assets/cad'), type=Path)
    parser.add_argument('--output', default=Path('models/cad_mass_properties.json'), type=Path)
    args = parser.parse_args()
    manifest_path = args.assets / 'cad-manifest.json'
    manifest = json.loads(manifest_path.read_text())
    source_bytes = args.step.read_bytes()
    lf_digest = hashlib.sha256(source_bytes).hexdigest()
    crlf_digest = hashlib.sha256(source_bytes.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')).hexdigest()
    if manifest['source_sha256'] not in (lf_digest, crlf_digest):
        raise ValueError('STEP content does not match the locked CAD manifest')
    reader = STEPControl_Reader()
    if reader.ReadFile(str(args.step)) != IFSelect_RetDone:
        raise ValueError('OpenCascade could not read STEP')
    reader.TransferRoots()
    explorer = TopExp_Explorer(reader.OneShape(), TopAbs_SOLID)
    solids = []
    while explorer.More():
        solids.append(explorer.Current())
        explorer.Next()
    if len(solids) != manifest['step_solids']:
        raise ValueError(f'Expected {manifest["step_solids"]} STEP solids, got {len(solids)}')
    rotation = np.array([[-1., 0., 0.], [0., 0., 1.], [0., 1., 0.]])
    origin_mm = np.array([0., 32., 15.5])
    anchors = {'chassis': np.zeros(3)}
    for side, sign in (('L', 1), ('R', -1)):
        for body, anchor in (('upper', 'A'), ('lower', 'B'), ('wheel', 'W'), ('rod', 'C')):
            anchors[f'{side}_{body}'] = np.asarray(manifest['anchors_robot_m'][anchor]) * [1, sign, 1]
    parts = []
    for part in sorted(manifest['parts'], key=lambda x: x['solid']):
        properties = GProp_GProps()
        BRepGProp.VolumeProperties_s(solids[part['solid']], properties)
        volume = float(properties.Mass())
        relative_error = volume / part['volume_mm3'] - 1
        if abs(relative_error) > 1e-7:
            raise ValueError(f'Solid ordering/volume mismatch for {part["solid"]}: {relative_error}')
        cad_center = np.array(properties.CentreOfMass().Coord())
        tensor = properties.MatrixOfInertia()
        cad_inertia = np.array([[tensor.Value(i + 1, j + 1) for j in range(3)] for i in range(3)])
        center = (rotation @ cad_center - origin_mm) * .001 - anchors[part['body']]
        # OpenCascade density=1: mm^5 / volume(mm^3) -> inertia per kg in mm^2.
        unit_inertia = rotation @ cad_inertia @ rotation.T / volume * 1e-6
        parts.append({'solid': part['solid'], 'body': part['body'], 'mesh': part['file'],
                      'reference_mass_kg': part['mass_kg_estimated'], 'cad_volume_m3': volume * 1e-9,
                      'centroid_body_m': center.tolist(), 'inertia_per_kg_body_m2': unit_inertia.tolist(),
                      'volume_relative_error_vs_manifest': relative_error})
    comparisons = []
    source_xml = ET.parse(args.assets / 'mushibot.xml').getroot()
    for body in sorted({p['body'] for p in parts}):
        rows = [p for p in parts if p['body'] == body]
        mass = sum(p['reference_mass_kg'] for p in rows)
        center = sum(p['reference_mass_kg'] * np.asarray(p['centroid_body_m']) for p in rows) / mass
        inertia = sum(p['reference_mass_kg'] * (np.asarray(p['inertia_per_kg_body_m2']) +
                       parallel_axis(np.asarray(p['centroid_body_m']))) for p in rows) - mass * parallel_axis(center)
        element = source_xml.find(f".//body[@name='{body}']/inertial")
        old_center = np.fromstring(element.attrib['pos'], sep=' ')
        raw = np.fromstring(element.attrib['fullinertia'], sep=' ')
        old_inertia = np.array([[raw[0],raw[3],raw[4]], [raw[3],raw[1],raw[5]], [raw[4],raw[5],raw[2]]])
        center_error = float(np.linalg.norm(center - old_center))
        inertia_error = float(np.linalg.norm(inertia - old_inertia) / np.linalg.norm(old_inertia))
        if center_error > 1e-8 or inertia_error > 1e-6:
            raise ValueError(f'Exact BRep reconstruction disagrees for {body}: {center_error}, {inertia_error}')
        comparisons.append({'body': body, 'centroid_error_m': center_error,
                            'inertia_relative_frobenius_error': inertia_error})
    table = {
        'schema_version': 1,
        'method': 'OpenCascade BRepGProp.VolumeProperties on original STEP; no visual-mesh or AABB inertia approximation',
        'evidence': {
            'step_file': args.step.name, 'step_sha256': lf_digest,
            'step_sha256_normalized_crlf': crlf_digest,
            'manifest_sha256': hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
            'mjcf_sha256': hashlib.sha256((args.assets / 'mushibot.xml').read_bytes()).hexdigest(),
            'cadquery_ocp_version': version('cadquery-ocp'),
            'source_builder': 'https://github.com/dycsrl/Micro-Wheeled_leg-Robot-Model/blob/a330459b2d3e4567974bb6415abb51086fa5d683/scripts/build_wl_model.py',
        },
        'body_reconstruction_check': comparisons,
        'parts': parts,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(table, indent=2) + '\n')
    print(json.dumps({'output': str(args.output), 'solids': len(solids), 'exported_parts': len(parts),
                      'body_reconstruction_check': comparisons}, indent=2))


if __name__ == '__main__':
    main()
