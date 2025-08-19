#!/usr/bin/env python3
"""
Normalize the mesh scale for Shadow Hand IGN URDF by setting mesh scale="0.001 0.001 0.001"
to convert likely millimeter-based collada assets into meters.

Run once to patch robots/shadowhand/shadowhand_ign_shadow_hand_fixed.urdf
"""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / 'robots' / 'shadowhand' / 'shadowhand_ign_shadow_hand_fixed.urdf'

def main():
    if not URDF.exists():
        print(f"URDF not found: {URDF}")
        return
    tree = ET.parse(URDF)
    root = tree.getroot()
    changed = 0
    for mesh in root.iter('mesh'):
        fn = mesh.get('filename') or ''
        if 'meshes/' in fn:
            # Set uniform scale to 0.001 for mm->m conversion
            if mesh.get('scale') != '0.001 0.001 0.001':
                mesh.set('scale', '0.001 0.001 0.001')
                changed += 1
    tree.write(URDF, encoding='utf-8', xml_declaration=True)
    print(f"Updated scale for {changed} mesh entries in {URDF.name}")

if __name__ == '__main__':
    main()


