#!/usr/bin/env python3
"""
Prepare Shadow Hand meshes for the IGN full URDF by:
- Creating meshes/visual symlinks to existing components meshes
- Rewriting collision .stl paths in URDF to use the corresponding visual .dae paths

This avoids missing mesh errors and enables visualization in SAPIEN.
"""
import os
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SH_DIR = ROOT / 'robots' / 'shadowhand'
URDF = SH_DIR / 'shadowhand_ign_shadow_hand_fixed.urdf'
VIS_DIR = SH_DIR / 'meshes' / 'visual'

MAPPING = {
    'forearm.dae': 'components/forearm/forearm_E3M5.dae',
    'wrist.dae': 'components/wrist/wrist_E3M5.dae',
    'palm.dae': 'components/palm/palm_E3M5.dae',
    'thumb_proximal.dae': 'components/th_proximal/th_proximal_E3M5.dae',
    'thumb_middle.dae': 'components/th_middle/th_middle_E3M5.dae',
    'thumb_distal.dae': 'components/th_distal/mst/th_distal_mst.dae',
    'knuckle.dae': 'components/f_knuckle/f_knuckle_E3M5.dae',
    'finger_proximal.dae': 'components/f_proximal/f_proximal_E3M5.dae',
    'finger_middle.dae': 'components/f_middle/f_middle_E3M5.dae',
    'finger_distal.dae': 'components/f_distal/mst/f_distal_mst.dae',
    'metacarpal.dae': 'components/lf_metacarpal/lf_metacarpal_E3M5.dae',
}

def ensure_symlinks():
    (SH_DIR / 'meshes').mkdir(exist_ok=True)
    VIS_DIR.mkdir(parents=True, exist_ok=True)
    created = []
    for name, rel_target in MAPPING.items():
        target = SH_DIR / 'meshes' / rel_target
        link = VIS_DIR / name
        if not target.exists():
            print(f"[warn] Target not found for {name}: {target}")
            continue
        if link.exists() or link.is_symlink():
            try:
                if link.resolve() == target.resolve():
                    continue
                link.unlink()
            except Exception:
                try:
                    link.unlink(missing_ok=True)
                except Exception:
                    pass
        link.parent.mkdir(parents=True, exist_ok=True)
        os.symlink(os.path.relpath(target, link.parent), link)
        created.append((name, link, target))
    print(f"Created/verified {len(created)} visual symlinks.")

def rewrite_urdf_collision_to_visual():
    if not URDF.exists():
        print(f"URDF not found: {URDF}")
        return
    tree = ET.parse(URDF)
    root = tree.getroot()
    changes = 0
    for mesh in root.iter('mesh'):
        fn = mesh.get('filename')
        if not fn:
            continue
        # Normalize path
        p = Path(fn)
        parts = list(p.parts)
        if 'meshes' in parts:
            idx = parts.index('meshes')
            sub = parts[idx+1] if idx+1 < len(parts) else ''
            if sub == 'collision':
                # Replace collision/* with visual/* and force .dae
                base = Path(parts[-1]).stem  # name without suffix
                new_rel = Path('meshes') / 'visual' / f"{base}.dae"
                mesh.set('filename', str(new_rel))
                changes += 1
    out = URDF  # in-place
    tree.write(out, encoding='utf-8', xml_declaration=True)
    print(f"Rewrote {changes} collision mesh paths to visual .dae in URDF.")

def main():
    ensure_symlinks()
    rewrite_urdf_collision_to_visual()
    print("Done preparing Shadow Hand meshes.")

if __name__ == '__main__':
    main()


