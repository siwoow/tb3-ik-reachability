"""Create a portable bundle without Isaac, meshes, environments or source edits."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--planner-root', default='~/tb3_grasp_analysis')
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    planner = Path(args.planner_root).expanduser()
    bundle = root/'dist/tb3_grasp_bridge'
    bundle.mkdir(parents=True, exist_ok=True)
    for name in ('tb3_grasp_ros2',):
        shutil.copytree(root/name, bundle/name, dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    for name in ('audit_planner.py', 'build_native.py', 'native_probe.cpp', 'executor_contract.json', 'requirements-jetson.txt', 'DEPLOYMENT.md'):
        shutil.copy2(root/name, bundle/name)
    if (root/'dist/wheels').exists():
        shutil.copytree(root/'dist/wheels', bundle/'wheels', dirs_exist_ok=True)
    runtime = bundle/'planner_runtime'
    shutil.copytree(planner/'tb3_grasp', runtime/'tb3_grasp', dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    (runtime/'assets').mkdir(exist_ok=True)
    for name in ('resolved.urdf', 'collision_bounds.json'):
        shutil.copy2(planner/'assets'/name, runtime/'assets'/name)
    (runtime/'results').mkdir(exist_ok=True)
    for name in ('workspace_summary.json', 'grasp_workspace.csv'):
        shutil.copy2(planner/'results'/name, runtime/'results'/name)
    license_file = planner/'vendor/turtlebot3_manipulation/LICENSE'
    if license_file.exists():
        shutil.copy2(license_file, runtime/'ROBOTIS_LICENSE')
    manifest = {str(p.relative_to(bundle)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in bundle.rglob('*') if p.is_file() and p.name != 'bundle_manifest.json'}
    (bundle/'bundle_manifest.json').write_text(json.dumps(manifest, indent=2))
    out = root/'dist/tb3_grasp_bridge.tar.gz'
    with tarfile.open(out, 'w:gz') as archive:
        archive.add(bundle, arcname=bundle.name)
    print(out)


if __name__ == '__main__':
    main()
