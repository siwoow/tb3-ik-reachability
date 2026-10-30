"""Compile a CPU-only IK adapter against unchanged, fingerprinted headers."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def main():
    root = Path(__file__).resolve().parent
    p = argparse.ArgumentParser()
    p.add_argument('--source-root', default='~/turtlebot3_ws/src/pick_and_place')
    p.add_argument('--output', default=str(root/'native_probe'))
    args = p.parse_args()
    source = Path(args.source_root).expanduser()
    contract = json.loads((root/'executor_contract.json').read_text())
    for relative, expected in contract['executor_header_sha256'].items():
        if hashlib.sha256((source/relative).read_bytes()).hexdigest() != expected:
            raise RuntimeError('Original executor header changed; audit before rebuilding: '+relative)
    output = Path(args.output).expanduser().resolve()
    subprocess.run(['g++', '-std=c++17', '-O2', '-I', str(source/'include'),
                    str(root/'native_probe.cpp'), '-o', str(output)], check=True)
    manifest = {'binary_sha256': hashlib.sha256(output.read_bytes()).hexdigest(),
                'adapter_source_sha256': hashlib.sha256((root/'native_probe.cpp').read_bytes()).hexdigest(),
                'executor_header_sha256': contract['executor_header_sha256']}
    output.with_suffix('.manifest.json').write_text(json.dumps(manifest, indent=2))
    print(output)


if __name__ == '__main__':
    main()
