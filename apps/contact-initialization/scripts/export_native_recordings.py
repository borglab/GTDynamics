#!/usr/bin/env python3
"""Record real C++ GTSAM solves and repeated native timings for static replay.

This script deliberately strips the held-out root reference before invoking the
native executable. It does not implement initialization or optimization.
"""
import argparse
import hashlib
import json
import platform
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def main():
    app = Path(__file__).resolve().parents[1]
    repo = app.parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--optimizer', type=Path, required=True)
    parser.add_argument('--sequences', type=Path, default=app / 'public/sequences.json')
    parser.add_argument('--sequence', action='append', help='Only record these sequence IDs')
    parser.add_argument('--repetitions', type=int, default=11)
    parser.add_argument('--warmup', type=int, default=2)
    parser.add_argument('--iterations', type=int, default=100)
    args = parser.parse_args()
    if args.repetitions < 3 or args.warmup < 0 or args.iterations < 1:
        parser.error('Require at least 3 repetitions, nonnegative warmup, and positive iterations.')
    registry = json.loads(args.sequences.read_text())
    revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
    status = subprocess.check_output(['git', 'status', '--porcelain', '--untracked-files=no'], cwd=repo, text=True)
    cpu = platform.processor() or platform.machine()
    cpuinfo = Path('/proc/cpuinfo')
    if cpuinfo.exists():
        cpu = next((line.split(':', 1)[1].strip() for line in cpuinfo.read_text().splitlines()
                    if line.startswith('model name')), cpu)
    native_source_files = [repo / 'gtdynamics/utils/ContactInitializer.h',
                           repo / 'gtdynamics/utils/ContactInitializer.cpp',
                           repo / 'gtdynamics/utils/ContactInitializationFactors.h',
                           app / 'backend/optimizer.cpp', app / 'backend/CMakeLists.txt']
    source_hashes = {str(path.relative_to(repo)): hashlib.sha256(path.read_bytes()).hexdigest()
                     for path in native_source_files if path.exists()}
    executable = args.optimizer.resolve()
    executable_hash = hashlib.sha256(executable.read_bytes()).hexdigest()
    for sequence in registry['sequences']:
        if args.sequence and sequence['id'] not in args.sequence:
            continue
        data_path = args.sequences.parent / sequence['data']
        raw = data_path.read_bytes()
        data = json.loads(raw)
        # The executable receives only geometry and the explicit contact schedule.
        native_input = {key: data[key] for key in ('model', 'contacts', 'frames', 'samples')}
        recording = {
            'schemaVersion': 2, 'sequenceId': sequence['id'],
            'solver': 'C++ GTSAM Levenberg–Marquardt',
            'recordedAtUtc': datetime.now(timezone.utc).isoformat(),
            'source': {'repositoryRevision': revision, 'workingTreeModified': bool(status.strip()),
                       'nativeSourceSha256': source_hashes, 'executableSha256': executable_hash,
                       'datasetSha256': hashlib.sha256(raw).hexdigest(),
                       'referenceExcludedFromNativeInput': True},
            'hardware': {'cpu': cpu, 'system': platform.system(), 'machine': platform.machine()},
            'models': {},
        }
        for model in ('ground', 'patch', 'point'):
            request = {'dataset': native_input, 'model': model, 'iterations': args.iterations,
                       'repetitions': args.repetitions, 'warmup': args.warmup, 'perturb': False}
            process = subprocess.run([str(executable)], input=json.dumps(request), text=True,
                                     capture_output=True, check=True, timeout=300)
            response = json.loads(process.stdout)
            if response.get('error'):
                raise RuntimeError(response['error'])
            if not response.get('result', {}).get('history'):
                raise RuntimeError(f'{model} native solve returned no replay snapshots')
            recording['models'][model] = response
            benchmark = response['benchmark']
            print(f"{sequence['id']} / {model}: GTSAM median {benchmark['medianMs']:.4f} ms "
                  f"({args.repetitions} runs)", flush=True)
        output = args.sequences.parent / sequence['recording']
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(recording, separators=(',', ':'), allow_nan=False) + '\n')
        print(f'Wrote {output} ({output.stat().st_size / 1e6:.2f} MB)', flush=True)


if __name__ == '__main__':
    main()
