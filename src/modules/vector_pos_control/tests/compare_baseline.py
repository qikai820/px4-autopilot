#!/usr/bin/env python3
"""Compile one deterministic replay against two source trees and compare float bytes."""
import argparse
from pathlib import Path
import subprocess
import tempfile

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('baseline', type=Path, help='Pre-change source snapshot (src/lib/vector_control and vector controller modules)')
p.add_argument('build', type=Path, help='PX4 generated headers')
a = p.parse_args()
root = Path(__file__).resolve().parents[4]
test = Path(__file__).with_name('test_vector_compatibility.cpp')
with tempfile.TemporaryDirectory(prefix='vector-parity-') as temp:
    traces = []
    for label, tree in [('baseline', a.baseline.resolve()), ('current', root)]:
        includes = [a.build.resolve(), tree / 'src', tree / 'src/lib',
                    tree / 'src/modules/vector_att_control/VectorAttitude',
                    tree / 'src/modules/vector_rate_control',
                    tree / 'src/modules/control_allocator/VehicleActuatorEffectiveness',
                    root / 'platforms/posix/include', root / 'platforms/common/include',
                    root / 'src', root / 'src/include', root / 'src/lib', root / 'src/lib/matrix', root / 'platforms/common']
        sources = [tree / 'src/modules/vector_att_control/VectorAttitude/VectorAttitude.cpp',
                   tree / 'src/modules/vector_rate_control/RateControl/rate_control.cpp', test]
        binary = Path(temp) / label
        subprocess.run(['g++', '-std=c++17', '-O2', '-D__PX4_POSIX', '-D__EXPORT='] +
                       ['-I' + str(path) for path in includes] + list(map(str, sources)) + ['-o', str(binary)], check=True)
        trace = binary.with_suffix('.bin')
        subprocess.run([str(binary), str(trace)], check=True)
        traces.append(trace.read_bytes())
    assert traces[0] and traces[0] == traces[1], 'Voliro output trace changed'
    print(f'PASS: bit-identical Voliro reference/PID/allocation replay: {len(traces[0]) // 4} floats')
