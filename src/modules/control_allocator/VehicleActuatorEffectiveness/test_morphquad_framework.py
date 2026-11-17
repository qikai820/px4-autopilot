#!/usr/bin/env python3
"""Host regression of the actual MorphQuad allocator and common vector controller."""
from pathlib import Path
import argparse
import subprocess
import tempfile
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('build', type=Path)
a = p.parse_args()
source = Path(__file__).resolve().parent
root = source.parents[3]
includes = [a.build.resolve(), a.build.resolve() / 'src/lib', root / 'platforms/common',
            root / 'platforms/common/include', root / 'platforms/posix/include', root / 'src',
            root / 'src/modules', root / 'src/include', root / 'src/lib', root / 'src/lib/matrix',
            root / 'src/lib/control_allocation', root / 'src/modules/vector_att_control/VectorAttitude',
            root / 'src/modules/vector_rate_control']
sources = [source / 'test_morphquad_framework.cpp', source / 'ActuatorEffectivenessMorphQuad.cpp',
           root / 'src/modules/vector_att_control/VectorAttitude/VectorAttitude.cpp',
           root / 'src/modules/vector_rate_control/RateControl/rate_control.cpp']
with tempfile.TemporaryDirectory(prefix='morphquad-framework-') as temp:
    binary = str(Path(temp) / 'test')
    subprocess.run(['g++', '-std=gnu++17', '-O2', '-D__PX4_POSIX', '-D__PX4_LINUX', '-D__EXPORT=', '-DMODULE_NAME="morphquad_test"'] +
                   ['-I' + str(path) for path in includes] + list(map(str, sources)) + ['-o', binary], check=True)
    subprocess.run([binary], check=True)
