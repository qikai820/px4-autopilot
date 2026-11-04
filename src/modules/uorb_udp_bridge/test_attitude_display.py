#!/usr/bin/env python3
"""Test the exact angle conversion used by the UDP bridge on the host."""
import argparse
from pathlib import Path
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('build', type=Path)
args = parser.parse_args()
source = Path(__file__).resolve().parent
root = source.parents[2]
includes = [args.build.resolve(), root / 'platforms/posix/include', root / 'platforms/common/include',
            root / 'src', root / 'src/include', root / 'src/lib', root / 'src/lib/matrix']
with tempfile.TemporaryDirectory(prefix='uorb-attitude-display-') as temp:
    executable = str(Path(temp) / 'test')
    subprocess.run(['g++', '-std=c++17', '-O2', '-D__PX4_POSIX', '-D__EXPORT=',
                    *['-I'+str(path) for path in includes], str(source / 'test_attitude_display.cpp'),
                    '-o', executable], check=True)
    subprocess.run([executable], check=True)
