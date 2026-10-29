#!/usr/bin/env python3
"""Run Voliro T allocation angle/force/torque/bounds regressions on the host."""
from pathlib import Path
import argparse
import subprocess
import tempfile
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('build',nargs='?',help='Accepted for compatibility; host test does not need a firmware build')
args=p.parse_args()
source=Path(__file__).resolve().parent
root=source.parents[3]
build=Path(args.build).resolve() if args.build else root/'build/cuav_7-nano_default'
with tempfile.TemporaryDirectory(prefix='voliro-t-test-') as temp:
    binary=str(Path(temp)/'test')
    subprocess.run(['g++','-I'+str(build),'-std=c++17','-O2','-D__PX4_POSIX','-I'+str(root/'platforms/common/include'),'-I'+str(root/'platforms/posix/include'),'-I'+str(root/'src'),'-I'+str(root/'src/lib'),
                    '-I'+str(root/'src/lib/matrix'),str(source/'test_voliro_t.cpp'),'-o',binary],check=True)
    subprocess.run([binary],check=True)
