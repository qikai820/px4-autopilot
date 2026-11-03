#!/usr/bin/env python3
"""Run Voliro T allocation, tail guard and final-output feedback regressions on the host."""
from pathlib import Path
import argparse
import subprocess
import tempfile
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('build',nargs='?',help='Configured PX4 build with generated uORB and parameter headers')
args=p.parse_args()
source=Path(__file__).resolve().parent
root=source.parents[3]
build=Path(args.build).resolve() if args.build else root/'build/cuav_7-nano_volirot'
with tempfile.TemporaryDirectory(prefix='voliro-t-test-') as temp:
    includes=[build,build/'src/lib',root/'platforms/common',root/'platforms/common/include',
              root/'platforms/posix/include',root/'src',root/'src/modules',root/'src/include',
              root/'src/lib',root/'src/lib/matrix',root/'src/lib/control_allocation',
              root/'platforms/posix/src/px4/common/include',root/'platforms/posix/src/px4/generic/generic/include',root/'boards/px4/sitl/src',source]
    common=['g++','-std=gnu++17','-O2','-D__PX4_POSIX','-D__PX4_LINUX','-D__EXPORT=']+['-I'+str(i) for i in includes]
    for name,sources in [('test',['test_voliro_t.cpp']),
                         ('effectiveness',['test_voliro_t_effectiveness.cpp','ActuatorEffectivenessVoliroT.cpp'])]:
        binary=str(Path(temp)/name)
        subprocess.run(common+[str(source/s) for s in sources]+['-o',binary],check=True)
        subprocess.run([binary],check=True)
