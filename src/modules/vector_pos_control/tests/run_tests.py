#!/usr/bin/env python3
"""Run actual copied/original controller cores on the host. Requires a configured PX4 build for generated uORB headers."""
from pathlib import Path
import argparse, subprocess, tempfile
p=argparse.ArgumentParser(description=__doc__);p.add_argument('build',type=Path);a=p.parse_args()
root=Path(__file__).resolve().parents[4];build=a.build.resolve()
sources=['src/modules/vector_pos_control/tests/test_vector_control.cpp',
'src/modules/vector_pos_control/VectorPosition/VectorPosition.cpp','src/modules/vector_pos_control/VectorPosition/ControlMath.cpp',
'src/modules/mc_pos_control/PositionControl/PositionControl.cpp','src/modules/mc_pos_control/PositionControl/ControlMath.cpp',
'src/modules/vector_att_control/VectorAttitude/VectorAttitude.cpp','src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp',
'src/modules/vector_rate_control/RateControl/rate_control.cpp','src/lib/rate_control/rate_control.cpp']
includes=['platforms/posix/include','platforms/common/include','src','src/include','src/lib','src/lib/matrix','platforms/common','src/modules/vector_att_control/VectorAttitude','src/modules/mc_att_control/AttitudeControl']
with tempfile.TemporaryDirectory(prefix='vector-control-test-') as temp:
 cmd=['g++','-std=c++17','-O2','-D__PX4_POSIX','-D__EXPORT=','-I'+str(build)]+['-I'+str(root/i) for i in includes]+[str(root/i) for i in sources]+['-o',temp+'/test']
 subprocess.run(cmd,check=True,cwd=root);subprocess.run([temp+'/test'],check=True)
