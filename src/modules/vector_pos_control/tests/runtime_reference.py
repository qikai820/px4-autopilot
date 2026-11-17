#!/usr/bin/env python3
"""Exercise actual position/attitude/rate modules and their atomic references in SITL."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('build', type=Path)
p.add_argument('--voliro', action='store_true', help='Exercise Voliro motion-monitor and mode transitions')
p.add_argument('--reaction', action='store_true', help='Exercise reaction P projection gating and stale-axis retention')
p.add_argument('--spatial-large', action='store_true', help='Exercise MorphQuad +/-120 degrees, frame reset and RC-loss leveling')
a = p.parse_args()
if a.spatial_large and (a.voliro or a.reaction):
    p.error('--spatial-large requires the MorphQuad reference test')
build = a.build.resolve()
root = Path(__file__).resolve().parents[4]
with tempfile.TemporaryDirectory(prefix='vector-reference-runtime-') as temp:
    work = Path(temp)
    library = work / 'reference.px4mod'
    includes = [build, build / 'src/lib', root / 'platforms/common', root / 'platforms/common/include',
                root / 'platforms/posix/include', root / 'src', root / 'src/include', root / 'src/lib', root / 'src/lib/matrix', root / 'platforms/posix/src/px4/common/include',
                root / 'platforms/posix/src/px4/generic/generic/include', root / 'boards/px4/sitl/src']
    subprocess.run(['g++', '-std=gnu++17', '-O2', '-fPIC', '-shared', '-D__PX4_POSIX', '-D__PX4_LINUX', '-DENABLE_LOCKSTEP_SCHEDULER', '-D__EXPORT='] +
                   (['-DVECTOR_TEST_SPATIAL_LARGE'] if a.spatial_large else []) + [
                    '-I' + str(build)] + ['-I' + str(path) for path in includes] +
                   [str(Path(__file__).with_name('runtime_reaction.cpp') if a.reaction else Path(__file__).with_name('runtime_voliro_modes.cpp') if a.voliro else Path(__file__).with_suffix('.cpp')), '-o', str(library)], check=True)
    startup = work / 'startup.sh'
    lines = ['#!/bin/sh', f'. "{build}/bin/px4-alias.sh"', f'param select "{work}/params.bson"',
             f'param set CA_AIRFRAME {17 if a.voliro else 16}', 'param set VEC_REF_EN 1', 'param set VEC_AUX_ATT 1',
             'param set COM_SPOOLUP_TIME 0', 'param set MPC_TKO_RAMP_T 0', 'param set RC_MAP_AUX1 1', 'param set RC_MAP_AUX2 2', 'param set MAV_TYPE 2',
             'vector_rate_control start', 'vector_att_control start', 'vector_pos_control start',
             f'dyn "{library}"', 'vector_pos_control status', 'vector_att_control status', 'vector_rate_control status', 'shutdown']
    if a.voliro:
        index = lines.index('vector_rate_control start')
        lines[index:index] = ['param set VT_CTRL_MODE 1', 'param set VT_TILT_EN 1', 'param set VT_TILT_VMAX 5',
                              'param set VT_PITCH_MAX 30', 'param set RC_MAP_VECTOR_SW 8']
    else:
        lines.insert(lines.index('vector_rate_control start'), 'control_allocator start')
        if a.spatial_large:
            index = lines.index('vector_rate_control start')
            lines[index:index] = ['param set VEC_ATT_MAX 120', 'param set VEC_ATT_RATE 90', 'param set VEC_ATT_ACC 360']
    startup.write_text('\n'.join(lines) + '\n')
    env = os.environ.copy()
    env['PATH'] = str(build / 'bin') + ':' + env['PATH']
    output = subprocess.run([str(build / 'bin/px4'), '-d', '-i', '92' if a.voliro else '91', '-w', temp, '-s', str(startup), str(build / 'etc')],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env, timeout=25)
    print(output.stdout)
    assert output.returncode == 0 and ('REACTION_RUNTIME_OK' if a.reaction else 'VOLIRO_MODES_RUNTIME_OK' if a.voliro else 'VECTOR_REFERENCE_RUNTIME_OK') in output.stdout
