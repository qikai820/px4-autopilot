#!/usr/bin/env python3
"""Boot isolated controller chains without sensors, MAVLink or actuator drivers."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('build', type=Path)
a = p.parse_args()
build = a.build.resolve()
cases = [(16, 0, False, False), (17, 0, False, False), (0, 0, True, False),
         (17, 1, False, False), (17, 0, False, True), (17, 1, False, True),
         (16, 0, True, False), (16, 1, True, False), (17, 0, True, False), (17, 1, True, False)]
for instance, (airframe, mode, spatial, monitor) in enumerate(cases, 72):
    kind = 'vector' if airframe in (16, 17) else 'mc'
    modules = [f'{kind}_pos_control', f'{kind}_att_control', f'{kind}_rate_control']
    with tempfile.TemporaryDirectory(prefix='px4-control-smoke-') as temp:
        startup = Path(temp) / 'startup.sh'
        lines = ['#!/bin/sh', f'. "{build}/bin/px4-alias.sh"', f'param select "{temp}/params.bson"',
                 f'param set CA_AIRFRAME {airframe}', f'param set VT_CTRL_MODE {mode}',
                 f'param set VEC_REF_EN {int(spatial)}', f'param set VT_TILT_EN {int(monitor)}',
                 'param set MAV_TYPE 2', f'. "{build}/etc/init.d/rc.{kind}_apps"']
        lines += [f'{module} status || exit 1' for module in modules + ['control_allocator', 'flight_mode_manager']]
        lines += ['echo CONTROL_SMOKE_OK', 'shutdown']
        startup.write_text('\n'.join(lines) + '\n')
        env = os.environ.copy()
        env['PATH'] = str(build / 'bin') + ':' + env['PATH']
        proc = subprocess.Popen([str(build / 'bin/px4'), '-d', '-i', str(instance), '-w', temp,
                                 '-s', str(startup), str(build / 'etc')],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
        try:
            output, _ = proc.communicate(timeout=20)
        except subprocess.TimeoutExpired:
            proc.terminate()
            output, _ = proc.communicate(timeout=5)
            raise RuntimeError(output)
        if proc.returncode != 0 or 'CONTROL_SMOKE_OK' not in output:
            raise RuntimeError(output)
        if kind == 'vector':
            expected = 'enabled' if (airframe == 17 and mode == 1) or (airframe == 16 and spatial) else 'disabled'
            if output.count(f'Vector reference: {expected} (startup setting)') != 3:
                raise RuntimeError(output)
        print(f'PASS startup/status: {kind}, CA_AIRFRAME={airframe}, VT_CTRL_MODE={mode}, '
              f'VEC_REF_EN={int(spatial)}, VT_TILT_EN={int(monitor)}; instance {instance}')
