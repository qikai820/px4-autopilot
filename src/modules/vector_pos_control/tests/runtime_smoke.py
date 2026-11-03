#!/usr/bin/env python3
"""Boot each controller chain in isolated POSIX instances, without sensors, MAVLink or actuator drivers."""
import argparse, os, subprocess, tempfile
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__);p.add_argument('build',type=Path);a=p.parse_args();build=a.build.resolve()
for instance,kind,airframe,mode,modules in [(72,'vector',16,0,['vector_pos_control','vector_att_control','vector_rate_control']),
                                         (73,'vector',17,0,['vector_pos_control','vector_att_control','vector_rate_control']),
                                         (74,'mc',0,0,['mc_pos_control','mc_att_control','mc_rate_control']),
                                         (75,'vector',17,1,['vector_pos_control','vector_att_control','vector_rate_control'])]:
    with tempfile.TemporaryDirectory(prefix='px4-control-smoke-') as temp:
        startup=Path(temp)/'startup.sh'
        lines=['#!/bin/sh',f'. "{build}/bin/px4-alias.sh"',f'param select "{temp}/params.bson"',
               f'param set CA_AIRFRAME {airframe}',f'param set VT_CTRL_MODE {mode}',
               'param set MAV_TYPE 2',f'. "{build}/etc/init.d/rc.{kind}_apps"']
        for module in modules+['control_allocator','flight_mode_manager']:
            lines.append(f'{module} status || exit 1')
        lines+=['echo CONTROL_SMOKE_OK','shutdown']
        startup.write_text('\n'.join(lines)+'\n')
        env=os.environ.copy();env['PATH']=str(build/'bin')+':'+env['PATH']
        proc=subprocess.Popen([str(build/'bin/px4'),'-d','-i',str(instance),'-w',temp,'-s',str(startup),str(build/'etc')],
                              stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,env=env)
        try:
            output,_=proc.communicate(timeout=20)
        except subprocess.TimeoutExpired:
            proc.terminate();output,_=proc.communicate(timeout=5)
            raise RuntimeError(output)
        if proc.returncode!=0 or 'CONTROL_SMOKE_OK' not in output:
            raise RuntimeError(output)
        if kind == 'vector':
            expected = 'enabled' if mode == 1 and airframe == 17 else 'disabled'
            if output.count(f'Voliro dynamics: {expected} (startup setting)') != 3:
                raise RuntimeError(output)
        print(f'PASS startup/status: {kind}, CA_AIRFRAME={airframe}, VT_CTRL_MODE={mode}; instance {instance}')
