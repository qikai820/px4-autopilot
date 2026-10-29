#!/usr/bin/env python3
"""Start an isolated PX4 POSIX + JSBSim UDP simulation; never arms a vehicle."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

PX4_ROOT = Path(__file__).resolve().parents[3]
MODELS = {'voliro_t':19003, 'morphquad':19001, 'morphquad2':19002, 'quadrotor_x':3010}

def parse_args():
    args=sys.argv[1:]
    if not args or args[0] not in ('start','stop','status','console','logs'):
        args=['start']+args
    command=[]
    if '--' in args:
        index=args.index('--');command=args[index+1:];args=args[:index]
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['start','stop','status','console','logs'])
    p.add_argument('model',nargs='?',choices=MODELS,default='voliro_t')
    p.add_argument('-s','--scene',default='scene/LSZH.xml')
    p.add_argument('--jsbsim-root',type=Path,default=Path(os.environ.get('JSBSIM_ROOT',PX4_ROOT.parent/'px4_jsbsim')))
    p.add_argument('--build-dir',type=Path,default=PX4_ROOT/'build/px4_sitl_vector')
    p.add_argument('--instance',type=int,default=1)
    p.add_argument('--px4-port',type=int,default=15460)
    p.add_argument('--bridge-port',type=int,default=15461)
    p.add_argument('--server-port',type=int,default=15800)
    p.add_argument('--qgc-port',type=int,default=14550)
    p.add_argument('--runtime-dir',type=Path)
    p.add_argument('--build',action='store_true',help='Build PX4 SITL and the modified JSBSim bridge/server before starting')
    p.add_argument('--jobs',type=int,default=4)
    p.add_argument('--log-profile',type=int,default=1,help='SDLOG_PROFILE in this local instance (default: standard flight topics)')
    p.add_argument('--flightgear',action='store_true',help='Enable the original FlightGear UDP visualization output')
    a=p.parse_args(args)
    a.command=command
    if not 1<=a.instance<=99:p.error('--instance must be in 1..99 (separate from physical FC system ID 1)')
    for name in ['px4_port','bridge_port','server_port','qgc_port']:
        if not 1<=getattr(a,name)<=65535:p.error(name+' must be in 1..65535')
    if len({a.px4_port,a.bridge_port,a.server_port})!=3:p.error('Simulation ports must be distinct')
    a.build_dir=a.build_dir.resolve();a.jsbsim_root=a.jsbsim_root.resolve()
    a.runtime_dir=(a.runtime_dir or a.build_dir/f'run-{a.model}-{a.instance}').resolve()
    return a

def token(pid):
    try:
        fields=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        return fields[19] if fields[0]!='Z' else None
    except (OSError,IndexError):return None

def alive(record):
    return bool(record and token(record['pid'])==record['token'])

def record(proc):
    return {'pid':proc.pid,'token':token(proc.pid)}

def save(path,state):
    temp=path.with_suffix('.new');temp.write_text(json.dumps(state,indent=2)+'\n');temp.replace(path)

def client(state,module,*args,timeout=5):
    command=[str(Path(state['build_dir'])/'bin'/('px4-'+module)),'--instance',str(state['instance']),*args]
    return subprocess.run(command,capture_output=True,text=True,timeout=timeout)

def stop(state):
    if alive(state.get('px4')):
        try:client(state,'shutdown',timeout=6)
        except (subprocess.TimeoutExpired,OSError):pass
    # Only terminate groups whose original leader still matches our saved start token.
    for name in ['bridge','px4']:
        item=state.get(name)
        if alive(item):
            try:os.killpg(item['pid'],signal.SIGTERM)
            except ProcessLookupError:pass
    server=state.get('server')
    if alive(server):
        try:os.kill(server['pid'],signal.SIGTERM)
        except ProcessLookupError:pass
    end=time.monotonic()+5
    while time.monotonic()<end and any(alive(state.get(k)) for k in ['px4','bridge','server']):time.sleep(.1)
    for name in ['bridge','px4','server']:
        item=state.get(name)
        if alive(item):
            try:os.kill(item['pid'],signal.SIGKILL)
            except ProcessLookupError:pass

def check_port(port,tcp=False):
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM if tcp else socket.SOCK_DGRAM) as s:
        try:s.bind(('127.0.0.1',port))
        except OSError as e:raise RuntimeError(f'{"TCP" if tcp else "UDP"} port {port} is occupied; choose another port: {e}')

def launch(a,path):
    build=a.build_dir;project=a.jsbsim_root;data=project/'px4-jsbsim-bridge'
    bridge=project/'build/px4-jsbsim-bridge/jsbsim_bridge'
    server=bridge.with_name('jsbsim_server');run=project/'scripts/run.sh'
    env=os.environ.copy();env.setdefault('CCACHE_DIR','/tmp/px4-vector-ccache')
    if a.build:
        subprocess.run(['make','px4_sitl_vector',f'-j{a.jobs}'],cwd=PX4_ROOT,env=env,check=True)
        subprocess.run(['cmake','--build',str(project/'build'),'--target','jsbsim_bridge','jsbsim_server','--parallel',str(a.jobs)],env=env,check=True)
    for binary in [build/'bin/px4',bridge,server,run]:
        if not binary.is_file():raise RuntimeError(f'Missing {binary}; build with --build first')
    for binary in [bridge,server]:
        contents=binary.read_bytes()
        for key in [b'JSBSIM_CONFIG_FILE',b'JSBSIM_SERVER_PORT']+([b'JSBSIM_WRENCH_FEEDBACK'] if binary==bridge else []):
            if key not in contents:raise RuntimeError(f'{binary} lacks {key.decode()}; use --build')
    if not (build/'etc/init.d-posix/airframes'/f'{MODELS[a.model]}_jsbsim_{a.model}').is_file():
        raise RuntimeError('PX4 ROMFS lacks the selected SITL airframe; use --build')
    for port in [a.px4_port,a.bridge_port,18570+a.instance,14580+a.instance,14280+a.instance,13030+a.instance]:check_port(port)
    check_port(a.server_port,tcp=True)
    scene=Path(a.scene);scene=scene if scene.is_absolute() else data/scene
    if not scene.is_file():raise RuntimeError(f'Scene not found: {scene}')
    tree=ET.parse(data/'configs'/f'{a.model}.xml');interface=tree.getroot().find('mavlink_interface')
    for name,value in {'use_tcp':0,'enable_lockstep':0,'udp_port':a.bridge_port,'fc_addr':'127.0.0.1','fc_port':a.px4_port}.items():
        element=interface.find(name)
        if element is None:element=ET.SubElement(interface,name)
        element.text=str(value)
    # Do not send telemetry from the isolated instance to hardware-side consumers.
    for tag in ['plotjuggler','wrench_feedback']:
        section=tree.getroot().find(tag)
        if section is not None:
            enabled=section.find('enabled')
            if enabled is not None:enabled.text='false'
    config=a.runtime_dir/'jsbsim.xml';tree.write(config,encoding='utf-8',xml_declaration=True)
    work=a.runtime_dir/'rootfs';work.mkdir(exist_ok=True)
    env.update({'PX4_SYS_AUTOSTART':str(MODELS[a.model]),'PX4_SIM_MODEL':'jsbsim_'+a.model,
                'PX4_SIM_UDP_PORT':str(a.px4_port),'PX4_GCS_UDP_PORT':str(a.qgc_port),
                'PX4_PARAM_SYS_HITL':'0','PX4_PARAM_SDLOG_MODE':'2','PX4_PARAM_SDLOG_BACKEND':'1',
                'PX4_PARAM_SDLOG_PROFILE':str(a.log_profile),'PX4_PARAM_SYS_AUTOSTART':str(MODELS[a.model]),
                'JSBSIM_CONFIG_FILE':str(config),'JSBSIM_SERVER_PORT':str(a.server_port),
                'HEADLESS':'0' if a.flightgear else '1','PX4_SIM_SPEED_FACTOR':'1','JSBSIM_WRENCH_FEEDBACK':'0'})
    state={'model':a.model,'scene':str(scene),'instance':a.instance,'system_id':a.instance+1,
           'build_dir':str(build),'runtime_dir':str(a.runtime_dir),'px4_port':a.px4_port,
           'bridge_port':a.bridge_port,'server_port':a.server_port,'qgc_port':a.qgc_port,
           'config':str(config),'created':time.time()}
    px4log=a.runtime_dir/'px4.log';jsblog=a.runtime_dir/'jsbsim.log'
    try:
        with px4log.open('w') as log:
            px4=subprocess.Popen([str(build/'bin/px4'),'-d','-i',str(a.instance),'-w',str(work),
                                  '-s',str(build/'etc/init.d-posix/rcS'),str(build/'etc')],
                                 stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
        state['px4']=record(px4);save(path,state)
        with jsblog.open('w') as log:
            sim=subprocess.Popen([str(run),a.model,'-s',str(scene)],cwd=project,env=env,
                                 stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        state['bridge']=record(sim);save(path,state)
        deadline=time.monotonic()+45;last=''
        while time.monotonic()<deadline:
            if px4.poll() is not None or sim.poll() is not None:
                raise RuntimeError(f'Process exited; inspect {px4log} and {jsblog}')
            children=Path(f'/proc/{sim.pid}/task/{sim.pid}/children')
            if children.exists():
                for child in children.read_text().split():
                    state['server']={'pid':int(child),'token':token(int(child))}
                save(path,state)
            try:
                rate=client(state,'listener','vehicle_angular_velocity','-n','1',timeout=2).stdout
                attitude=client(state,'listener','vehicle_attitude','-n','1',timeout=2).stdout
                last=rate+'\n'+attitude
                if 'xyz:' in rate and 'q:' in attitude and not any(v in last.lower() for v in ['nan',' inf']):
                    log_status=client(state,'logger','status',timeout=2).stdout
                    if 'Full File Logging Running' in log_status:
                        state['ready']=True;state['ready_time']=time.time();save(path,state)
                        print_status(state);print(log_status.strip());return
            except (subprocess.TimeoutExpired,OSError):pass
            time.sleep(.5)
        raise RuntimeError('No valid attitude/rates or file logging after 45s. Last state:\n'+last)
    except BaseException:
        stop(state);state['ready']=False;save(path,state);raise

def print_status(state):
    print(f'Model: {state["model"]}, PX4 instance {state["instance"]}, MAVLink system ID {state["system_id"]}')
    print(f'PX4: {"running" if alive(state.get("px4")) else "stopped"}; JSBSim: {"running" if alive(state.get("bridge")) else "stopped"}')
    print(f'UDP: PX4 127.0.0.1:{state["px4_port"]} <-> JSBSim 127.0.0.1:{state["bridge_port"]}')
    print(f'JSBSim internal TCP: 127.0.0.1:{state["server_port"]}; QGC destination: 127.0.0.1:{state["qgc_port"]}')
    print('Runtime/logs: '+state['runtime_dir'])

def main():
    a=parse_args();a.runtime_dir.mkdir(parents=True,exist_ok=True);path=a.runtime_dir/'state.json'
    with (a.runtime_dir/'launcher.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        state=json.loads(path.read_text()) if path.exists() else None
        if a.action=='start':
            if state and any(alive(state.get(k)) for k in ['px4','bridge','server']):
                print_status(state);raise RuntimeError('Instance already exists; stop it first')
            launch(a,path)
        elif a.action=='status':
            if not state:raise RuntimeError('No saved instance')
            print_status(state)
            if alive(state.get('px4')):
                for module in ['commander','simulator_mavlink','logger']:
                    print(client(state,module,'status').stdout.strip())
        elif a.action=='stop':
            if state:stop(state);print_status(state)
        elif a.action=='console':
            if not state or not alive(state.get('px4')):raise RuntimeError('PX4 is not running')
            command=a.command
            if command and command[0]=='--':command=command[1:]
            if not command:raise RuntimeError('Append a command: console voliro_t -- commander status')
            result=client(state,command[0],*command[1:],timeout=15)
            print(result.stdout,end='');print(result.stderr,end='',file=sys.stderr);return result.returncode
        elif a.action=='logs':
            print(a.runtime_dir/'px4.log');print(a.runtime_dir/'jsbsim.log');print(a.runtime_dir/'rootfs/log')
    return 0

if __name__=='__main__':
    try:sys.exit(main())
    except (RuntimeError,subprocess.CalledProcessError) as e:print('ERROR:',e,file=sys.stderr);sys.exit(1)
