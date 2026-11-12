#!/usr/bin/env python3
"""Run Voliro allocation, AirMode, tail guard and closed-loop host regressions."""
from pathlib import Path
import argparse
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('build', nargs='?', help='PX4 build with generated uORB and parameter headers')
p.add_argument('--jsbsim-source', type=Path, help='JSBSim source tree for native closed-loop verification')
p.add_argument('--jsbsim-library', type=Path, help='Matching JSBSim static library')
p.add_argument('--jsbsim-model', type=Path, help='Voliro aircraft directory; default: JSBSim data model')
p.add_argument('--output', type=Path, help='Keep native closed-loop CSV traces here')
p.add_argument('--sanitize', action='store_true', help='Enable host address/undefined/float-divide checks')
args = p.parse_args()
if bool(args.jsbsim_source) != bool(args.jsbsim_library):
    p.error('--jsbsim-source and --jsbsim-library must be supplied together')
if args.jsbsim_model and not args.jsbsim_source:
    p.error('--jsbsim-model requires the native JSBSim options')
source = Path(__file__).resolve().parent
root = source.parents[3]
build = Path(args.build).resolve() if args.build else root / 'build/cuav_7-nano_volirot'
with tempfile.TemporaryDirectory(prefix='voliro-t-test-') as temp:
    work = Path(temp)
    includes = [build, build / 'src/lib', root / 'platforms/common', root / 'platforms/common/include',
                root / 'platforms/posix/include', root / 'src', root / 'src/modules', root / 'src/include',
                root / 'src/lib', root / 'src/lib/matrix', root / 'src/lib/control_allocation',
                root / 'src/modules/vector_att_control/VectorAttitude',
                root / 'platforms/posix/src/px4/common/include',
                root / 'platforms/posix/src/px4/generic/generic/include', root / 'boards/px4/sitl/src', source]
    common = ['g++', '-std=gnu++17', '-O2', '-D__PX4_POSIX', '-D__PX4_LINUX', '-D__EXPORT=']
    common += ['-I' + str(i) for i in includes]
    if args.sanitize:
        common += ['-fsanitize=address,undefined,float-divide-by-zero', '-fno-omit-frame-pointer']
    control_sources = [root / 'src/modules/vector_rate_control/RateControl/rate_control.cpp',
                       root / 'src/modules/vector_att_control/VectorAttitude/VectorAttitude.cpp']
    wrapper = [source / 'test_voliro_t_effectiveness.cpp', source / 'ActuatorEffectivenessVoliroT.cpp'] + control_sources
    for name, sources in [('allocation', [source / 'test_voliro_t.cpp']), ('effectiveness', wrapper)]:
        binary = str(work / name)
        subprocess.run(common + list(map(str, sources)) + ['-o', binary], check=True)
        subprocess.run([binary], check=True)
    subprocess.run([str(work / 'effectiveness'), '--replay', str(source / 'testdata/manual_takeoff_baa9a7b4.csv')], check=True)
    if args.jsbsim_source:
        jsbsim = args.jsbsim_source.resolve()
        aircraft = args.jsbsim_model.resolve() if args.jsbsim_model else jsbsim / 'data/aviation/aircraft/voliro_t'
        model = work / 'model'
        shutil.copytree(aircraft, model)
        shutil.copytree(aircraft.parent.parent / 'engine', model / 'engine', dirs_exist_ok=True)
        tree = ET.parse(model / 'voliro_t.xml')
        for node in list(tree.getroot()):
            if node.tag in ('input', 'output'):
                tree.getroot().remove(node)
        tree.write(model / 'voliro_t.xml')
        output = args.output.resolve() if args.output else work / 'traces'
        output.mkdir(parents=True, exist_ok=True)
        binary = str(work / 'native')
        subprocess.run(common + ['-DVOLIRO_AIRMODE_JSBSIM', '-I' + str(jsbsim / 'src')]
                       + list(map(str, wrapper)) + [str(args.jsbsim_library.resolve()), '-lexpat', '-lpthread', '-ldl', '-o', binary], check=True)
        subprocess.run([binary, '--native', str(model), str(jsbsim / 'data/aviation/systems'), str(output)], check=True)
