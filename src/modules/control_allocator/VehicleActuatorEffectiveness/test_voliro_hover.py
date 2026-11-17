#!/usr/bin/env python3
"""Reproduce log21-like oscillation and check reaction P shaping in native JSBSim. No hardware connection."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('build', nargs='?', help='PX4 build with generated uORB and parameter headers')
p.add_argument('--jsbsim-source', type=Path, help='JSBSim source tree for native closed-loop verification')
p.add_argument('--jsbsim-library', type=Path, help='Matching JSBSim static library')
p.add_argument('--jsbsim-model', type=Path, help='Voliro aircraft directory; default: JSBSim data model')
p.add_argument('--output', type=Path, help='Keep native test results and model hashes here')
p.add_argument('--sanitize', action='store_true', help='Enable host address/undefined/float-divide checks')
p.add_argument('--extended', action='store_true', help='Include inverted hover and both +/-160 degree endpoints')
p.add_argument('--tail-reverse-max', type=float, default=0.6,
               help='Test fixture CA_CT_T5RMAX in N; default is the recorded flight configuration (0.6)')
p.add_argument('--reaction-scale', type=float, default=0.5, help='VT_RCT_P_SCALE for the corrected cases')
args = p.parse_args()
if not 0 < args.tail_reverse_max <= 100:
    p.error('--tail-reverse-max must be finite and in (0, 100] N')
if not 0.1 <= args.reaction_scale < 1:
    p.error('--reaction-scale must be finite and in [0.1, 1)')
if not args.jsbsim_source or not args.jsbsim_library:
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
    common += [f'-DVOLIRO_HOVER_TAIL_RMAX={args.tail_reverse_max}f']
    common += [f'-DVOLIRO_HOVER_REACTION_SCALE={args.reaction_scale}f']
    common += ['-I' + str(i) for i in includes]
    if args.sanitize:
        common += ['-fsanitize=address,undefined,float-divide-by-zero', '-fno-omit-frame-pointer']
    control_sources = [root / 'src/modules/vector_rate_control/RateControl/rate_control.cpp',
                       root / 'src/modules/vector_att_control/VectorAttitude/VectorAttitude.cpp']
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
                       + list(map(str, [source / 'test_voliro_hover.cpp', source / 'ActuatorEffectivenessVoliroT.cpp'] + control_sources)) + [str(args.jsbsim_library.resolve()), '-lexpat', '-lpthread', '-ldl', '-o', binary], check=True)
        run = subprocess.run([binary, str(model), str(jsbsim / 'data/aviation/systems')] + (['--extended'] if args.extended else []),
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        print(run.stdout)
        (output / 'native-hover.log').write_text(run.stdout)
        manifest = {str(path.relative_to(aircraft)): hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in sorted(aircraft.rglob('*.xml'))}
        (output / 'model-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        engine_manifest = {str(path.relative_to(model / 'engine')): hashlib.sha256(path.read_bytes()).hexdigest()
                           for path in sorted((model / 'engine').rglob('*.xml'))}
        (output / 'engine-manifest.json').write_text(json.dumps(engine_manifest, indent=2) + '\n')
        (output / 'test-config.json').write_text(json.dumps({
            'extended': args.extended, 'CA_CT_T5RMAX_N': args.tail_reverse_max,
            'VT_RCT_P_SCALE': args.reaction_scale,
            'CA_CT_T5MAX_N': 10.0, 'CA_CT_X_m': 0.02, 'CA_CT_TAIL_m': 0.74,
            'model': str(aircraft), 'actuator_warmup': 'rigid-body hold, nonzero dt',
            'jsbsim_library': str(args.jsbsim_library.resolve()),
            'jsbsim_library_sha256': hashlib.sha256(args.jsbsim_library.read_bytes()).hexdigest(),
            'sanitized_controller_and_fixture': args.sanitize,
            'exit_code': run.returncode,
        }, indent=2) + '\n')
        run.check_returncode()
