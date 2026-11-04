#!/usr/bin/env python3
"""Check servo scaling and independent pitch using a private, network-free model copy."""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('model', type=Path)
p.add_argument('prefix', type=Path)
p.add_argument('systems', type=Path)
a = p.parse_args()
with tempfile.TemporaryDirectory(prefix='voliro-pitch-range-') as temp:
    model = Path(temp) / 'model'
    shutil.copytree(a.model.resolve(), model)
    tree = ET.parse(model / 'voliro_t.xml')
    xml = tree.getroot()
    for tag in ('input', 'output'):
        for node in list(xml.findall(tag)):
            xml.remove(node)
    tree.write(model / 'voliro_t.xml')
    binary = Path(temp) / 'test'
    prefix = a.prefix.resolve()
    subprocess.run(['g++', '-std=c++17', '-O2', '-I'+str(prefix / 'include/JSBSim'),
                    str(Path(__file__).with_suffix('.cpp')), str(prefix / 'lib/libJSBSim.a'),
                    '-lexpat', '-lpthread', '-o', str(binary)], check=True)
    subprocess.run([str(binary), str(model), str(a.systems.resolve())], check=True)
