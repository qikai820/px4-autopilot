#!/usr/bin/env python3
"""Validate the corrected external model using an installed native JSBSim library."""
import argparse
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("model", type=Path)
parser.add_argument("prefix", type=Path, help="JSBSim installation prefix")
parser.add_argument("systems", type=Path)
args = parser.parse_args()
model = args.model.resolve()
xml = ET.parse(model / "voliro_t.xml").getroot()
assert float(xml.find("mass_balance/emptywt").text) == 5.04
assert float(xml.find("mass_balance/location/x").text) == 0
for force in xml.findall("external_reactions/force"):
    expected = .64 if force.attrib["name"] == "tail_z" else -.14
    assert abs(float(force.find("location/x").text) - expected) < 1e-9
contacts = xml.findall("ground_reactions/contact/location/x")
assert [float(x.text) for x in contacts] == [-.24, -.24, .18]
for location in xml.findall("metrics/location/x"):
    assert float(location.text) == -.11
ET.parse(model / "VoliroTActuators.xml")
print("PASS: CG, rigid geometry, landing contacts, reference points and XML structure", flush=True)
source = Path(__file__).with_suffix(".cpp")
with tempfile.TemporaryDirectory(prefix="voliro-t-model-test-") as temp:
    binary = Path(temp) / "test_model"
    subprocess.run([
        "g++", "-std=c++17", "-O2", "-I" + str(args.prefix / "include/JSBSim"),
        str(source), str(args.prefix / "lib/libJSBSim.a"), "-lexpat", "-lpthread", "-o", str(binary),
    ], check=True)
    subprocess.run([str(binary), str(model), str(args.systems.resolve())], check=True)
