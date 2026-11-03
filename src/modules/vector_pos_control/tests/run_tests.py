#!/usr/bin/env python3
"""Test actual controller cores using generated uORB headers from a PX4 build.

The optional native JSBSim plant uses a temporary copy of the supplied model,
with its network input/output blocks removed. No PX4 instance or vehicle is used.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET


def native_fixture(model_source, destination):
    shutil.copytree(model_source, destination)
    for path in sorted(destination.glob("*.xml")):
        print(f"JSBSim source {path.name}: sha256={hashlib.sha256(path.read_bytes()).hexdigest()}", flush=True)
    tree = ET.parse(destination / "voliro_t.xml")
    xml = tree.getroot()
    mass = xml.find("mass_balance/emptywt")
    assert mass.get("unit") == "KG" and abs(float(mass.text) - 5.04) < 1e-6
    inertia = []
    for axis in ("ixx", "iyy", "izz"):
        node = xml.find("mass_balance/" + axis)
        assert node.get("unit") == "KG*M2" and float(node.text) > 0
        inertia.append(float(node.text))
    for axis in ("ixy", "ixz", "iyz"):
        node = xml.find("mass_balance/" + axis)
        assert node is None or abs(float(node.text)) < 1e-9, "test controller uses diagonal inertia"

    def location(path):
        node = xml.find(path)
        assert node.get("unit") == "M"
        return [float(node.find(axis).text) for axis in ("x", "y", "z")]

    cg = location("mass_balance/location[@name='CG']")
    front = [location(f"external_reactions/force[@name='front_{i}_z']/location") for i in range(4)]
    tail = location("external_reactions/force[@name='tail_z']/location")
    x = cg[0] - front[0][0]
    y = abs(front[0][1] - cg[1])
    z = sum(cg[2] - point[2] for point in front) / 4
    tail_arm = tail[0] - cg[0]
    assert x > 0 and y > 0 and tail_arm > 0
    for tag in ("input", "output"):
        for node in list(xml.findall(tag)):
            xml.remove(node)
    tree.write(destination / "voliro_t.xml", encoding="utf-8", xml_declaration=True)
    print(f"JSBSim fixture: inertia={inertia}, geometry={[x, y, z, tail_arm]}", flush=True)
    return inertia + [x, y, z, tail_arm]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build", type=Path)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--sweep", action="store_true")
    parser.add_argument("--jsbsim-model", type=Path)
    parser.add_argument("--jsbsim-prefix", type=Path)
    parser.add_argument("--jsbsim-systems", type=Path)
    args = parser.parse_args()
    if args.jsbsim_model and (not args.jsbsim_prefix or not args.jsbsim_systems):
        parser.error("--jsbsim-model requires --jsbsim-prefix and --jsbsim-systems")
    root = Path(__file__).resolve().parents[4]
    sources = [
        "src/modules/vector_pos_control/VectorPosition/VectorPosition.cpp",
        "src/modules/vector_pos_control/VectorPosition/ControlMath.cpp",
        "src/modules/mc_pos_control/PositionControl/PositionControl.cpp",
        "src/modules/mc_pos_control/PositionControl/ControlMath.cpp",
        "src/modules/vector_att_control/VectorAttitude/VectorAttitude.cpp",
        "src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp",
        "src/modules/vector_rate_control/RateControl/rate_control.cpp",
        "src/lib/rate_control/rate_control.cpp",
    ]
    includes = [
        "platforms/posix/include", "platforms/common/include", "src", "src/include",
        "src/lib", "src/lib/matrix", "platforms/common",
        "src/modules/vector_att_control/VectorAttitude", "src/modules/mc_att_control/AttitudeControl",
    ]
    common = ["g++", "-std=c++17", "-O2", "-D__PX4_POSIX", "-D__EXPORT=", "-I" + str(args.build.resolve())]
    common += ["-I" + str(root / path) for path in includes]
    common += [str(root / path) for path in sources]
    tests = Path(__file__).resolve().parent
    with tempfile.TemporaryDirectory(prefix="vector-control-test-") as temp:
        executable = str(Path(temp) / "test")
        for name in ("test_vector_control", "test_vector_reference", "test_voliro_dynamics"):
            subprocess.run(common + [str(tests / (name + ".cpp")), "-o", executable], check=True, cwd=root)
            arguments = [str(args.csv.resolve())] if name == "test_voliro_dynamics" and args.csv else []
            subprocess.run([executable] + arguments, check=True)
        if args.sweep:
            rows = []
            for gains in itertools.product([6, 8, 12], [0, .3, 1], [.3, .6], [1, 2]):
                output = subprocess.check_output([executable, "--tune", *map(str, gains)], text=True)
                matches = re.findall(r"finite=(\d) peak=([\d.]+)deg final=([\d.inf]+)deg", output)
                score = max(float(value[2]) for value in matches) if len(matches) == 3 and all(value[0] == "1" for value in matches) else float("inf")
                rows.append({"gains": gains, "final_worst_deg": score, "runs": output.strip().splitlines()})
            rows.sort(key=lambda row: row["final_worst_deg"])
            print(json.dumps(rows[:10], indent=2))
        if args.jsbsim_model:
            model = Path(temp) / "model"
            parameters = native_fixture(args.jsbsim_model.resolve(), model)
            prefix = args.jsbsim_prefix.resolve()
            native_cmd = common + [str(tests / "test_voliro_dynamics.cpp"), "-DVECTOR_TEST_JSBSIM",
                                   "-I" + str(prefix / "include/JSBSim"), str(prefix / "lib/libJSBSim.a"),
                                   "-lexpat", "-lpthread", "-o", executable]
            subprocess.run(native_cmd, check=True, cwd=root)
            subprocess.run([executable, "--jsbsim", str(model), str(args.jsbsim_systems.resolve()), *map(str, parameters)], check=True)


if __name__ == "__main__":
    main()
