#!/usr/bin/env python3
"""Check that the MJCF joint ranges and motor ctrlranges match the URDF.

For every revolute joint in urdf/rpo.urdf, each MJCF file must use the
URDF lower/upper limits as the joint range, and the URDF effort as the
symmetric ctrlrange of the motor driving that joint.

Usage: python3 scripts/check_limits.py [--tol 1e-4]
Exits 1 if any limit drifts, 0 otherwise. Standard library only.
"""

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
URDF = ROOT / "urdf" / "rpo.urdf"
MJCF_FILES = sorted((ROOT / "mjcf").glob("*.xml"))


def urdf_limits(path):
    limits = {}
    for joint in ET.parse(path).getroot().iter("joint"):
        limit = joint.find("limit")
        if joint.get("type") != "revolute" or limit is None:
            continue
        limits[joint.get("name")] = (
            float(limit.get("lower")),
            float(limit.get("upper")),
            float(limit.get("effort")),
        )
    return limits


def floats(text):
    return tuple(float(v) for v in text.split())


def check_mjcf(path, limits, tol):
    root = ET.parse(path).getroot()
    ranges = {j.get("name"): j.get("range") for j in root.iter("joint")}
    ctrl = {m.get("joint"): m.get("ctrlrange") for m in root.iter("motor")}
    errors = []
    for name, (lower, upper, effort) in limits.items():
        if ranges.get(name) is None:
            errors.append(f"{name}: no joint range")
        else:
            lo, hi = floats(ranges[name])
            if abs(lo - lower) > tol or abs(hi - upper) > tol:
                errors.append(f"{name}: range [{lo}, {hi}], urdf [{lower}, {upper}]")
        if ctrl.get(name) is None:
            errors.append(f"{name}: no motor ctrlrange")
        else:
            lo, hi = floats(ctrl[name])
            if abs(lo + effort) > tol or abs(hi - effort) > tol:
                errors.append(f"{name}: ctrlrange [{lo}, {hi}], urdf effort {effort}")
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--tol", type=float, default=1e-4)
    args = parser.parse_args()

    limits = urdf_limits(URDF)
    if not limits or not MJCF_FILES:
        print("no urdf joints or mjcf files found")
        return 1
    failed = False
    for path in MJCF_FILES:
        errors = check_mjcf(path, limits, args.tol)
        rel = path.relative_to(ROOT)
        if errors:
            failed = True
            print(f"{rel}: {len(errors)} mismatch(es)")
            for err in errors:
                print(f"  {err}")
        else:
            print(f"{rel}: ok ({len(limits)} joints)")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
