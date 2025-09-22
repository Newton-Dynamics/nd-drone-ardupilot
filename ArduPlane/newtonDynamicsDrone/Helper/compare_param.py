#!/usr/bin/env python3
"""
Compare two ArduPilot-style parameter files, even with different line formats.

Usage (recommended, non-interactive):
  python compare_params.py --first file1.param --second file2.param --output diff.csv

If any of --first/--second/--output is omitted, you'll be prompted:
  First File:
  Second File:
  Output:

Supported line formats:
  NAME <spaces> VALUE
  NAME,VALUE[,extra...]
  NAME=VALUE
Tolerates inline comments (#, //, ;), headers (Param,Value), blanks.
"""

import argparse
import csv
from decimal import Decimal, InvalidOperation
from pathlib import Path
import sys

COMMENT_MARKERS = ("//", "#", ";")

def _strip_inline_comment(s: str) -> str:
    for m in COMMENT_MARKERS:
        idx = s.find(m)
        if idx != -1:
            s = s[:idx]
    return s

def _first_token(value: str) -> str:
    v = value.strip().strip(",")
    if "," in v:
        v = v.split(",", 1)[0]
    parts = v.split()
    return parts[0] if parts else ""

def _parse_value(value_str: str):
    raw = _first_token(_strip_inline_comment(value_str).strip().strip('"').strip("'"))
    if raw == "":
        return {"raw": "", "num": None}
    try:
        num = Decimal(raw)
    except InvalidOperation:
        num = None
    return {"raw": raw, "num": num}

def parse_param_file(p: Path):
    """
    Returns dict: { NAME: {"raw": str, "num": Decimal|None} }
    Last occurrence of a repeated NAME wins.
    """
    out = {}
    with p.open("r", encoding="utf-8-sig", errors="replace") as f:
        for line in f:
            s = line.strip()
            if not s or any(s.startswith(m) for m in COMMENT_MARKERS):
                continue

            name, val = None, None
            if "," in s:
                left, right = s.split(",", 1)
                name, val = left.strip(), right.strip()
            elif "=" in s:
                left, right = s.split("=", 1)
                name, val = left.strip(), right.strip()
            else:
                parts = s.split(None, 1)
                if len(parts) == 2:
                    name, val = parts[0].strip(), parts[1].strip()
                else:
                    continue

            if name.lower() in {"param", "parameter", "name"}:
                continue

            out[name] = _parse_value(val)
    return out

def values_equal(v1, v2):
    if v1["num"] is not None and v2["num"] is not None:
        return v1["num"] == v2["num"]
    return v1["raw"].strip() == v2["raw"].strip()

def prompt_path(label: str, must_exist: bool, default: str | None = None) -> Path:
    while True:
        prompt = f"{label} " + (f"[{default}]: " if default else ": ")
        entered = input(prompt).strip()
        if not entered and default:
            entered = default
        if not entered:
            continue
        p = Path(entered).expanduser()
        if must_exist:
            if p.exists() and p.is_file():
                return p.resolve()
            print(f"  -> File not found: {p}")
        else:
            # ensure parent exists
            if p.suffix == "":
                p = p.with_suffix(".csv")
            p.parent.mkdir(parents=True, exist_ok=True)
            return p.resolve()

def main():
    ap = argparse.ArgumentParser(description="Compare two .param/.parm files (robust formats).")
    ap.add_argument("--first", "--file1", "-1", dest="file1", type=Path, help="First parameter file")
    ap.add_argument("--second", "--file2", "-2", dest="file2", type=Path, help="Second parameter file")
    ap.add_argument("--output", "-o", dest="output", type=Path, help="Output CSV path")
    ap.add_argument("--order", choices=["alpha", "file1"], default="alpha",
                    help="Row order: alphabetical (default) or preserve file1 order then extras")
    args = ap.parse_args()

    # Interactive prompts if any argument is missing
    file1 = args.file1 if args.file1 else prompt_path("First File:", must_exist=True)
    file2 = args.file2 if args.file2 else prompt_path("Second File:", must_exist=True)

    if file1.resolve() == file2.resolve():
        print("Error: First and Second files are the same path.", file=sys.stderr)
        sys.exit(2)

    default_out = Path.cwd() / f"{file1.stem}_vs_{file2.stem}_diff.csv"
    output = args.output if args.output else prompt_path("Output:", must_exist=False, default=str(default_out))

    d1 = parse_param_file(file1)
    d2 = parse_param_file(file2)

    if args.order == "file1":
        seen = set()
        ordered = []
        for k in d1.keys():
            if k not in seen:
                seen.add(k)
                ordered.append(k)
        for k in d2.keys():
            if k not in seen:
                seen.add(k)
                ordered.append(k)
        all_params = ordered
    else:
        all_params = sorted(set(d1.keys()) | set(d2.keys()))

    with output.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["param", "file1_value", "file2_value", "same_name", "unknown_parameter", "same_value"])
        for name in all_params:
            in1, in2 = name in d1, name in d2
            same_name = in1 and in2
            v1_raw = d1[name]["raw"] if in1 else ""
            v2_raw = d2[name]["raw"] if in2 else ""

            if not in1 and in2:
                unknown = "unknown in file1"
                same_val = ""
            elif in1 and not in2:
                unknown = "unknown in file2"
                same_val = ""
            else:
                unknown = ""
                same_val = values_equal(d1[name], d2[name])

            writer.writerow([name, v1_raw, v2_raw, same_name, unknown, same_val])

    print(f"\nFirst File : {file1}")
    print(f"Second File: {file2}")
    print(f"Output     : {output}")
    print("Done.")

if __name__ == "__main__":
    main()
