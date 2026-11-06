#!/usr/bin/env python3
"""Build and run the firmware-logic host tests (BUILD_INSTRUCTIONS.md 8.1) and the C side of the
cross-check (8.3).

    py run_host_tests.py

Compiler: $CC if set (e.g. CC=gcc), else `python -m ziglang cc` (pip install ziglang).
Builds test_host.c against telem_core.c + diag_core.c three times (TELEM_BAMO_DIAG = 0, 1, 2)
with -Wall -Wextra -Werror, runs each, then builds xcheck and writes build/xcheck_*.bin.
"""
import os
import shlex
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
LORA = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
FW = os.path.join(LORA, "LoRa_board", "Core")
BUILD = os.path.join(HERE, "build")
CFLAGS = ["-std=c11", "-O1", "-g", "-Wall", "-Wextra", "-Werror", "-I", os.path.join(FW, "Inc")]


def cc():
    if os.environ.get("CC"):
        return shlex.split(os.environ["CC"])
    return [sys.executable, "-m", "ziglang", "cc"]


def exe(name):
    return os.path.join(BUILD, name + (".exe" if os.name == "nt" else ""))


def compile_(out, sources, defines):
    cmd = cc() + CFLAGS + [f"-D{d}" for d in defines] + sources + ["-o", out]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        print(r.stdout + r.stderr)
        raise SystemExit(f"compile failed: {os.path.basename(out)}")


def build_xcheck():
    """Build and run the cross-check writer; returns (packets.bin, scenario.bin)."""
    os.makedirs(BUILD, exist_ok=True)
    out = exe("xcheck")
    compile_(out, [os.path.join(HERE, "xcheck_main.c"), os.path.join(HERE, "xcheck_fill.c"),
                   os.path.join(FW, "Src", "telem_core.c")], ["TELEM_BAMO_DIAG=0"])
    pk, sc = os.path.join(BUILD, "xcheck_packets.bin"), os.path.join(BUILD, "xcheck_scenario.bin")
    subprocess.run([out, pk, sc], check=True)
    return pk, sc


def main():
    os.makedirs(BUILD, exist_ok=True)
    failed = []
    for flag in (0, 1, 2):
        out = exe(f"test_host_flag{flag}")
        compile_(out, [os.path.join(HERE, "test_host.c"), os.path.join(FW, "Src", "telem_core.c"),
                       os.path.join(FW, "Src", "diag_core.c")], [f"TELEM_BAMO_DIAG={flag}"])
        r = subprocess.run([out], capture_output=True, text=True)
        print(r.stdout, end="")
        if r.returncode:
            failed.append(flag)
    pk, sc = build_xcheck()
    print(f"xcheck: wrote {os.path.relpath(pk, LORA)} and {os.path.relpath(sc, LORA)}")
    if failed:
        raise SystemExit(f"host tests FAILED for TELEM_BAMO_DIAG={failed}")
    print("host tests: all passed")


if __name__ == "__main__":
    main()
