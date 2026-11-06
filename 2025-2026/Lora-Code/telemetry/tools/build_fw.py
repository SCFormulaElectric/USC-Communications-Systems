#!/usr/bin/env python3
"""Command-line firmware build for both flavours, with size report (BUILD_INSTRUCTIONS.md 6.1).

    py build_fw.py                 # Telemetry (flag 0) and Diag (flag 1)
    py build_fw.py --flags 0,1,2   # also the diagnostics-first build

Uses the GNU toolchain bundled with STM32CubeIDE (found under C:/ST, or set ARM_GCC_BIN to its
bin folder). Same flags as the CubeIDE `Telemetry` / `Diag` configurations: -Os, -ffunction-
sections, --gc-sections, nano specs. Output: telemetry/tools/build/<flavour>/LoRa_board.elf/.bin/.map
Fails if flash > 16 KB or RAM (data + bss + min heap + min stack) > 6 KB.
"""
import argparse
import glob
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.normpath(os.path.join(HERE, "..", "..", "LoRa_board"))
OUT = os.path.join(HERE, "build")
FLASH, RAM = 16 * 1024, 6 * 1024
MIN_HEAP, MIN_STACK = 0x200, 0x400          # from STM32F042F4PX_FLASH.ld
NAMES = {0: "Telemetry", 1: "Diag", 2: "DiagFirst"}


def toolchain():
    env = os.environ.get("ARM_GCC_BIN")
    if env:
        return env
    hits = glob.glob("C:/ST/STM32CubeIDE_*/STM32CubeIDE/plugins/*gnu-tools-for-stm32*/tools/bin")
    if not hits:
        raise SystemExit("arm-none-eabi-gcc not found: set ARM_GCC_BIN to the toolchain bin folder")
    return sorted(hits)[-1]


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode or r.stderr.strip():
        sys.stdout.write(r.stdout + r.stderr)
    if r.returncode:
        raise SystemExit("build failed")
    return r.stdout


def build(flag, tc, opt="-Os"):
    gcc = os.path.join(tc, "arm-none-eabi-gcc")
    out = os.path.join(OUT, NAMES[flag] + ("" if opt == "-Os" else opt.replace("-", "_")))
    os.makedirs(out, exist_ok=True)
    cpu = ["-mcpu=cortex-m0", "-mthumb", "-mfloat-abi=soft"]
    cflags = cpu + ["-std=gnu11", "-DUSE_HAL_DRIVER", "-DSTM32F042x6", f"-DTELEM_BAMO_DIAG={flag}",
                    opt, "-ffunction-sections", "-fdata-sections", "-Wall", "-Wextra", "--specs=nano.specs"]
    for d in ("Core/Inc", "Drivers/STM32F0xx_HAL_Driver/Inc", "Drivers/STM32F0xx_HAL_Driver/Inc/Legacy",
              "Drivers/CMSIS/Device/ST/STM32F0xx/Include", "Drivers/CMSIS/Include"):
        cflags += ["-I", os.path.join(PROJ, d)]
    srcs = sorted(glob.glob(os.path.join(PROJ, "Core", "Src", "*.c")))
    srcs += sorted(glob.glob(os.path.join(PROJ, "Drivers", "STM32F0xx_HAL_Driver", "Src", "*.c")))
    srcs += glob.glob(os.path.join(PROJ, "Core", "Startup", "*.s"))
    objs = []
    for s in srcs:
        o = os.path.join(out, os.path.splitext(os.path.basename(s))[0] + ".o")
        extra = ["-Wno-unused-parameter"] if "Drivers" in s else []   # vendor HAL code
        run([gcc] + cflags + extra + ["-c", s, "-o", o])
        objs.append(o)
    elf = os.path.join(out, "LoRa_board.elf")
    run([gcc] + cpu + ["-T", os.path.join(PROJ, "STM32F042F4PX_FLASH.ld"), "--specs=nosys.specs",
                       "--specs=nano.specs", "-Wl,--gc-sections", "-static", f"-Wl,-Map={elf[:-4]}.map"]
        + objs + ["-Wl,--start-group", "-lc", "-lm", "-Wl,--end-group", "-o", elf])
    run([os.path.join(tc, "arm-none-eabi-objcopy"), "-O", "binary", elf, elf[:-4] + ".bin"])
    size = run([os.path.join(tc, "arm-none-eabi-size"), elf])
    text, data, bss = (int(x) for x in size.splitlines()[1].split()[:3])
    return elf, size, text, data, bss


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flags", default="0,1")
    ap.add_argument("--opt", default="-Os", help="optimisation flag (the CubeIDE Debug config uses -O0)")
    args = ap.parse_args()
    tc = toolchain()
    ok = True
    for flag in (int(f) for f in args.flags.split(",")):
        elf, size, text, data, bss = build(flag, tc, args.opt)
        flash, ram = text + data, data + bss
        ram_total = ram + MIN_HEAP + MIN_STACK
        print(f"== {NAMES[flag]} (TELEM_BAMO_DIAG={flag})  {os.path.relpath(elf, os.path.dirname(PROJ))}")
        print(size.rstrip())
        print(f"   flash {flash} / {FLASH} B ({100 * flash / FLASH:.1f}%), free {FLASH - flash} B")
        print(f"   RAM   {ram} B static + {MIN_HEAP} heap + {MIN_STACK} stack = {ram_total} / {RAM} B "
              f"({100 * ram_total / RAM:.1f}%)")
        if flash > FLASH or ram_total > RAM:
            print("   DOES NOT FIT")
            ok = False
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
