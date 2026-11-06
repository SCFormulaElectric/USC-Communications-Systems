#!/usr/bin/env python3
"""Command-line build of the LoRa module setup tool (same flags as the CubeIDE `Setup` config).

    py build.py              # -> build/LoRa_module_setup.elf / .bin, prints sizes
    py build.py --flash      # also flashes it over ST-LINK (STM32CubeProgrammer CLI from CubeIDE)
"""
import argparse
import glob
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "build")


def find(pattern, what):
    hits = sorted(glob.glob(pattern))
    if not hits:
        raise SystemExit(f"{what} not found ({pattern})")
    return hits[-1]


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.stdout.strip() or r.stderr.strip():
        print((r.stdout + r.stderr).rstrip())
    if r.returncode:
        raise SystemExit("failed")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flash", action="store_true")
    a = ap.parse_args()
    tc = os.environ.get("ARM_GCC_BIN") or find(
        "C:/ST/STM32CubeIDE_*/STM32CubeIDE/plugins/*gnu-tools-for-stm32*/tools/bin", "arm-none-eabi-gcc")
    gcc = os.path.join(tc, "arm-none-eabi-gcc")
    os.makedirs(OUT, exist_ok=True)
    cpu = ["-mcpu=cortex-m0", "-mthumb", "-mfloat-abi=soft"]
    cflags = cpu + ["-std=gnu11", "-DSTM32F042x6", "-Os", "-g3", "-ffunction-sections", "-fdata-sections",
                    "-Wall", "-Wextra", "--specs=nano.specs",
                    "-I", os.path.join(HERE, "Drivers/CMSIS/Device/ST/STM32F0xx/Include"),
                    "-I", os.path.join(HERE, "Drivers/CMSIS/Include")]
    objs = []
    for s in glob.glob(os.path.join(HERE, "Core/Src/*.c")) + glob.glob(os.path.join(HERE, "Core/Startup/*.s")):
        o = os.path.join(OUT, os.path.splitext(os.path.basename(s))[0] + ".o")
        run([gcc] + cflags + ["-c", s, "-o", o])
        objs.append(o)
    elf = os.path.join(OUT, "LoRa_module_setup.elf")
    run([gcc] + cpu + ["-T", os.path.join(HERE, "STM32F042F4PX_FLASH.ld"), "--specs=nosys.specs",
                       "--specs=nano.specs", "-Wl,--gc-sections", "-static"] + objs
        + ["-Wl,--start-group", "-lc", "-lm", "-Wl,--end-group", "-o", elf])
    run([os.path.join(tc, "arm-none-eabi-objcopy"), "-O", "binary", elf, elf[:-4] + ".bin"])
    run([os.path.join(tc, "arm-none-eabi-size"), elf])
    if a.flash:
        cli = find("C:/ST/STM32CubeIDE_*/STM32CubeIDE/plugins/*cubeprogrammer*/tools/bin/STM32_Programmer_CLI.exe",
                   "STM32_Programmer_CLI")
        run([cli, "-c", "port=SWD", "-w", elf[:-4] + ".bin", "0x08000000", "-v", "-rst"])


if __name__ == "__main__":
    main()
