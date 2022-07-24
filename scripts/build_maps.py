"""
Automatically find *.map and compile and publish them
"""

from genericpath import isdir
import os
from argparse import ArgumentParser
import subprocess

ERICW_TOOLS_DIR = "../ericw-tools/build"
QBSP_CMD = os.path.join(ERICW_TOOLS_DIR, "qbsp/qbsp")
BSPINFO_CMD = os.path.join(ERICW_TOOLS_DIR, "bspinfo/bspinfo")
BSPUTIL_CMD = os.path.join(ERICW_TOOLS_DIR, "bsputil/bsputil")
LIGHT_CMD = os.path.join(ERICW_TOOLS_DIR, "light/light")
VIS_CMD = os.path.join(ERICW_TOOLS_DIR, "vis/vis")

Q2TOOLS_DIR = "../q2tools-220/release"
BSP4_CMD = os.path.join(Q2TOOLS_DIR, "4bsp")
DATA4_CMD = os.path.join(Q2TOOLS_DIR, "4data")
VIS4_CMD = os.path.join(Q2TOOLS_DIR, "4vis")
RAD4_CMD = os.path.join(Q2TOOLS_DIR, "4rad")

silent = False

def exec(*args):
    if not silent:
        print(" ".join(args))

    result = subprocess.run(
        args, stdout=subprocess.PIPE
    ).stdout.decode('utf-8')

    if not silent:
        [print(f"> {line}") for line in result.split("\n")]

    return result

def AmendArgs(pargs : ArgumentParser):
    pargs.add_argument("-q", "--quiet", action="store_true")

def ArgsParsed(args):
    silent = args.quiet

if __name__ == "__main__":
    args = ArgumentParser()

    args.add_argument("-m", "--map")
    args.add_argument("-o", "--output", required=False)
    args.add_argument("-q220", default=False, action="store_true")

    cmdline = args.parse_args()
    workmap = cmdline.map.replace(".map", ".bsp")

    if not cmdline.output:
        cmdline.output = workmap

    if os.path.isdir(cmdline.output):
        cmdline.output = os.path.join(cmdline.output, os.path.basename(workmap))

    print(f"compiling {cmdline.map} to {cmdline.output}")

    if cmdline.q220:
        exec(BSP4_CMD, "-v", "-qbsp", cmdline.map)
        exec(VIS4_CMD, "-fast", "-v", workmap)
        exec(RAD4_CMD, "-bounce", "1", "-v", workmap)
    else:
        exec(QBSP_CMD, "-qbism", "-subdivide", "1024", cmdline.map)
        exec(VIS_CMD, workmap)
        exec(LIGHT_CMD, "-bounce", "1", workmap)


    exec("cp", workmap, cmdline.output)