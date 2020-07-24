#!/usr/bin/env python3

"""Main."""

import sys
from cpu import *
import re

cpu = CPU()

if len(sys.argv) != 2:
    print("Missing program file argument!")

ls8_re = re.compile(r"^(?P<byte>[01]{8})?(\s*#(?P<comment>.*))?")

filename = sys.argv[1]
program = []

with open(filename, "r") as f:

    for line in f.readlines():
        m = ls8_re.match(line)
        if m and m.group("byte"):
            program.append(int(m.group("byte"), 2))

try:
    cpu.load(program)
    cpu.run()
except KeyboardInterrupt:
    exit()
