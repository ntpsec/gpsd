#!/usr/bin/env python
# This code run compatibly under Python 2 and 3.x for x >= 2.
# Preserve this property!
#
# This file is Copyright 2019 by the GPSD project
# SPDX-License-Identifier: BSD-2-Clause

"""Simple program to extract WMM2015 data for use in geoid.c

Extract 5x5 magnetic variations in WMM2015 from MagneticField."""

import sys
import subprocess


for lat in range(-90, 91, 5):
    if -90 != lat:
        sys.stdout.write("},")

    sys.stdout.write("\n    /* %d */\n    { " % lat)

    cnt = 0
    for lon in range(-180, 181, 5):
        ge = subprocess.Popen(["MagneticField"],
                              stdin=subprocess.PIPE,
                              stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE,
                              bufsize=0)
        # magnetic variation varies, pin at 1 Jan 2020
        out, err = ge.communicate(b"2020-01-01 %d %d\n" % (lat, lon))
        # round to even cm
        parts = out.split()
        val = round(float(parts[0]) * 100)
        sys.stdout.write("%6d" % val)
        cnt += 1
        if 0 == (cnt % 9):
            sys.stdout.write(",\n      ")
        elif 73 != cnt:
            sys.stdout.write(", ")

sys.stdout.write("}\n};\n")
