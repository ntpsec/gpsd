#!/usr/bin/env python3
#
# This file is Copyright 2010 by the GPSD project
# SPDX-License-Identifier: BSD-2-clause
#
# This code runs compatibly under Python 2 and 3.x for x >= 2.
# Preserve this property!
# Codacy D203 and D211 conflict, I choose D203
# Codacy D212 and D213 conflict, I choose D212

"""Program to generate code for gpsd masks.

With -p, dump a Python status mask list translated from the C one.

With -c, generate C code to dump masks for debugging purposes.

With -t, tabulate usage of defines to find unused ones.  Requires -c or -d.
"""

from __future__ import absolute_import, print_function, division

import argparse
import glob
import sys

try:
    from subprocess import getstatusoutput
except ImportError:
    from commands import getstatusoutput


class SourceExtractor(object):

    """SourceExtractor Class."""

    def __init__(self, sourcefile, clientside):
        """Init for SourceExtractor."""
        self.sourcefile = sourcefile
        self.clientside = clientside
        self.daemonfiles = [
            "gpsd.c",
            "libgpsd_core.c",
            "pseudonmea.c",
            "drivers.c",
            "gpsmon/gpsmon.c",
            "subframe.c"
        ] + glob.glob("driver_*.c") + glob.glob("gpsmon/monitor_*.c")
        self.masks = []
        self.primitive_masks = []
        for line in open(self.sourcefile):
            if (((line.startswith("#define") and
                 ("_SET" in line or "_IS" in line)))):
                fields = line.split()
                self.masks.append((fields[1], fields[2]))
                if ((fields[2].startswith("(1llu<<") or
                     fields[2].startswith("INTERNAL_SET"))):
                    self.primitive_masks.append((fields[1], fields[2]))

    def in_library(self, flg):
        """Grep in library."""
        (status, _output) = getstatusoutput(
            "grep '%s' libgps/libgps_core.c libgps/libgps_json.c gpsctl.c" %
            flg)
        return status == 0

    def in_daemon(self, flg):
        """Grep in daemon."""
        (status, _output) = getstatusoutput(
            "grep '%s' %s" % (flg, " ".join(self.daemonfiles)))
        return status == 0

    def relevant(self, flg):
        """Relevant??"""
        if self.clientside:
            return self.in_library(flg)

        return self.in_daemon(flg)


if __name__ == '__main__':
    description = 'Tool for reporting gpsd status masks.'
    usage = '%(prog)s [OPTIONS] [host[:port[:device]]]'
    epilog = ('BSD terms apply: see the file COPYING in the distribution root'
              ' for details.')

    parser = argparse.ArgumentParser(
        description=description,
        epilog=epilog,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        usage=usage)
    parser.add_argument(
        '-?',
        action="help",
        help='show this help message and exit'
    )
    parser.add_argument(
        '-d',
        '--daemongen',
        action="store_true",
        default=False,
        dest='daemongen',
        help=('Generate C code to dump masks for debugging'
              ' [Default %(default)s]'),
    )
    parser.add_argument(
        '-c',
        '--clientgen',
        action="store_true",
        default=False,
        dest='clientgen',
        help=('Generate C code to dump masks for debugging'
              ' [Default %(default)s]'),
    )
    parser.add_argument(
        '-p',
        '--pythonize',
        action="store_true",
        default=False,
        dest='pythonize',
        help=('Dump a Python status mask list. [Default %(default)s]'),
    )
    parser.add_argument(
        '-t',
        '--tabulate',
        action="store_true",
        default=False,
        dest='tabulate',
        help=('Tabulate usage of defines to find unused ones. '
              ' Requires -c or -d. [Default %(default)s]'),
    )
    # parser.add_argument(
    #     '-V', '--version',
    #     action='version',
    #     help='Output version to stderr, then exit',
    #     version="%(prog)s: Version " + gps_version + "\n",
    # )
    parser.add_argument(
        'arguments',
        help='Source Directory',
        nargs='?',
    )
    options = parser.parse_args()

    if not options.arguments:
        srcdir = '.'
    else:
        srcdir = options.arguments

    clientside = SourceExtractor(srcdir + "/include/gps.h",
                                 clientside=True)
    daemonside = SourceExtractor(srcdir + "/include/gpsd.h",
                                 clientside=False)
    if options.clientgen:
        source = clientside
        banner = "Library"
    elif options.daemongen:
        source = daemonside
        banner = "Daemon"
    else:
        sys.stderr.write("maskaudit: need -c or -d option\n")
        sys.exit(1)

    try:
        if options.tabulate:
            print("%-14s        %8s" % (" ", banner))
            for (flag, value) in source.masks:
                print("%-14s    %8s" % (flag, source.relevant(flag)))
        if options.pythonize:
            for (d, v) in source.masks:
                if v[-1] == 'u':
                    v = v[:-1]
                print("%-15s\t= %s" % (d, v))
        if not options.pythonize and not options.tabulate:
            maxout = 0
            for (d, v) in source.primitive_masks:
                if source.relevant(d):
                    stem = d
                    if stem.endswith("_SET"):
                        stem = stem[:-4]
                    if stem.endswith("_IS"):
                        stem = stem[:-3]
                    maxout += len(stem) + 1
            print("""
// This code is generated by maskaudit.py.  Do not hand-hack it!

/*
 * Also, beware that it is something of a CPU hog when called on every packet.
 * Try to write guards so it is only called at higher log levels.
 */

#include \"../include/gpsd_config.h\"  /* must be before all includes */

#include <stdio.h>
#include <string.h>

#include \"../include/gpsd.h\"

const char *gps_maskdump(gps_mask_t set)
{
    static char buf[%d];
    const struct {
        gps_mask_t      mask;
        const char      *name;
    } *sp, names[] = {""" % (maxout + 3,))
            masks = clientside.primitive_masks + daemonside.primitive_masks
            for (flag, value) in masks:
                stem = flag
                if stem.endswith("_SET"):
                    stem = stem[:-4]
                if stem.endswith("_IS"):
                    stem = stem[:-3]
                print("        {%s,\t\"%s\"}," % (flag, stem))
            print('''\
    };

    memset(buf, '\\0', sizeof(buf));
    buf[0] = '{';
    for (sp = names; sp < names + sizeof(names)/sizeof(names[0]); sp++)
        if ((set & sp->mask)!=0) {
            (void)strlcat(buf, sp->name, sizeof(buf));
            (void)strlcat(buf, "|", sizeof(buf));
        }
    if (buf[1] != \'\\0\')
        buf[strlen(buf)-1] = \'\\0\';
    (void)strlcat(buf, "}", sizeof(buf));
    return buf;
}
''')
    except KeyboardInterrupt:
        pass

# The following sets edit modes for GNU EMACS
# Local Variables:
# mode:python
# End:
# vim: set expandtab shiftwidth=4
