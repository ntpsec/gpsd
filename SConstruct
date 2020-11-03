""" SCons build recipe for the GPSD project

Important targets:

build      - build the software (default)
dist       - make distribution tarball (requires GNU tar)
install    - install programs, libraries, and manual pages
uninstall  - undo an install

check      - run regression and unit tests.
audit      - run code-auditing tools
testbuild  - test-build the code from a tarball
website    - refresh the website
release    - ship a release

--clean    - clean all normal build targets
-c         - clean all normal build targets

Setting the DESTDIR environment variable will prefix the install destinations
without changing the --prefix prefix.

Pretty much all this file does is create buildtmp, and call SConscript.
"""

# Unfinished items:
# * Coveraging mode: gcc "-coverage" flag requires a hack
#   for building the python bindings

# This file is Copyright 2010 by the GPSD project
# SPDX-License-Identifier: BSD-2-clause
#
# This code runs compatibly under Python 2 and 3.x for x >= 2.
# Preserve this property!
from __future__ import print_function

import atexit      # for atexit.register()
import os

# gpsd needs Scons version at least 2.3
EnsureSConsVersion(2, 3, 0)
# gpsd needs Python version at least 2.6
EnsurePythonVersion(2, 6)

# one touch clean!
if GetOption('clean'):
    atexit.register(lambda: os.system("rm -rf buildtmp"))

# name 'build' is already taken
# Not everything respects this  chdir()
SConscriptChdir(1)
# SConscript('SConscript', variant_dir='buildtmp', must_exit=True, duplicate=1)
VariantDir('buildtmp', '.')
SConscript('buildtmp/SConscript', must_exit=True, duplicate=1)
