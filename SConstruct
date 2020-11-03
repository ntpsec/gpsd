# see library directories " SCons build recipe for the GPSD project

# Important targets:
#
# build      - build the software (default)
# dist       - make distribution tarball (requires GNU tar)
# install    - install programs, libraries, and manual pages
# uninstall  - undo an install
#
# check      - run regression and unit tests.
# audit      - run code-auditing tools
# testbuild  - test-build the code from a tarball
# website    - refresh the website
# release    - ship a release
#
# --clean    - clean all normal build targets
#
# Setting the DESTDIR environment variable will prefix the install destinations
# without changing the --prefix prefix.
#
# Pretty much all this file does is create buildtmp, and call SConscript.
#
# Unfinished items:
# * Coveraging mode: gcc "-coverage" flag requires a hack
#   for building the python bindings
#
# This file is Copyright 2010 by the GPSD project
# SPDX-License-Identifier: BSD-2-clause
#
# This code runs compatibly under Python 2 and 3.x for x >= 2.
# Preserve this property!
from __future__ import print_function

import ast
import atexit      # for atexit.register()
import functools
import glob
import operator
import os
import pickle
import re
# replacement for functions from the commands module, which is deprecated.
import subprocess
import sys
import time
from distutils import sysconfig
import SCons

# Facilitate debugging with pdb.
# At pdb startup, the environment is such that setting breakpoints in
# SConstruct requires specifying its full absolute path, which is incovenient.
# Stopping here at an automatic breakpoint makes this easier.  Note that this
# code has no effect unless pdb is loaded.
# To use this, run with pdb and continue from the initial pdb prompt.
pdb_module = sys.modules.get('pdb')
if pdb_module:
    pdb_module.set_trace()
    pass  # Breakpoint default file is now SConstruct


# gpsd needs Scons version at least 2.3
EnsureSConsVersion(2, 3, 0)
# gpsd needs Python version at least 2.6
EnsurePythonVersion(2, 6)

# Have scons rebuild an existing target when the source timestamp changes
# and the MD5 changes.  To prevent rebuidling when gpsd_config.h rebuilt,
# with no  changes.
Decider('MD5-timestamp')

# support building with various Python versions.
sconsign_file = '.sconsign.{}.dblite'.format(pickle.HIGHEST_PROTOCOL)
SConsignFile(os.getcwd() + os.path.sep + sconsign_file)

# Start by reading configuration variables from the cache
opts = Variables('.scons-option-cache')

# one touch clean!
if GetOption('clean'):
    atexit.register(lambda: os.system("rm -rf buildtmp"))

# name 'build' is already taken
# Not everything respects this  chdir()
SConscriptChdir(1)
# SConscript('SConscript', variant_dir='buildtmp', must_exit=True, duplicate=1)
VariantDir('buildtmp', '.')
SConscript('buildtmp/SConscript', must_exit=True, duplicate=1)
