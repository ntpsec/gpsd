#!/bin/bash

# This file is Copyright 2010 by the GPSD project
# SPDX-License-Identifier: BSD-2-clause

set -e

logs='last'
if [ -n "$1" ]; then
    logs=$1
fi

if [ ! -x /usr/bin/getbuildlog ]; then
	echo 'Please install the devscripts package!'
	exit 1
fi

TMPDIR=`mktemp -d`
OLDPWD=`pwd`

cd ${TMPDIR}
getbuildlog gpsd $logs || true
grep -c -E -- '--- (./)?test' * | grep -E ':0$' | sed 's,^gpsd_[^_]*_\([^.]*\)\.log:0,\1: no regressions,'
grep -E -- '--- (./)?test' * | sed 's,\s2013.*,,;s,^gpsd_[^_]*_\([^.]*\)\.log:--- ,\1 ,' | sort -u
cd ${OLDPWD}
rm -rf ${TMPDIR}

