#!/usr/bin/env bash
set -e
set -x



export PATH=/usr/sbin:/usr/bin:/sbin:/bin
if uname -a | grep -qi freebsd; then
    export PATH="${PATH}:/usr/local/bin:/usr/local/sbin"
fi

if [ "${USE_CCACHE}" = "true" ] && [ -n "${CCACHE_DIR}" ] && command -v ccache >/dev/null; then

    # create dir first, required by realpath
    mkdir -p "${CCACHE_DIR}"

    # fix for ccache: error: BASEDIR: not an absolute path
    CCACHE_DIR=$(realpath "${CCACHE_DIR}")
    export CCACHE_DIR
    CCACHE_BASEDIR=$(realpath "${CCACHE_BASEDIR}")
    export CCACHE_BASEDIR

    if [ -d "/usr/lib64/ccache" ]; then
        # fedora
        PATH="/usr/lib64/ccache:${PATH}"
    elif [ -d "/usr/local/libexec/ccache/" ]; then
        # freebsd
        PATH="/usr/local/libexec/ccache:${PATH}"
    else
        # debian, .....
        PATH="/usr/lib/ccache:${PATH}"
    fi
    export PATH
    echo 'max_size = 100M' > "${CCACHE_DIR}/ccache.conf"
else
    USE_CCACHE="false"
    export USE_CCACHE
fi

if command -v lsb_release >/dev/null && lsb_release -d | grep -qi -e debian -e ubuntu; then
    eval "$(dpkg-buildflags --export=sh)"
    DEB_HOST_MULTIARCH=$(dpkg-architecture -qDEB_HOST_MULTIARCH)
    export DEB_HOST_MULTIARCH
    PYTHONS="$(pyversions -v -r '>= 2.3'; py3versions -v -r '>= 3.4')"
    export PYTHONS
    SCONSOPTS="${SCONSOPTS} libdir=/usr/lib/${DEB_HOST_MULTIARCH}"
else
    SCONSOPTS="${SCONSOPTS} libdir=/usr/lib"
fi

SCONSOPTS="${SCONSOPTS} $* prefix=/usr \
	systemd=yes \
	nostrip=yes \
	dbus_export=yes \
	docdir=/usr/share/doc/gpsd \
	gpsd_user=gpsd \
	gpsd_group=dialout \
	debug=yes"
# set qt_versioned here only for non-CentOS systems (CentOS below)
if [ ! -f "/etc/centos-release" ]; then
    SCONSOPTS="${SCONSOPTS} qt_versioned=5"
fi

SCONS=$(command -v scons)
export SCONS

if command -v nproc >/dev/null; then
    NPROC=$(nproc)
    SCONS_PARALLEL="-j ${NPROC} "
    if [ "${SLOW_CHECK}" != "yes" ]; then
        CHECK_NPROC=$(( 4 * NPROC ))
        SCONS_CHECK_PARALLEL="-j ${CHECK_NPROC} "
    else
        SCONS_CHECK_PARALLEL="${SCONS_PARALLEL}"
    fi
else
    SCONS_PARALLEL=""
    SCONS_CHECK_PARALLEL=""
fi

if [ -f "/etc/centos-release" ]; then
    if grep -q "^CentOS Linux release 7" /etc/centos-release ; then
        PYTHONS="2"
        export PYTHONS
        # Qt version 5 currently doesn't compile, so omit versioning
        # for now
        #SCONSOPTS="${SCONSOPTS} qt_versioned=5"
    elif grep -q "^CentOS Linux release 8" /etc/centos-release ; then
        SCONS=$(command -v scons-3)
        export SCONS
        PYTHONS="3"
        export PYTHONS
        SCONSOPTS="${SCONSOPTS} qt_versioned=5"
    else
        # other CentOS versions not explicitly considered here, handle
        # as per non-CentOS logic
        SCONSOPTS="${SCONSOPTS} qt_versioned=5"
    fi
fi

export SCONSOPTS

if [ -z "$PYTHONS" ]; then
    PYTHONS="3"
    export PYTHONS
fi

if [ -n "${SCAN_BUILD}" ]; then
    PYTHONS="3"
    export PYTHONS
fi

for py in $PYTHONS; do
    _SCONS="${SCONS} target_python=python${py}"
    "python${py}"     ${_SCONS} ${SCONSOPTS} --clean
    rm -f .sconsign.*.dblite
    ${SCAN_BUILD} "python${py}"     ${_SCONS} ${SCONS_PARALLEL}${SCONSOPTS} build-all
    if [ -z "${NOCHECK}" ]; then
        "python${py}"     ${_SCONS} ${SCONS_CHECK_PARALLEL}${SCONSOPTS} check
    fi
done



if [ "${USE_CCACHE}" = "true" ]; then
    ccache -s
fi
