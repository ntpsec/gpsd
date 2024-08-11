# scons file.  Only meant to be run from SContruct.
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
try:
    # sysconfig since 2.7, still in 3.13
    import sysconfig
    PYTHON_SYSCONFIG_IMPORT = 'import sysconfig'
except:
    # distutils gone in 3.13, but needed for 2.6
    from distutils import sysconfig
    PYTHON_SYSCONFIG_IMPORT = 'from distutils import sysconfig'
import SCons

# scons does not like targets that come and go (if cleaning, if python,
# etc). All targets are needed for proper cleaning. If a target should
# not be built (if not python), then do not include the target in the
# next layer sources.

# scons gets confused by targets that are not a real file (shmclean, etc.).
# Set them Pseudo (like in a Makefile) and use an Alias() for them.

# Note: Do not use context.TryRun() as that breaks cross-compiling

# Facilitate debugging with pdb.
# At pdb startup, the environment is such that setting breakpoints in
# SConstruct requires specifying its full absolute path, which is incovenient.
# Stopping here at an automatic breakpoint makes this easier.  Note that this
# code has no effect unless pdb is loaded.
# To use this, run with pdb and continue from the initial pdb prompt.
pdb_module = sys.modules.get('pdb')
if pdb_module:
    pdb_module.set_trace()
    pass  # Breakpoint default file is now SConscript

# gpsd needs Scons version at least 2.3
EnsureSConsVersion(2, 3, 0)
# gpsd needs Python version at least 2.6
EnsurePythonVersion(2, 6)

# By user choice, or due to system-dependent availability, the scons
# executable may be called using names other than plain "scons",
# e.g. "scons-3" on CentOS 8.
scons_executable_name = os.path.basename(sys.argv[0]) or 'scons'


# Have scons rebuild an existing target when the source(s) MD5 changes
# Do not use time to prevent rebuilding when sources, like gpsd_config.h,
# are rebuilt, but with no changes.
Decider('MD5')

# Put .sconsign*dblite and .scons-options-cache in variantdir for
# one-touch cleaning
# support building with various Python versions.
sconsign_file = '.sconsign.%d.dblite' % pickle.HIGHEST_PROTOCOL
SConsignFile(os.getcwd() + os.path.sep + sconsign_file)

# Start by reading configuration variables from the cache
opts = Variables('.scons-option-cache')


# ugly hack from http://www.catb.org/esr/faqs/practical-python-porting/
# handle python2/3 strings
def polystr(o):
    if bytes is str:  # Python 2
        return str(o)

    # python 3.
    if isinstance(o, str):
        return o
    if isinstance(o, (bytes, bytearray)):
        return str(o, encoding='latin1')
    if isinstance(o, int):
        return str(o)

    raise ValueError


def strtobool (val):
    val = val.lower()
    if val in ('y', 'yes', 't', 'true', 'on', '1'):
        return True
    elif val in ('n', 'no', 'f', 'false', 'off', '0'):
        return False
    else:
        raise ValueError("invalid truth value %r" % (val,))


# Helper functions for revision hackery
def GetMtime(file):
    """Get mtime of given file, or 0."""
    try:
        return os.stat(file).st_mtime
    except OSError:
        return 0


def FileList(patterns, exclusion=None):
    """Get list of files based on patterns, minus excluded path."""
    files = functools.reduce(operator.add, map(glob.glob, patterns), [])
    for file in files:
        if file.find(exclusion):
            files.remove(file)
    return files


# FIXME: replace with TryAction()
def _getstatusoutput(cmd, nput=None, shell=True, cwd=None, env=None):
    pipe = subprocess.Popen(cmd, shell=shell, cwd=cwd, env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    (output, errout) = pipe.communicate(input=nput)
    status = pipe.returncode
    return (status, output)


# Workaround for old SCons bug that couldn't overwrite existing symlinks
# This was fixed in 2.3.2, but we allow 2.3.0 (e.g., on Ubuntu 14)
#
# In the troublesome versions, we monkey-patch os.symlink to bully it through

standard_os_symlink = os.symlink


def _forced_symlink(source, link_name):
    try:
        standard_os_symlink(source, link_name)
    except OSError:
        # Out of paranoia, only do this when the target is a symlink
        if os.path.islink(link_name):
            os.remove(link_name)
            standard_os_symlink(source, link_name)
        else:
            raise


if SCons.__version__ in ['2.3.0', '2.3.1']:
    os.symlink = _forced_symlink


# SCons 2.3.0 is also missing the Psuedo method.  See the workaround after
# the initial 'env' setup.

# All man pages.  Always build them all.
all_manpages = {
    "man/cgps.1": "man/cgps.adoc",
    "man/gegps.1": "man/gegps.adoc",
    "man/gps.1": "man/gps.adoc",
    "man/gps2udp.1": "man/gps2udp.adoc",
    "man/gpscsv.1": "man/gpscsv.adoc",
    "man/gpscat.1": "man/gpscat.adoc",
    "man/gpsctl.1": "man/gpsctl.adoc",
    "man/gpsd.8": "man/gpsd.adoc",
    "man/gpsdebuginfo.1": "man/gpsdebuginfo.adoc",
    "man/gpsdctl.8": "man/gpsdctl.adoc",
    "man/gpsdecode.1": "man/gpsdecode.adoc",
    "man/gpsd_json.5": "man/gpsd_json.adoc",
    "man/gpsfake.1": "man/gpsfake.adoc",
    "man/gpsinit.8": "man/gpsinit.adoc",
    "man/gpsmon.1": "man/gpsmon.adoc",
    "man/gpspipe.1": "man/gpspipe.adoc",
    "man/gpsplot.1": "man/gpsplot.adoc",
    "man/gpsprof.1": "man/gpsprof.adoc",
    "man/gpsrinex.1": "man/gpsrinex.adoc",
    "man/gpssnmp.1": "man/gpssnmp.adoc",
    "man/gpssubframe.1": "man/gpssubframe.adoc",
    "man/gpxlogger.1": "man/gpxlogger.adoc",
    "man/lcdgps.1": "man/lcdgps.adoc",
    "man/libgps.3": "man/libgps.adoc",
    "man/libgpsmm.3": "man/libgpsmm.adoc",
    "man/libQgpsmm.3": "man/libgpsmm.adoc",
    "man/ntpshmmon.1": "man/ntpshmmon.adoc",
    "man/ppscheck.8": "man/ppscheck.adoc",
    "man/ubxtool.1": "man/ubxtool.adoc",
    "man/xgps.1": "man/xgps.adoc",
    "man/xgpsspeed.1": "man/xgpsspeed.adoc",
    "man/zerk.1": "man/zerk.adoc",
}

# doc files to install in share/gpsd/doc
doc_files = [
    'AUTHORS',
    'build.adoc',
    'COPYING',
    'www/example1.c.txt',
    'www/example2.py.txt',
    'NEWS',
    'README.adoc',
    'SUPPORT.adoc',
]

# doc files to install in share/gpsd/doc/
icon_files = [
    'packaging/X11/gpsd-logo.png',
]

# MIB files to install in $PREFIX/share/snmp/mibs/gpsd/
mib_files = [
    'man/GPSD-MIB',
]
mib_lint = (mib_files + ['SConstruct', 'SConscript'])

# gpsd_version, and variantdir, from SConstruct
Import('*')

# Create a fixed-name symlink to the build tree, for scripts and symlinks.
# FIXME: Make this work with Execute()
vdir_parent = os.path.dirname(os.path.abspath(os.path.dirname(variantdir)))
try:
    os.symlink(variantdir, os.path.join(vdir_parent, 'buildtmp'))
except OSError:
    pass  # May already exist

# API (JSON) version
api_version_major = 3
api_version_minor = 15

# client library version
libgps_version_current = 30
libgps_version_revision = 0
libgps_version_age = 0
libgps_version = "%d.%d.%d" % (libgps_version_current, libgps_version_age,
                               libgps_version_revision)
#
# Release identification ends here

# Hosting information (mainly used for templating web pages) begins here
# Each variable foo has a corresponding @FOO@ expanded in .in files.
# There are no project-dependent URLs or references to the hosting site
# anywhere else in the distribution; preserve this property!
annmail = "gpsd-announce@nongnu.org"
bugtracker = "https://gitlab.com/gpsd/gpsd/-/issues"
cgiupload = "root@thyrsus.com:/var/www/cgi-bin/"
clonerepo = "git@gitlab.com:gpsd/gpsd.git"
devmail = "gpsd-dev@lists.nongnu.org"
download = "http://download-mirror.savannah.gnu.org/releases/gpsd/"
formserver = "www@thyrsus.com"
gitrepo = "git@gitlab.com:gpsd/gpsd.git"
ircchan = "irc://chat.libera.chat:6697/#gpsd"
mailman = "https://lists.nongnu.org/mailman/listinfo/"
mainpage = "https://gpsd.io"
projectpage = "https://gitlab.com/gpsd/gpsd"
scpupload = "garyemiller@dl.sv.nongnu.org:/releases/gpsd/"
sitename = "GPSD"
sitesearch = "gpsd.io"
tiplink = "<a href='https://www.patreon.com/esr'>" \
          "leave a remittance at Patreon</a>"
tipwidget = '<p><a href="https://www.patreon.com/esr">' \
            'Donate here to support continuing development.</a></p>'
usermail = "gpsd-users@lists.nongnu.org"
webform = "http://www.thyrsus.com/cgi-bin/gps_report.cgi"
website = "https://gpsd.io/"
# Hosting information ends here



# Utility productions


def Utility(target, source, action, **kwargs):
    targ = env.Command(target=target, source=source, action=action, **kwargs)
    # why always build?  wasteful?
    # when gpsdecode is the source this rebuilds the entire daemon!
    # env.AlwaysBuild(targ)
    # Why precious?
    env.Precious(targ)
    # Is pseudo really needed (didn't used to be)?
    env.Pseudo(targ)
    # Alias to make name work without variantdir prefix
    env.Alias(target, targ)
    return targ


def UtilityWithHerald(herald, target, source, action, **kwargs):
    if not env.GetOption('silent'):
        action = ['@echo "%s"' % herald] + action
    return Utility(target=target, source=source, action=action, **kwargs)


# Spawn replacement that suppresses non-error stderr
def filtered_spawn(sh, escape, cmd, args, env):
    proc = subprocess.Popen([sh, '-c', ' '.join(args)],
                            env=env, close_fds=True, stderr=subprocess.PIPE)
    _, stderr = proc.communicate()
    if proc.returncode:
        sys.stderr.write(stderr)
    return proc.returncode

#
# Build-control options
#


# guess systemd defaults
systemd_dir = '/lib/systemd/system'
systemd = os.path.exists(systemd_dir)

# Set distribution-specific defaults here
imloads = True

boolopts = (
    # GPS protocols
    # for back compatibility, deprecated Feb 2021
    ("ashtech",       True,  "alias for NMEA0183 support, deprecated"),
    ("earthmate",     True,  "DeLorme EarthMate Zodiac support"),
    ("evermore",      True,  "EverMore binary support"),
    ("fury",          True,  "Jackson Labs Fury and Firefly support"),
    ("fv18",          True,  "San Jose Navigation FV-18 support"),
    ("garmin",        True,  "Garmin kernel driver support"),
    ("garmintxt",     True,  "Garmin Simple Text support"),
    ("geostar",       True,  "Geostar Protocol support"),
    ("greis",         True,  "Javad GREIS support"),
    ("itrax",         True,  "iTrax hardware support"),
    ("navcom",        True,  "Navcom NCT support"),
    ("nmea2000",      True,  "NMEA2000/CAN support"),
    ("oncore",        True,  "Motorola OnCore chipset support"),
    ("sirf",          True,  "SiRF chipset support"),
    ("skytraq",       True,  "Skytraq chipset support"),
    ("superstar2",    True,  "Novatel SuperStarII chipset support"),
    ("tnt",           True,  "True North Technologies support"),
    ("tripmate",      True,  "DeLorme TripMate support"),
    ("tsip",          True,  "Trimble TSIP support"),
    # Non-GPS protocols
    ("aivdm",         True,  "AIVDM support"),
    ("gpsclock",      True,  "Furuno GPSClock support"),
    ("isync",         True,  "Spectratime iSync LNRClok/GRCLOK support"),
    # Time service
    ("oscillator",    True,  "Disciplined oscillator support"),
    # Export methods
    ("dbus_export",   True,  "enable DBUS export support"),
    ("shm_export",    True,  "export via shared memory"),
    ("socket_export", True,  "data export over sockets"),
    # Communication
    ("bluez",         True,  "BlueZ support for Bluetooth devices"),
    ('usb',           True,  "libusb support for USB devices"),
    # Other daemon options
    ("control_socket", True,  "control socket for hotplug notifications"),
    ("systemd",       systemd, "systemd socket activation"),
    # Client-side options
    ("clientdebug",   True,  "client debugging support"),
    ("libgpsmm",      True,  "build C++ bindings"),
    ("ncurses",       True,  "build with ncurses"),
    ("qt",            True,  "build Qt bindings"),
    # Daemon options
    ("squelch",       False, "squelch gpsd_log/gpsd_hexdump to save cpu"),
    # Build control
    ("coveraging",    False, "build with code coveraging enabled"),
    ("debug",         False, "add debug information to build, unoptimized"),
    ("debug_opt",     False, "add debug information to build, optimized"),
    ("gpsdclients",   True,  "gspd client programs"),
    ("gpsd",          True,  "gpsd itself"),
    ("implicit_link", imloads, "implicit linkage is supported in shared libs"),
    # FIXME: should check for Pi, not for "linux"
    ("magic_hat", sys.platform.startswith('linux'),
     "special Linux PPS hack for Raspberry Pi et al"),
    ("minimal", False, "turn off every option not set on the command line"),
    ("nostrip",       False, "don't symbol-strip binaries at link time"),
    ("profiling",     False, "build with profiling enabled"),
    ("python",        True,  "build Python support and modules."),
    ("shared",        True,  "build shared libraries, not static"),
    ("timeservice",   False, "time-service configuration"),
    ("xgps",          True,  "include xgps and xgpsspeed."),
    # Test control
    ("slow",          False, "run tests with realistic (slow) delays"),
)

# now step on the boolopts just read from '.scons-option-cache'
# Otherwise if no cache, then no boolopts.
for (name, default, helpd) in boolopts:
    opts.Add(BoolVariable(name, helpd, default))

# See PEP 394 for why 'python' is the preferred name for Python.
# override with "target_python=XX" on scons command line if want different
# Later there are tests for OS specifics.
def_target_python = "python"
def_python_shebang = "/usr/bin/env %s" % def_target_python

# Gentoo, Fedora, openSUSE systems use uucp for ttyS* and ttyUSB*
if os.path.exists("/etc/gentoo-release"):
    def_group = "uucp"
else:
    def_group = "dialout"

# darwin and BSDs do not have /run, maybe others.
if os.path.exists("/run"):
    rundir = "/run"
else:
    rundir = "/var/run"

nonboolopts = (
    ("gpsd_group",       def_group,     "privilege revocation group"),
    ("gpsd_user",        "nobody",      "privilege revocation user",),
    ("manbuild",         "auto",
     "build help in man and HTML formats.  No/Auto/Yes."),
    ("max_clients",      '64',          "maximum allowed clients"),
    ("max_devices",      '6',           "maximum allowed devices"),
    ("prefix",           "/usr/local",  "installation directory prefix"),
    ("python_coverage",  "coverage run", "coverage command for Python progs"),
    ("python_libdir",    "",            "Python module directory prefix"),
    ("python_shebang",   def_python_shebang, "Python shebang"),
    ("qt_versioned",     "",            "version for versioned Qt"),
    ("release",          "",            "Suffix for gpsd version"),
    ("rundir",           rundir,
     "Directory for run-time variable data"),
    ("sysroot",          "",
     "Logical root directory for headers and libraries.\n"
     "For cross-compiling, or building with multiple local toolchains.\n"
     "See gcc and ld man pages for more details."),
    ("target",           "",
     "Prefix to the binary tools to use (gcc, ld, etc.)\n"
     "For cross-compiling, or building with multiple local toolchains.\n"
     ),
    # If build and target platform are different, then redefining target
    # platform might be necessary to use better build flags
    ("target_platform", sys.platform,
     "target platform for cross-compiling (linux, darwin, etc.)"),
    ("target_python",  def_target_python,  "target Python version as command"),
)

# now step on the non boolopts just read from '.scons-option-cache'
# why?
for (name, default, helpd) in nonboolopts:
    opts.Add(name, helpd, default)

pathopts = (
    ("bindir",       "bin",                "application binaries directory"),
    ("docdir",       "share/gpsd/doc",     "documents directory"),
    ("icondir",      "share/gpsd/icons",   "icon directory"),
    ("includedir",   "include",            "header file directory"),
    ("libdir",       "lib",                "system libraries"),
    ("mandir",       "share/man",          "manual pages directory"),
    # /usr/share/snmp/mibs is default for net-snmp
    ("mibdir",       "share/snmp/mibs/gpsd",    "MIB directory"),
    ("pkgconfig",    "$libdir/pkgconfig",  "pkgconfig file directory"),
    ("sbindir",      "sbin",               "system binaries directory"),
    ("sharedir",     "share/gpsd",         "share directory"),
    ("sysconfdir",   "etc",                "system configuration directory"),
    ("udevdir",      "/lib/udev",          "udev rules directory"),
    ("unitdir",      systemd_dir,          "Directory for systemd unit files"),
)

# now step on the path options just read from '.scons-option-cache'
for (name, default, helpd) in pathopts:
    opts.Add(PathVariable(name, helpd, default, PathVariable.PathAccept))


#
# Environment creation
#
import_env = (
    # Variables used by programs invoked during the build
    "DISPLAY",         # Required for dia to run under scons
    "GROUPS",          # Required by gpg
    "HOME",            # Required by gpg
    "LANG",            # To avoid Gtk warnings with Python >=3.7
    'PATH',            # Required for ccache and Coverity scan-build
    'CCACHE_DIR',      # Required for ccache
    'CCACHE_RECACHE',  # Required for ccache (probably there are more)
    # pkg-config (required for crossbuilds at least, and probably pkgsrc)
    'PKG_CONFIG_LIBDIR',
    'PKG_CONFIG_PATH',
    'PKG_CONFIG_SYSROOT_DIR',
    # Variables for specific packaging/build systems
    "MACOSX_DEPLOYMENT_TARGET",  # MacOSX 10.4 (and probably earlier)
    'STAGING_DIR',               # OpenWRT and CeroWrt
    'STAGING_PREFIX',            # OpenWRT and CeroWrt
    'CWRAPPERS_CONFIG_DIR',      # pkgsrc
    # Variables used in testing
    'WRITE_PAD',       # So we can test WRITE_PAD values on the fly.
)

envs = {}
for var in import_env:
    if var in os.environ:
        envs[var] = os.environ[var]
envs["GPSD_HOME"] = os.getcwd() + os.sep + 'gpsd'

env = Environment(tools=["default", "tar", "textfile"], options=opts, ENV=envs)

# Release identification begins here.
#
# Actual releases follow the normal X.Y or X.Y.Z scheme.  The version
# number in git between releases has the form X.Y~dev, when it is
# expected that X.Y will be the next actual release.  As an example,
# when 3.20 is the last release, and 3.20.1 is the expected next
# release, the version in git will be 3.20.1~dev.  Note that ~ is used,
# because there is some precedent, ~ is an allowed version number in
# the Debian version rules, and it does not cause confusion with
# whether - separates components of the package name, separates the
# name from the version, or separates version components.

if 'dev' in gpsd_version:
    (st, gpsd_revision) = _getstatusoutput('git describe --tags')
    if st != 0:
        # If git describe failed
        # Try to use current commit hash
        (st, gpsd_commit) = _getstatusoutput('git rev-parse HEAD')
        if st == 0 and gpsd_commit:
            # Format output similar to normal revision
            gpsd_revision = '%s-g%s' % (gpsd_version, polystr(gpsd_commit[:9]))
        else:
            # Only if git describe and git rev-parse failed
            # Use timestamp from latest relevant file,
            # ignoring generated files (../$variantdir)
            # from root, not from $variantdir
            files = FileList(['../*.c', '../*/*.c', '../*.cpp', '../*/*.cpp',
                              '../include/*.h', '../*.in', '../*/*.in',
                              '../SConstruct', '../SConscript'],
                              '../%s' % variantdir)
            timestamps = map(GetMtime, files)
            if timestamps:
                from datetime import datetime
                latest = datetime.fromtimestamp(sorted(timestamps)[-1])
                gpsd_revision = '%s-%s' % (gpsd_version, latest.isoformat())
            else:
                gpsd_revision = gpsd_version  # Paranoia
else:
    gpsd_revision = gpsd_version

gpsd_revision = polystr(gpsd_revision.strip())

# Distros like to add a suffix to the version.  Fedora, and others,
# call it the "release".  It often looks like: r1
if env['release']:
    gpsd_revision += "-" + polystr(env['release'])

# SCons 2.3.0 lacks the Pseudo method.  If it's missing here, make it a
# dummy and hope for the best.
try:
    env.Pseudo
except AttributeError:
    env.Pseudo = lambda x: None

#  Minimal build turns off every option not set on the command line,
if ARGUMENTS.get('minimal'):
    for (name, default, helpd) in boolopts:
        # Ensure gpsd and gpsdclients are always enabled unless explicitly
        # turned off.
        if ((default is True and
             not ARGUMENTS.get(name) and
             name not in ("gpsd", "gpsdclients"))):
            env[name] = False

# Time-service build = stripped-down with some diagnostic tools
if ARGUMENTS.get('timeservice'):
    timerelated = ("gpsd",
                   "ipv6",
                   "magic_hat",
                   "ncurses",
                   "oscillator",
                   "socket_export",
                   )
    for (name, default, helpd) in boolopts:
        if ((default is True and
             not ARGUMENTS.get(name) and
             name not in timerelated)):
            env[name] = False

opts.Save('.scons-option-cache', env)

for (name, default, helpd) in pathopts:
    env[name] = env.subst(env[name])

env['VERSION'] = gpsd_version
env['SC_PYTHON'] = sys.executable  # Path to SCons Python

# Set defaults from environment.  Note that scons doesn't cope well
# with multi-word CPPFLAGS/LDFLAGS/SHLINKFLAGS values; you'll have to
# explicitly quote them or (better yet) use the "=" form of GNU option
# settings.
#
# Scons also uses different internal names than most other build-systems.
# So we rely on MergeFlags/ParseFlags to do the right thing for us.
#
# scons uses gcc, or clang, to link. Thus LDFLAGS does not serve its
# traditional function of providing arguments to ln. LDFLAGS set in the
# environment before running scons get moved into CCFLAGS by scons.
# LDFLAGS set while running scons get ignored.
#
# This means all uses of LDFLAG in this file ae simply dead code.  Except
# for the import from the environment passed to scons.

env['STRIP'] = "strip"
env['PKG_CONFIG'] = "pkg-config"
for i in ["AR",      # linker for static libs, usually "ar"
          "CC",
          "CXX",
          # "LD",    # scons does not use LD, usually "ld"
          "PKG_CONFIG",
          "SHLINK",  # linker for shared libs, usually "gcc" or "g++", NOT "ld"
          "STRIP",
          "TAR"]:
    if i in os.environ:
        env[i] = os.getenv(i)
for i in ["ARFLAGS",
          "CCFLAGS",
          "CFLAGS",
          "CPPFLAGS",
          "CXXFLAGS",
          "LDFLAGS",
          "LINKFLAGS",
          "SHLINKFLAGS",
          ]:
    if i in os.environ:
        # MergeFlags() puts the options where scons wants them, not
        # where you asked them to go.
        env.MergeFlags(Split(os.getenv(i)))


# Keep scan-build options in the environment
for key, value in os.environ.items():
    if key.startswith('CCC_'):
        env.Append(ENV={key: value})

# Placeholder so we can kluge together something like VPATH builds.
# ${SRCDIR} replaces occurrences for $(srcdir) in the autotools build.
# scons can get confused if this is not a full path
# WARNING: ${SRCDIR} may contain spaces!
# FIXME: could get variantdir from SRCDIR
env['SRCDIR'] = os.getcwd()

# We may need to force slow regression tests to get around race
# conditions in the pty layer, especially on a loaded machine.
if env["slow"]:
    env['REGRESSOPTS'] = "-S"
else:
    env['REGRESSOPTS'] = ""

if env.GetOption("silent"):
    env['REGRESSOPTS'] += " -Q"


def announce(msg, end=False):
    if not env.GetOption("silent"):
        print(msg)
    if end:
        # duplicate message at exit
        atexit.register(lambda: print(msg))


announce("scons version: %s" % SCons.__version__)
announce("scons is running under Python version: %s" %
         ".".join(map(str, sys.version_info)))
announce("gpsd version: %s" % polystr(gpsd_revision))

# DESTDIR environment variable means user prefix the installation root.
DESTDIR = os.environ.get('DESTDIR', '')


def installdir(idir, add_destdir=True):
    # use os.path.join to handle absolute paths properly.
    wrapped = os.path.join(env['prefix'], env[idir])
    if add_destdir:
        wrapped = os.path.normpath(DESTDIR + os.path.sep + wrapped)
    wrapped.replace("/usr/etc", "/etc")
    wrapped.replace("/usr/lib/systemd", "/lib/systemd")
    return wrapped


# Honor the specified installation prefix in link paths.
if env["sysroot"]:
    env.Prepend(LIBPATH=[env["sysroot"] + installdir('libdir',
                add_destdir=False)])

# Don't change CCFLAGS if already set by environment.
if 'CCFLAGS' in os.environ:
    announce('Warning: CCFLAGS from environment overriding scons settings')
else:
    # Should we build with profiling?
    if env['profiling']:
        env.Append(CCFLAGS=['-pg'])
    # Should we build with coveraging?
    if env['coveraging']:
        env.Append(CFLAGS=['-coverage'])
        env.Append(LINKFLAGS=['-coverage'])
    # Should we build with debug symbols?
    if env['debug'] or env['debug_opt']:
        env.Append(CCFLAGS=['-g3'])
        env.Append(LINKFLAGS=['-g3'])
    # Should we build with optimisation?
    if env['debug'] or env['coveraging']:
        env.Append(CCFLAGS=['-O0'])
    else:
        env.Append(CCFLAGS=['-O2'])

# Cross-development

devenv = (("ADDR2LINE", "addr2line"),
          ("AR", "ar"),
          ("AS", "as"),
          ("CC", "gcc"),
          ("CPP", "cpp"),
          ("CXX", "c++"),
          ("CXXFILT", "c++filt"),
          ("GCCBUG", "gccbug"),
          ("GCOV", "gcov"),
          ("GPROF", "gprof"),
          ("GXX", "g++"),
          # ("LD", "ld"),     # scons does not use LD
          ("NM", "nm"),
          ("OBJCOPY", "objcopy"),
          ("OBJDUMP", "objdump"),
          ("RANLIB", "ranlib"),
          ("READELF", "readelf"),
          ("SIZE", "size"),
          ("STRINGS", "strings"),
          ("STRIP", "strip"),
          )

if env['target']:
    for (name, toolname) in devenv:
        env[name] = env['target'] + '-' + toolname

if env['sysroot']:
    env.MergeFlags({"CFLAGS": ["--sysroot=%s" % env['sysroot']]})
    env.MergeFlags({"LINKFLAGS": ["--sysroot=%s" % env['sysroot']]})


# Build help
def cmp(a, b):
    return (a > b) - (a < b)


# FIXME: include __doc__ in help
Help("""Arguments may be a mixture of switches and targets in any order.
Switches apply to the entire build regardless of where they are in the order.
Important switches include:

    prefix=/usr     probably what packagers want

Options are cached in a file named .scons-option-cache and persist to later
invocations.  The file is editable.  Delete it to start fresh.  Current option
values can be listed with 'scons -h'.
""" + opts.GenerateHelpText(env, sort=cmp))

# Configuration


def CheckFlt_Eval_Method(context):
    """Ensure FLT_EVAL_METHOD is 0"""
    context.Message('Checking FLT_EVAL_METHOD is 0... ')
    ret = context.TryLink("""
#include <float.h>

#ifndef FLT_EVAL_METHOD
    error
#endif
#if 0 != FLT_EVAL_METHOD
  error
#endif
int main(int argc, char **argv) {
    (void) argc; (void) argv;
    return 0;
}
    """, '.c')
    context.Result(ret)
    return ret


def CheckPKG(context, name):
    context.Message('Checking pkg-config for %s... ' % name)
    ret = context.TryAction('%s --exists \'%s\''
                            % (context.env['PKG_CONFIG'], name))[0]
    context.Result(ret)
    return ret


def CheckStrerror_r(context):
    """Return strerror_r(24,...).
Will return true if POSIX, false if gnu-like
Required because libc's are random about it.
"""
    context.Message('Checking if strerror_r() returns int... ')
    old_CFLAGS = context.env['CFLAGS'][:]  # Get a *copy* of the old list
    # Make the cast warning an error
    context.env.Append(CFLAGS="-Werror")
    ret = context.TryCompile("""
        #define _GNU_SOURCE

        #include <stddef.h>
        #include <string.h>

        int main(void) {
            char buf[100];
            int ret;

            ret = strerror_r(24, buf, sizeof(buf));
            return ret;
        }
    """, '.c')
    context.Result(ret)
    context.env.Replace(CFLAGS=old_CFLAGS)  # restore flags
    return ret


def CheckCompilerOption(context, option):
    context.Message('Checking if compiler accepts %s... ' % (option,))
    old_CFLAGS = context.env['CFLAGS'][:]  # Get a *copy* of the old list
    context.env.Append(CFLAGS=option)
    new_CFLAGS = context.env['CFLAGS'][:]  # Get a *copy* of the old list
    # we don't want to use options that gernerate warnings.
    context.env.Append(CFLAGS="-Werror")
    ret = context.TryLink("""
        int main(int argc, char **argv) {
            (void) argc; (void) argv;
            return 0;
        }
    """, '.c')
    if ret:
        # worked, remove the -Werror
        context.env.Replace(CFLAGS=new_CFLAGS)
    else:
        context.env.Replace(CFLAGS=old_CFLAGS)

    context.Result(ret)
    return ret


# Check if this compiler is C11 or better
def CheckC11(context):
    context.Message('Checking if compiler is C11... ')
    ret = context.TryLink("""
        #if (__STDC_VERSION__ < 201112L)
        #error Not C11
        #endif
        int main(int argc, char **argv) {
            (void) argc; (void) argv;
            return 0;
        }
    """, '.c')
    context.Result(ret)
    return ret


def GetPythonValue(context, name, imp, expr, brief=False):
    """Get a value from the target python, not the running one."""
    context.Message('Checking Python %s... ' % name)

    if context.env['target_python']:
        command = (context.env['target_python'] + " $SOURCE > $TARGET")
        text = "%s; print(%s)" % (imp, expr)

        # TryAction returns (1, outputStr), or (0, '') on fail
        (status, value) = context.TryAction(command, text, '.py')

        # do not disable python because this failed
        # maybe testing for newer python feature
    else:
        # FIXME: this ignores imp
        status = 1
        value = str(eval(expr))

    if 1 == status:
        # we could convert to str(), but caching turns it into bytes anyway
        value = value.strip()
        if brief is True:
            context.did_show_result = 1
            print("ok")

    context.Result(value)
    # return value
    return value


def GetLoadPath(context):
    context.Message("Getting system load path... ")


cleaning = env.GetOption('clean')
helping = env.GetOption('help')

# Always set up LIBPATH so that cleaning works properly.
# FIXME: use ${SRCDIR}?
env.Prepend(LIBPATH=[os.path.realpath(os.curdir)])

# from scons 3.0.5, any changes to env after this, until after
# config.Finish(), will be lost.  Use config.env until then.

config = Configure(env, custom_tests={
    'CheckC11': CheckC11,
    'CheckCompilerOption': CheckCompilerOption,
    'CheckFlt_Eval_Method': CheckFlt_Eval_Method,
    'CheckPKG': CheckPKG,
    'CheckStrerror_r': CheckStrerror_r,
    'GetPythonValue': GetPythonValue,
    })

# Use print, rather than announce, so we see it in -s mode.
print("This system is: %s" % sys.platform)

libgps_flags = []
rtlibs = []
bluezflags = []
capflags = []
confdefs = []
dbusflags = []
adoc_prog = False
ncurseslibs = []
mathlibs = []
xtlibs = []
tiocmiwait = True  # For cleaning, which works on any OS
usbflags = []
have_dia = False
# canplayer is part of can-utils, required for NMEA 2000 tests
have_canplayer = False
have_coverage = False
have_cppcheck = False
have_flake8 = False
have_pycodestyle = False
have_pylint = False
have_scan_build = False
have_smilint = False
have_tar = False
have_valgrind = False

# skip config part if cleaning or helping.
# per SCons 4.0.1 doc: Section 23.9. Not Configuring When Cleaning Targets
if not cleaning and not helping:
    # OS X aliases gcc to clang
    if (sys.platform != config.env['target_platform']):
        announce("Target system is: %s" % config.env['target_platform'])

    if 'CCVERSION' in env:
        announce("cc is %s, version %s" % (env['CC'], env['CCVERSION']))
    else:
        # sometimes scons can not determine clang version
        announce("cc is %s, WARNING version is unknown" % env['CC'])

    # clang accepts -pthread, then warns it is unused.
    if not config.CheckCC():
        announce("ERROR: CC doesn't work")

    if ((config.CheckCompilerOption("-pthread") and
         not config.env['target_platform'].startswith('darwin'))):
        config.env.MergeFlags("-pthread")

    if config.env['target_platform'].startswith('openbsd7'):
        # as of 5 Jan 23:
        # scons 4.4.0 with clang 13.0.0 has trouble determining clang version.
        # Then fails to add -fPIC.  So we force it here:
        config.env.Append(CCFLAGS=['-fPIC'])

    confdefs = ["/* gpsd_config.h generated by scons, do not hand-hack. */\n"]

    confdefs.append('#ifndef GPSD_CONFIG_H\n')

    confdefs.append('#define VERSION "%s"' % gpsd_version)
    confdefs.append('#define REVISION "%s"' % gpsd_revision)
    confdefs.append('#define GPSD_PROTO_VERSION_MAJOR %u' % api_version_major)
    confdefs.append('#define GPSD_PROTO_VERSION_MINOR %u' % api_version_minor)

    confdefs.append('#define GPSD_URL "%s"\n' % website)

    # TODO: Move these into an if block only on systems with glibc.
    # needed for isfinite(), pselect(), etc.
    # for strnlen() before glibc 2.10
    # glibc 2.10+ needs 200908L (or XOPEN 700+) for strnlen()
    # on newer glibc _DEFAULT_SOURCE resets _POSIX_C_SOURCE
    # we set it just in case
    confdefs.append('#if !defined(_POSIX_C_SOURCE)')
    confdefs.append('#define _POSIX_C_SOURCE 200809L')
    confdefs.append('#endif\n')
    # for daemon(), cfmakeraw(), strsep() and setgroups()
    # on glibc 2.19+
    # may also be added by pkg_config
    # on linux this eventually sets _USE_XOPEN
    confdefs.append('#if !defined(_DEFAULT_SOURCE)')
    confdefs.append('#define _DEFAULT_SOURCE')
    confdefs.append('#endif\n')

    # sys/un.h, and more, needs __USE_MISC with glibc and osX
    # __USE_MISC is set by _DEFAULT_SOURCE or _BSD_SOURCE

    # TODO: Many of these are now specified by POSIX.  Check if
    # defining _XOPEN_SOURCE is necessary, and limit to systems where
    # it is.
    # 500 means X/Open 1995
    # getsid(), isascii(), nice(), putenv(), strdup(), sys/ipc.h need 500
    # 600 means X/Open 2004
    # Ubuntu and OpenBSD isfinite() needs 600
    # 700 means X/Open 2008
    # glibc 2.10+ needs 700+ for strnlen()
    # Python.h wants 600 or 700

    # removed 2 Jul 2019 to see if anything breaks...
    # confdefs.append('#if !defined(_XOPEN_SOURCE)')
    # confdefs.append('#define _XOPEN_SOURCE 700')
    # confdefs.append('#endif\n')
    # Reinstated for FreeBSD (below) 16-Aug-2019

    if config.env['target_platform'].startswith('linux'):
        # for cfmakeraw(), strsep(), etc. on CentOS 7
        # glibc 2.19 and before
        # sets __USE_MISC
        confdefs.append('#if !defined(_BSD_SOURCE)')
        confdefs.append('#define _BSD_SOURCE')
        confdefs.append('#endif\n')
        # for strnlen() and struct ifreq
        # glibc before 2.10, deprecated in 2.10+
        confdefs.append('#if !defined(_GNU_SOURCE)')
        confdefs.append('#define _GNU_SOURCE 1')
        confdefs.append('#endif\n')
    elif config.env['target_platform'].startswith('darwin'):
        # strlcpy() and SIGWINCH need _DARWIN_C_SOURCE
        confdefs.append('#if !defined(_DARWIN_C_SOURCE)')
        confdefs.append('#define _DARWIN_C_SOURCE 1\n')
        confdefs.append('#endif\n')
        # vsnprintf() needs __DARWIN_C_LEVEL >= 200112L
        # snprintf() needs __DARWIN_C_LEVEL >= 200112L
        # _DARWIN_C_SOURCE forces __DARWIN_C_LEVEL to 900000L
        # see <sys/cdefs.h>

        # set internal lib versions at link time.
        libgps_flags = ["-Wl,-current_version,%s" % libgps_version,
                        "-Wl,-compatibility_version,%s" % libgps_version,
                        "-Wl,-install_name,%s/$TARGET.srcpath" %
                        installdir('libdir', add_destdir=False)]
    elif config.env['target_platform'].startswith('freebsd'):
        # for isascii(), putenv(), nice(), strptime()
        confdefs.append('#if !defined(_XOPEN_SOURCE)')
        confdefs.append('#define _XOPEN_SOURCE 700')
        confdefs.append('#endif\n')
        # required to define u_int in sys/time.h
        confdefs.append('#if !defined(_BSD_SOURCE)')
        confdefs.append("#define _BSD_SOURCE 1\n")
        confdefs.append('#endif\n')
        # required to get strlcpy(), and more, from string.h
        confdefs.append('#if !defined(__BSD_VISIBLE)')
        confdefs.append("#define __BSD_VISIBLE 1\n")
        confdefs.append('#endif\n')
    elif config.env['target_platform'].startswith('openbsd'):
        # required to define u_int in sys/time.h
        confdefs.append('#if !defined(_BSD_SOURCE)')
        confdefs.append("#define _BSD_SOURCE 1\n")
        confdefs.append('#endif\n')
        # required to get strlcpy(), and more, from string.h
        confdefs.append('#if !defined(__BSD_VISIBLE)')
        confdefs.append("#define __BSD_VISIBLE 1\n")
        confdefs.append('#endif\n')
    elif config.env['target_platform'].startswith('netbsd'):
        # required to get strlcpy(), and more, from string.h
        confdefs.append('#if !defined(_NETBSD_SOURCE)')
        confdefs.append("#define _NETBSD_SOURCE 1\n")
        confdefs.append('#endif\n')
    elif config.env['target_platform'].startswith('sunos5'):
        # tested with gcc-5.5 on slowlaris 10
        # required to get isascii(), and more, from ctype.h
        confdefs.append('#if !defined(__XPG4_CHAR_CLASS__)')
        confdefs.append("#define __XPG4_CHAR_CLASS__ 1\n")
        confdefs.append('#endif\n')
        confdefs.append('#if !defined(__XPG6)')
        confdefs.append('#define _XPG6\n')
        confdefs.append('#endif\n')
        # for things like strlcat(), strlcpy)
        confdefs.append('#if !defined(__EXTENSIONS__)')
        confdefs.append('#define __EXTENSIONS__\n')
        confdefs.append('#endif\n')

    cxx = config.CheckCXX()
    if not cxx:
        announce("C++ doesn't work, suppressing libgpsmm and Qt build.")
        config.env["libgpsmm"] = False
        config.env["qt"] = False

    # define a helper function for pkg-config - we need to pass
    # --static for static linking, too.
    #
    # Using "--libs-only-L --libs-only-l" instead of "--libs" avoids
    # a superfluous "-rpath" option in some FreeBSD cases, and the resulting
    # scons crash.
    # However, it produces incorrect results for Qt5Network in OSX, so
    # it can't be used unconditionally.
    def pkg_config(pkg, shared=env['shared'], rpath_hack=False):
        libs = '--libs-only-L --libs-only-l' if rpath_hack else '--libs'
        if not shared:
            libs += ' --static'
        return ['!%s --cflags %s %s' % (env['PKG_CONFIG'], libs, pkg)]

    if config.CheckPKG('libcap'):
        confdefs.append("#define HAVE_LIBCAP 1\n")
        try:
            capflags = pkg_config('libcap')
        except OSError:
            announce("pkg_config is confused about the state "
                     "of libcap.")
            capflags = []

    # The actual distinction here is whether the platform has ncurses in the
    # base system or not. If it does, pkg-config is not likely to tell us
    # anything useful. FreeBSD does, Linux doesn't. Most likely other BSDs
    # are like FreeBSD.
    ncurseslibs = []
    if config.env['ncurses']:
        if not config.CheckHeader(["curses.h"]):
            announce('Turning off ncurses support, curses.h not found.')
            config.env['ncurses'] = False
        elif config.CheckPKG('ncurses'):
            ncurseslibs = pkg_config('ncurses', rpath_hack=True)
            if config.CheckPKG('tinfo'):
                ncurseslibs += pkg_config('tinfo', rpath_hack=True)
        elif config.CheckPKG('ncursesw'):
            # One distro in 2022, Void, only ships the ncursesw
            # part of ncurses.
            ncurseslibs = pkg_config('ncursesw', rpath_hack=True)
            if config.CheckPKG('tinfo'):
                ncurseslibs += pkg_config('tinfo', rpath_hack=True)
        # It's not yet known whether rpath_hack is appropriate for
        # ncurses5-config.
        elif WhereIs('ncurses5-config'):
            ncurseslibs = ['!ncurses5-config --libs --cflags']
        elif WhereIs('ncursesw5-config'):
            ncurseslibs = ['!ncursesw5-config --libs --cflags']
        elif config.env['target_platform'].startswith('freebsd'):
            ncurseslibs = ['-lncurses']
        elif (config.env['target_platform'].startswith('darwin') or
              config.env['target_platform'].startswith('openbsd') or
              config.env['target_platform'].startswith('sunos5')):
            ncurseslibs = ['-lcurses']
        else:
            announce('Turning off ncurses support, library not found.')
            config.env['ncurses'] = False

    if config.env['usb']:
        # In FreeBSD except version 7, USB libraries are in the base system
        if config.CheckPKG('libusb-1.0'):
            confdefs.append("#define HAVE_LIBUSB 1\n")
            try:
                usbflags = pkg_config('libusb-1.0')
            except OSError:
                announce("pkg_config is confused about the state "
                         "of libusb-1.0.")
                usbflags = []
        elif config.env['target_platform'].startswith('freebsd'):
            # FIXME: shold directly test for libusb existence.
            confdefs.append("#define HAVE_LIBUSB 1\n")
            usbflags = ["-lusb"]
        else:
            confdefs.append("/* #undef HAVE_LIBUSB */\n")
            usbflags = []
    else:
        confdefs.append("/* #undef HAVE_LIBUSB */\n")
        usbflags = []
        config.env["usb"] = False

    if config.CheckLib('librt'):
        confdefs.append("#define HAVE_LIBRT 1\n")
        # System library - no special flags
        rtlibs = ["-lrt"]
    else:
        confdefs.append("/* #undef HAVE_LIBRT */\n")

    # for slowlaris socket(), bind(), etc.
    if config.CheckLib('libnsl'):
        confdefs.append("#define HAVE_LIBNSL\n")
        # System library - no special flags
        rtlibs += ["-lnsl"]
    else:
        confdefs.append("/* #undef HAVE_LIBNSL */\n")

    # for slowlaris socket(), bind(), etc.
    if config.CheckLib('libsocket'):
        confdefs.append("#define HAVE_LIBSOCKET\n")
        # System library - no special flags
        rtlibs += ["-lsocket"]
    else:
        confdefs.append("/* #undef HAVE_LIBNSOCKET */\n")

    # The main reason we check for libm explicitly is to set up the config
    # environment for CheckFunc for sincos().  But it doesn't hurt to omit
    # the '-lm' when it isn't appropriate.
    if config.CheckLib('libm'):
        mathlibs = ['-lm']
    else:
        mathlibs = []

    # FreeBSD uses -lthr for pthreads
    if config.CheckLib('libthr'):
        confdefs.append("#define HAVE_LIBTHR 1\n")
        # System library - no special flags
        rtlibs += ["-lthr"]
    else:
        confdefs.append("/* #undef HAVE_LIBTHR */\n")

    if config.env['dbus_export'] and config.CheckPKG('dbus-1'):
        confdefs.append("#define HAVE_DBUS 1\n")
        dbusflags = pkg_config("dbus-1")
        config.env.MergeFlags(dbusflags)
    else:
        confdefs.append("/* #undef HAVE_DBUS */\n")
        dbusflags = []
        if config.env["dbus_export"]:
            announce("Turning off dbus-export support, library not found.")
        config.env["dbus_export"] = False

    if config.env['bluez'] and config.CheckPKG('bluez'):
        confdefs.append("#define ENABLE_BLUEZ 1\n")
        bluezflags = pkg_config('bluez')
    else:
        confdefs.append("/* #undef ENABLE_BLUEZ */\n")
        bluezflags = []
        if config.env["bluez"]:
            announce("Turning off Bluetooth support, library not found.")
        config.env["bluez"] = False

    # in_port_t is not defined on Android
    if not config.CheckType("in_port_t", "#include <netinet/in.h>"):
        announce("Did not find in_port_t typedef, assuming unsigned short int")
        confdefs.append("typedef unsigned short int in_port_t;\n")

    # SUN_LEN is not defined on Android
    if ((not config.CheckDeclaration("SUN_LEN", "#include <sys/un.h>") and
         not config.CheckDeclaration("SUN_LEN", "#include <linux/un.h>"))):
        announce("SUN_LEN is not system-defined, using local definition")
        confdefs.append("#ifndef SUN_LEN\n")
        confdefs.append("#define SUN_LEN(ptr) "
                        "((size_t) (((struct sockaddr_un *) 0)->sun_path) "
                        "+ strlen((ptr)->sun_path))\n")
        confdefs.append("#endif /* SUN_LEN */\n")

    if config.CheckHeader(["linux/can.h"]):
        confdefs.append("#define HAVE_LINUX_CAN_H 1\n")
        announce("You have kernel CANbus available.")
    else:
        confdefs.append("/* #undef HAVE_LINUX_CAN_H */\n")
        announce("You do not have kernel CANbus available.")
        config.env["nmea2000"] = False

    # check for C11 or better, and __STDC__NO_ATOMICS__ is not defined
    # before looking for stdatomic.h
    if ((config.CheckC11() and
         not config.CheckDeclaration("__STDC_NO_ATOMICS__") and
         config.CheckHeader("stdatomic.h"))):
        confdefs.append("#define HAVE_STDATOMIC_H 1\n")
    else:
        confdefs.append("/* #undef HAVE_STDATOMIC_H */\n")
        if config.CheckHeader("libkern/OSAtomic.h"):
            confdefs.append("#define HAVE_OSATOMIC_H 1\n")
        else:
            confdefs.append("/* #undef HAVE_OSATOMIC_H */\n")
            announce("No memory barriers - SHM export and time hinting "
                     "may not be reliable.")

    # endian.h is required for rtcm104v2 unless the compiler defines
    # __ORDER_BIG_ENDIAN__, __ORDER_LITTLE_ENDIAN__ and __BYTE_ORDER__
    if ((config.CheckDeclaration("__ORDER_BIG_ENDIAN__") and
         config.CheckDeclaration("__ORDER_LITTLE_ENDIAN__") and
         config.CheckDeclaration("__BYTE_ORDER__"))):
        confdefs.append("#define HAVE_BUILTIN_ENDIANNESS 1\n")
        confdefs.append("/* #undef HAVE_ENDIAN_H */\n")
        confdefs.append("/* #undef HAVE_SYS_ENDIAN_H */\n")
        announce("Your compiler has built-in endianness support.")
    else:
        confdefs.append("/* #undef HAVE_BUILTIN_ENDIANNESS\n */")
        if config.CheckHeader("endian.h"):
            confdefs.append("#define HAVE_ENDIAN_H 1\n")
            confdefs.append("/* #undef HAVE_SYS_ENDIAN_H */\n")
            confdefs.append("/* #undef HAVE_MACHINE_ENDIAN_H */\n")
        elif config.CheckHeader("sys/endian.h"):
            confdefs.append("/* #undef HAVE_ENDIAN_H */\n")
            confdefs.append("#define HAVE_SYS_ENDIAN_H 1\n")
            confdefs.append("/* #undef HAVE_MACHINE_ENDIAN_H */\n")
        elif config.CheckHeader("machine/endian.h"):
            confdefs.append("/* #undef HAVE_ENDIAN_H */\n")
            confdefs.append("/* #undef HAVE_SYS_ENDIAN_H */\n")
            confdefs.append("#define HAVE_MACHINE_ENDIAN_H 1\n")
        else:
            confdefs.append("/* #undef HAVE_ENDIAN_H */\n")
            confdefs.append("/* #undef HAVE_SYS_ENDIAN_H */\n")
            confdefs.append("/* #undef HAVE_MACHINE_ENDIAN_H */\n")
            announce("You do not have the endian.h header file. "
                     "RTCM V2 support will fail.")

    for hdr in ("arpa/inet",
                "linux/serial",    # for serial_icounter_struct
                "netdb",
                "netinet/in",
                "netinet/ip",
                "sys/sysmacros",   # for major(), on linux
                "sys/socket",
                "sys/un",
                "syslog",
                "termios",
                "winsock2"
                ):
        if config.CheckHeader(hdr + ".h"):
            confdefs.append("#define HAVE_%s_H 1\n"
                            % hdr.replace("/", "_").upper())
        elif "termios" == hdr:
            announce("ERROR: %s.h not found" % hdr)
        else:
            confdefs.append("/* #undef HAVE_%s_H */\n"
                            % hdr.replace("/", "_").upper())


    if not config.CheckFlt_Eval_Method():
        announce("WARNING: FLT_EVAL_METHOD is not 0")


    if config.CheckStrerror_r():
        # POSIX behavior
        confdefs.append("#define STRERROR_R_INT\n")
    else:
        # glibc behavior
        confdefs.append("#define STRERROR_R_STR\n")

    # check for 64 bit time_t.  Needed for 2038.
    sizeof_time_t = config.CheckTypeSize("time_t", "#include <time.h>",
                                         expect=8)
    if 0 == sizeof_time_t:
        # see if we can force time64_t
        # this needs glibc 2.34 or later, and a compatible kernel
        sizeof_time_t = config.CheckTypeSize("time_t",
                                             "#define _TIME_BITS 64\n"
                                             "#define _FILE_OFFSET_BITS 64\n"
                                             "#include <time.h>",
                                             expect=8)
        if 0 != sizeof_time_t:
            # force time64_t
            confdefs.append("// Forcing 64-bit time_t\n"
                            "#define _TIME_BITS 64\n"
                            "#define _FILE_OFFSET_BITS 64\n")

    if 0 == sizeof_time_t:
        announce("WARNING: time_t is too small.  It will fail in 2038")
        sizeof_time_t = 4
    else:
        sizeof_time_t = 8

    confdefs.append("#define SIZEOF_TIME_T %s\n" % sizeof_time_t)

    # check function after libraries, because some function require libraries
    # for example clock_gettime() require librt on Linux glibc < 2.17
    # These are  are POSIX 2001, so no need to check for them.
    #  "fork", gmtime_r", "inet_ntop", strptime"
    # These are  are POSIX 2008, so no need to check for them.
    #  "strnlen"
    for f in ("cfmakeraw", "clock_gettime", "daemon", "fcntl", "getopt_long",
              "strlcat", "strlcpy"):
        if config.CheckFunc(f):
            confdefs.append("#define HAVE_%s 1\n" % f.upper())
        else:
            confdefs.append("/* #undef HAVE_%s */\n" % f.upper())

    # used to check for sincos(), but making that work with -Werror did not work.

    if config.CheckHeader(["sys/types.h", "sys/time.h", "sys/timepps.h"]):
        confdefs.append("#define HAVE_SYS_TIMEPPS_H 1\n")
        kpps = True
    else:
        kpps = False
        if config.env["magic_hat"]:
            announce("Forcing magic_hat=no since RFC2783 API is unavailable")
            config.env["magic_hat"] = False
    tiocmiwait = config.CheckDeclaration("TIOCMIWAIT",
                                         "#include <sys/ioctl.h>")
    if not tiocmiwait and not kpps:
        announce("WARNING: Neither TIOCMIWAIT (PPS) nor RFC2783 API (KPPS) "
                 "is available.", end=True)
        if config.env["timeservice"]:
            announce("ERROR: timeservice specified, but no PPS available")
            Exit(1)

    # Map options to libraries required to support them that might be absent.
    optionrequires = {
        "bluez": ["libbluetooth"],
        "dbus_export": ["libdbus-1"],
    }

    keys = list(map(lambda x: (x[0], x[2]), boolopts))  \
        + list(map(lambda x: (x[0], x[2]), nonboolopts)) \
        + list(map(lambda x: (x[0], x[2]), pathopts))
    keys.sort()
    for (key, helpd) in keys:
        value = config.env[key]
        if value and key in optionrequires:
            for required in optionrequires[key]:
                if not config.CheckLib(required):
                    announce("%s not found, %s cannot be enabled."
                             % (required, key))
                    value = False
                    break

        confdefs.append("/* %s */" % helpd)
        if isinstance(value, bool):
            if value:
                confdefs.append("#define %s_ENABLE 1\n" % key.upper())
            else:
                confdefs.append("/* #undef %s_ENABLE */\n" % key.upper())
        elif value in (0, "", "(undefined)"):
            confdefs.append("/* #undef %s */\n" % key.upper())
        else:
            if value.isdigit():
                confdefs.append("#define %s %s\n" % (key.upper(), value))
            else:
                confdefs.append("#define %s \"%s\"\n" % (key.upper(), value))

    # Simplifies life on hackerboards like the Raspberry Pi
    if config.env['magic_hat']:
        confdefs.append('''\
/* Magic device which, if present, means to grab a static /dev/pps0 for KPPS */
#define MAGIC_HAT_GPS   "/dev/ttyAMA0"
/* Generic device which, if present, means: */
/* to grab a static /dev/pps0 for KPPS */
#define MAGIC_LINK_GPS  "/dev/gpsd0"
''')

    confdefs.append('''\

#define GPSD_CONFIG_H
#endif /* GPSD_CONFIG_H */
''')

    # handle manbuild = no/auto/yes
    # do we have asciidoctor, perhaps versioned?
    adoc_prog = env.WhereIs('asciidoctor')
    if (not adoc_prog):
        adoc_prog = env.WhereIs('asciidoctor31')
    if (not adoc_prog):
        adoc_prog = env.WhereIs('asciidoctor30')
    if (not adoc_prog):
        adoc_prog = env.WhereIs('asciidoctor27')

    config.env['manbuild'] = config.env['manbuild'].lower()
    if ((not config.env['manbuild'] or
         'auto' == config.env['manbuild'])):
        if adoc_prog:
            config.env['manbuild'] = 1
            announce("Build of man and HTML documentation enabled.")
        else:
            config.env['manbuild'] = 0
            announce("WARNING: AsciiDoctor not found.\n"
                     "WARNING: Some documentation and html will not be built.",
                     end=True)
    else:
        try:
            config.env['manbuild'] = strtobool(
                config.env['manbuild'])
        except ValueError:
            announce("ERROR: manbuild must be no/auto/yes.")
            sys.exit(1)

        if 0 == config.env['manbuild']:
            adoc_prog = None
            announce("Build of man and HTML documentation disabled.")
        elif 1 == config.env['manbuild'] and not adoc_prog:
            announce("ERROR: manbuild=True, but AsciiDoctor not found.\n")
            sys.exit(1)
        else:
            announce("Build of man and HTML documentation enabled.")
    # end  handle manbuild = no/auto/yes

    # Determine if Qt network libraries are present, and
    # if not, force qt to off
    if config.env["qt"]:
        qt_net_name = 'Qt%sNetwork' % config.env["qt_versioned"]
        qt_network = config.CheckPKG(qt_net_name)
        if not qt_network:
            config.env["qt"] = False
            announce('Turning off Qt support, library not found.')

    # If supported by the compiler, enable all warnings except uninitialized
    # and missing-field-initializers, which we can't help triggering because
    # of the way some of the JSON-parsing code is generated.
    #
    # Some flags work for cc, but not c++, add those here, and to c_oply
    # below
    #
    # Do this after the other config checks, to keep warnings out of them.
    for option in (
        # -Wall and Wextra first as they modify later options
        '-Wall',
        '-Wextra',
        # clang: ask for C Annex F standard floating point
        '--disable-excess-fp-precision',
        # gcc: ask for C Annex F standard floating point
        '-fexcess-precision=standard',

        '-Wcast-align',
        '-Wcast-qual',
        # -Wimplicit-fallthrough same as
        # -Wimplicit-fallthrough=3, except osX hates the
        # second flavor
        '-Wimplicit-fallthrough',
        # '-Wimplicit-function-declaration',     # someday, annoys C++
        '-Wmissing-declarations',
        '-Wmissing-prototypes',
        '-Wno-missing-field-initializers',
        '-Wno-uninitialized',
        '-Wpointer-arith',
        '-Wreturn-type',
        '-Wstrict-prototypes',
        '-Wundef',
        '-Wvla',
        ):
        if option not in config.env['CFLAGS']:
            config.CheckCompilerOption(option)

    # check for misc audit programs
    try:
        have_canplayer = config.CheckProg('canplayer')
        have_coverage = config.CheckProg('coverage')
        have_cppcheck = config.CheckProg('cppcheck')
        have_dia = config.CheckProg('dia')
        have_flake8 = config.CheckProg('flake8')
        have_pycodestyle = config.CheckProg('pycodestyle')
        have_pylint = config.CheckProg('pylint')
        have_scan_build = config.CheckProg('scan-build')
        # smilint is part of libsmi package
        have_smilint = config.CheckProg('smilint')
        have_tar = config.CheckProg(env['TAR'])
        have_valgrind = config.CheckProg('valgrind')
    except AttributeError:
        # scons versions before Sep 2015 (2.4.0) don't have CheckProg
        # gpsd only asks for 2.3.0 or higher
        announce("scons CheckProg() failed..")

    if not have_canplayer:
        announce("Program canplayer not found -- skipping NMEA 2000 tests")
    if not have_coverage:
        announce("Program coverage not found -- skipping Python coverage")
    if not have_cppcheck:
        announce("Program cppcheck not found -- skipping cppcheck checks")
    if not have_dia:
        announce("Program dia not found -- not rebuiding cycle.svg.")
    if not have_flake8:
        announce("Program flake8 not found -- skipping flake8 checks")
    if not have_pycodestyle:
        announce("Program pycodestyle not found -- "
                 "skipping pycodestyle checks")
    if not have_pylint:
        announce("Program pylint not found -- skipping pylint checks")
    if not have_scan_build:
        announce("Program scan-build not found -- skipping scan-build checks")
    if not have_smilint:
        announce("Program smilint not found -- skipping MIB checks")
    if not have_tar:
        announce('WARNING: %s not found.  Can not build tar files.' %
                 env['TAR'])
    if not have_valgrind:
        announce("Program valgrind not found -- skipping valgrind checks")


# Set up configuration for target Python

PYTHON_LIBDIR_CALL = "sysconfig.get_path('purelib')"

target_python_path = ''
python_libdir = str(eval(PYTHON_LIBDIR_CALL))

# flag if we have xgps* dependencies, so xgps* should run OK
config.env['xgps_deps'] = False

if not cleaning and not helping and config.env['python']:
    if config.env['target_python']:
        try:
            config.CheckProg
        except AttributeError:
            # scons versions before Nov 2015 (2.4.1) don't have CheckProg
            # gpsd only asks for 2.3.0 or higher
            target_python_path = config.env['target_python']
        else:
            target_python_path = config.CheckProg(config.env['target_python'])

        if ((not target_python_path and
             'python' == config.env['target_python'])):
            # some distros don't install a python target, only python3
            announce("Target Python '%s' doesn't exist.  "
                     "Trying 'python3'." %
                     config.env['target_python'])
            config.env['target_python'] = 'python3'
            python_shebang = "/usr/bin/env %s" % def_target_python
            try:
                config.CheckProg
            except AttributeError:
                # FIXME: duplicates code above
                # scons versions before Nov 2015 (2.4.1) don't
                # have CheckProg # gpsd only asks for 2.3.0 or higher
                target_python_path = config.env['target_python']
            else:
                target_python_path = config.CheckProg(
                    config.env['target_python'])

        if not target_python_path:
            announce("Target Python '%s' doesn't exist.  Disabling Python." %
                     config.env['target_python'])
            announce("Use the target_python=XX configuration option if you "
                     "have a working python target.")
            config.env['python'] = False


    if config.env['python']:
        if not target_python_path:
            # Avoid double testing for target_python_path
            # Maximize consistency by using the reported sys.executable
            target_python_path = config.GetPythonValue('exe path',
                                                       'import sys',
                                                       'sys.executable')
        target_python_path = polystr(target_python_path)
        # python module directory
        if config.env['python_libdir']:
            python_libdir = config.env['python_libdir']
        else:
            python_libdir = config.GetPythonValue('lib dir',
                                                  PYTHON_SYSCONFIG_IMPORT,
                                                  PYTHON_LIBDIR_CALL)
            # follow FHS, put in /usr/local/libXX, not /usr/libXX
            # may be lib, lib32 or lib64
            python_libdir = polystr(python_libdir)
            python_libdir = python_libdir.replace("/usr/lib",
                                                  "/usr/local/lib")
        python_module_dir = str(python_libdir) + os.sep
        # Many systems can have a problem with the Python path
        if 'PYTHONPATH' in os.environ:
            announce("System PYTHONPATH='%s'" % os.environ['PYTHONPATH'])
        else:
            announce("System PYTHONPATH is empty")
        announce("Ensure your PYTHONPATH includes %s" % python_module_dir,
                 end=True)
        python_module_dir += 'gps'

        # aiogps is only available on Python >= 3.6
        sysver = config.GetPythonValue('target version',
                                       'import sys',
                                       '"%d.%d" % sys.version_info[0:2]')

        if tuple(map(int, sysver.split("."))) < (3, 6):
            config.env['aiogps'] = False
            announce("WARNING: Python%s too old (need 3.6): "
                     "gps/aiogps.py will not be installed" %
                     (sysver), end=True)
        else:
            config.env['aiogps'] = True

        # check for pyserial
        if not config.GetPythonValue('module serial (pyserial)',
                                     'import serial', '"found"'):
            # no pyserial, used by ubxtool and zerk
            announce("WARNING: ubxtool and zerk are missing optional "
                     "runtime module serial", end=True)

        config.env['xgps_deps'] = True

        # check for pycairo
        if not config.GetPythonValue('module cairo (pycairo)',
                                     'import cairo', '"found"'):
            # no pycairo, used by xgps, xgpsspeed
            config.env['xgps_deps'] = False
            announce("WARNING: Python module cairo (pycairo) not found.")

        # check for pygobject
        if not config.GetPythonValue('module gi (pygobject)',
                                     'import gi', '"found"'):
            # no pygobject, used by xgps, xgpsspeed
            config.env['xgps_deps'] = False
            announce("WARNING: Python module gi (pygobject) not found.")

        # gtk+ needed by pygobject
        if not config.CheckPKG('gtk+-3.0'):
            config.env['xgps_deps'] = False
            announce("WARNING: gtk+-3.0 not found.")

        if not config.env['xgps_deps']:
            announce("WARNING: xgps and xgpsspeed are missing runtime "
                     "dependencies", end=True)

        if not env['xgps']:
            # xgps* turned off by option
            config.env['xgps_deps'] = False

        # check for matplotlib
        if not config.GetPythonValue('module matplotlib',
                                     'import matplotlib', '"found"'):
            # no matplotlib, used by gpsplot
            announce("WARNING: gpsplot is missing required "
                     "runtime module matplotlib", end=True)

        config.env['PYTHON'] = target_python_path
        # For regress-driver
        config.env['ENV']['PYTHON'] = target_python_path


# get a list of the files from git, so they can go in distribution zip/tar
distfiles = config.TryAction("git ls-files > $TARGET")[1]
distfiles = polystr(distfiles).split()
# add in the built man pages, zip and tar files must contain man pages.
distfiles += all_manpages.keys()

env = config.Finish()
# All configuration should be finished.  env can now be modified.
# NO CONFIG TESTS AFTER THIS POINT!

qt_env = None
if not (cleaning or helping):
    # Be explicit about what we're doing.
    changelatch = False
    for (name, default, helpd) in boolopts + nonboolopts + pathopts:
        if env[name] != env.subst(default):
            if not changelatch:
                announce("Altered configuration variables:")
                changelatch = True
            announce("%s = %s (default %s): %s"
                     % (name, env[name], env.subst(default), helpd))
    if not changelatch:
        announce("All configuration flags are defaulted.")

    # Should we build the Qt binding?
    if env["qt"] and env["shared"]:
        qt_env = env.Clone()
        qt_env.MergeFlags('-DUSE_QT')
        qt_env.Append(OBJPREFIX='qt-')
        try:
            qt_env.MergeFlags(pkg_config(qt_net_name))
        except OSError:
            announce("pkg_config is confused about the state of %s."
                     % qt_net_name)
            qt_env = None

# Set up for Python coveraging if needed
pycov_path = None
if have_coverage and env['coveraging'] and env['python_coverage']:
    pycov_default = opts.options[opts.keys().index('python_coverage')].default
    pycov_current = env['python_coverage']
    pycov_list = pycov_current.split()
    if env.GetOption('num_jobs') > 1 and pycov_current == pycov_default:
        pycov_list.append('--parallel-mode')
    # May need absolute path to coveraging tool if 'PythonXX' is prefixed
    pycov_path = env.WhereIs(pycov_list[0])
if pycov_path:
    pycov_list[0] = pycov_path
    env['PYTHON_COVERAGE'] = ' '.join(pycov_list)
    env['ENV']['PYTHON_COVERAGE'] = ' '.join(pycov_list)
else:
    env['python_coverage'] = ''  # So we see it in the options

# Two shared libraries provide most of the code for the C programs

# gpsd client library
libgps_sources = [
    "libgps/ais_json.c",
    "libgps/bits.c",
    "libgps/gpsdclient.c",
    "libgps/gps_maskdump.c",      # generated
    "libgps/gpsutils.c",
    "libgps/hex.c",
    "libgps/json.c",
    "libgps/libgps_core.c",
    "libgps/libgps_dbus.c",
    "libgps/libgps_json.c",
    "libgps/libgps_shm.c",
    "libgps/libgps_sock.c",
    "libgps/netlib.c",
    "libgps/ntpshmread.c",
    "libgps/os_compat.c",
    "libgps/rtcm2_json.c",
    "libgps/rtcm3_json.c",
    "libgps/shared_json.c",
    "libgps/timespec_str.c",
]

# Client sources not to be built as C++ when building the Qt library.
libgps_c_only = set([
    "libgps/ais_json.c",
    "libgps/json.c",
    "libgps/libgps_json.c",
    "libgps/os_compat.c",
    "libgps/rtcm2_json.c",
    "libgps/rtcm3_json.c",
    "libgps/shared_json.c",
    "libgps/timespec_str.c",
])

if env['libgpsmm']:
    libgps_sources.append("libgps/libgpsmm.cpp")

# gpsd server library
libgpsd_sources = [
    "gpsd/bsd_base64.c",
    "gpsd/crc24q.c",
    "drivers/driver_ais.c",
    "drivers/driver_allystar.c",
    "drivers/driver_casic.c",
    "drivers/driver_evermore.c",
    "drivers/driver_garmin.c",
    "drivers/driver_garmin_txt.c",
    "drivers/driver_geostar.c",
    "drivers/driver_greis.c",
    "drivers/driver_greis_checksum.c",
    "drivers/driver_italk.c",
    "drivers/driver_navcom.c",
    "drivers/driver_nmea0183.c",
    "drivers/driver_nmea2000.c",
    "drivers/driver_oncore.c",
    "drivers/driver_rtcm2.c",
    "drivers/driver_rtcm3.c",
    "drivers/drivers.c",
    "drivers/driver_sirf.c",
    "drivers/driver_skytraq.c",
    "drivers/driver_superstar2.c",
    "drivers/driver_tsip.c",
    "drivers/driver_ubx.c",
    "drivers/driver_zodiac.c",
    "gpsd/geoid.c",
    "gpsd/gpsd_json.c",
    "gpsd/isgps.c",
    "gpsd/libgpsd_core.c",
    "gpsd/matrix.c",
    "gpsd/net_dgpsip.c",
    "gpsd/net_gnss_dispatch.c",
    "gpsd/net_ntrip.c",
    "gpsd/ntpshmwrite.c",
    "gpsd/packet.c",
    "gpsd/ppsthread.c",
    "gpsd/pseudoais.c",
    "gpsd/pseudonmea.c",
    "gpsd/serial.c",
    "gpsd/subframe.c",
    "gpsd/timebase.c",
]

# Build ffi binding
#
packet_ffi_extension = [
    "gpsd/crc24q.c",
    "drivers/driver_greis_checksum.c",
    "drivers/driver_rtcm2.c",
    "libgps/gpspacket.c",
    "gpsd/isgps.c",
    "libgps/hex.c",
    "libgps/os_compat.c",
    "gpsd/packet.c",
    ]


if env["shared"]:
    def GPSLibrary(env, target, source, version, parse_flags=None):
        # Note: We have a possibility of getting either Object or file
        # list for sources, so we run through the sources and try to make
        # them into SharedObject instances.
        obj_list = []
        for s in Flatten(source):
            if isinstance(s, str):
                obj_list.append(env.SharedObject(s))
            else:
                obj_list.append(s)
        return env.SharedLibrary(target=target,
                                 source=obj_list,
                                 parse_flags=parse_flags,
                                 SHLIBVERSION=version)

    def GPSLibraryInstall(env, libdir, source, version):
        # note: osX lib name s/b libgps.VV.dylib
        # where VV is libgps_version_current
        inst = env.InstallVersionedLib(libdir, source, SHLIBVERSION=version)
        return inst
else:
    def GPSLibrary(env, target, source, version, parse_flags=None):
        return env.StaticLibrary(target,
                                 [env.StaticObject(s) for s in source],
                                 parse_flags=parse_flags)

    def GPSLibraryInstall(env, libdir, source, version):
        return env.Install(libdir, source)

libgps_shared = GPSLibrary(env=env,
                           target="gps",
                           source=libgps_sources,
                           version=libgps_version,
                           parse_flags=rtlibs + libgps_flags)

libgps_static = env.StaticLibrary(
    target="gps_static",
    source=[env.StaticObject(s) for s in libgps_sources],
    parse_flags=rtlibs + capflags)

libgpsd_static = env.StaticLibrary(
    target="gpsd",
    source=[env.StaticObject(s, parse_flags=usbflags + bluezflags)
            for s in libgpsd_sources],
    parse_flags=usbflags + bluezflags + capflags)

# FFI library must always be shared, even with shared=no.
packet_ffi_objects = [env.SharedObject(s) for s in packet_ffi_extension]
packet_ffi_shared = env.SharedLibrary(target="gpsdpacket",
                                      source=packet_ffi_objects,
                                      SHLIBVERSION=libgps_version,
                                      parse_flags=rtlibs + libgps_flags)

libraries = [libgps_shared, packet_ffi_shared]

# Only attempt to create the qt library if we have shared turned on
# otherwise we have a mismash of objects in library
if qt_env:
    qtobjects = []
    qt_flags = qt_env['CFLAGS']
    for c_only in (
        '--disable-excess-fp-precision',
        '-fexcess-precision=standard',
        '-Wmissing-prototypes',
        '-Wstrict-prototypes',
        '-Wmissing-declarations'
        ):
        if c_only in qt_flags:
            qt_flags.remove(c_only)
    # Qt binding object files have to be renamed as they're built to avoid
    # name clashes with the plain non-Qt object files. This prevents the
    # infamous "Two environments with different actions were specified
    # for the same target" error.
    for src in libgps_sources:
        if src not in libgps_c_only:
            compile_with = qt_env['CXX']
            compile_flags = qt_flags
        else:
            compile_with = qt_env['CC']
            compile_flags = qt_env['CFLAGS']
        qtobjects.append(qt_env.SharedObject(src,
                                             CC=compile_with,
                                             CFLAGS=compile_flags))
    compiled_qgpsmmlib = GPSLibrary(env=qt_env,
                                    target="Qgpsmm",
                                    source=qtobjects,
                                    version=libgps_version,
                                    parse_flags=libgps_flags)
    libraries.append(compiled_qgpsmmlib)

# The libraries have dependencies on system libraries
# libdbus appears multiple times because the linker only does one pass.

gpsflags = mathlibs + rtlibs + dbusflags
gpsdflags = usbflags + bluezflags + gpsflags

# Source groups

gpsd_sources = [
    'gpsd/dbusexport.c',
    'gpsd/gpsd.c',
    'gpsd/shmexport.c',
    'gpsd/timehint.c'
]

if env['systemd']:
    gpsd_sources.append("gpsd/sd_socket.c")

gpsmon_sources = [
    'gpsmon/gpsmon.c',
    'gpsmon/monitor_garmin.c',
    'gpsmon/monitor_italk.c',
    'gpsmon/monitor_nmea0183.c',
    'gpsmon/monitor_oncore.c',
    'gpsmon/monitor_sirf.c',
    'gpsmon/monitor_superstar2.c',
    'gpsmon/monitor_tnt.c',
    'gpsmon/monitor_ubx.c',
]

# Python dependencies
# For generated dependencies, this causes them to be generated as needed.
# For non-generated dependencies, it causes them to be duplicated into
# the build tree as needed.


# Symlink creator for uplevel access to the 'gps' package
def PylibLink(target, source, env):
    _ = source, env
    os.symlink('../gps', target[0].get_path())


# All installed python programs
# All are templated
python_progs = [
    "clients/gegps",
    "clients/gpscat",
    "clients/gpscsv",
    "clients/gpsplot",
    "clients/gpsprof",
    "clients/gpssubframe",
    "clients/ubxtool",
    "clients/zerk",
    "gpsfake",
]
if env['xgps']:
    python_progs.append("clients/xgps")
    python_progs.append("clients/xgpsspeed")

# Import dependencies
# Update these whenever the imports change

# Internal imports within 'gps' package
env.Depends('gps/__init__.py', ['gps/gps.py', 'gps/misc.py'])
env.Depends('gps/aiogps.py', ['gps/client.py', 'gps/gps.py', 'gps/misc.py'])
env.Depends('gps/client.py', ['gps/misc.py', 'gps/watch_options.py'])
env.Depends('gps/gps.py',
            ['gps/client.py', 'gps/misc.py', 'gps/watch_options.py'])
env.Depends('gps/fake.py', 'gps/packet.py')
env.Depends('gps/packet.py', 'gps/misc.py')

# All Python programs import the 'gps' package
env.Depends(python_progs, 'gps/__init__.py')

# Additional specific import cases
env.Depends('clients/gpscat', ['gps/packet.py', 'gps/misc.py'])
env.Depends('clients/gpsplot', 'gps/clienthelpers.py')
env.Depends('clients/gpsprof', 'gps/clienthelpers.py')
env.Depends('clients/ubxtool', 'gps/ubx.py')
env.Depends('clients/xgps', 'gps/clienthelpers.py')
env.Depends('clients/xgpsspeed', 'gps/clienthelpers.py')
env.Depends('clients/zerk', 'gps/misc.py')
env.Depends('gpsfake', ['gps/fake.py', 'gps/misc.py'])

# Symlink for the clients to find the 'gps' package in the build tree
env.Depends(python_progs, env.Command('clients/gps', '', PylibLink))

# Non-import dependencies
# Dependency on program
env.Depends('regress-driver', 'gpsfake')
# Dependency on FFI packet library
env.Depends('gps/packet.py', packet_ffi_shared)

# Production programs

cgps = env.Program('clients/cgps', ['clients/cgps.c'],
                   LIBS=[libgps_static],
                   parse_flags=gpsflags + ncurseslibs)
gps2udp = env.Program('clients/gps2udp', ['clients/gps2udp.c'],
                      LIBS=[libgps_static],
                      parse_flags=gpsflags)
gpsctl = env.Program('gpsctl', ['gpsctl.c'],
                     LIBS=[libgpsd_static, libgps_static],
                     parse_flags=gpsdflags + gpsflags)
gpsd = env.Program('gpsd/gpsd', gpsd_sources,
                   LIBS=[libgpsd_static, libgps_static],
                   parse_flags=gpsdflags + gpsflags + capflags)
gpsdctl = env.Program('clients/gpsdctl', ['clients/gpsdctl.c'],
                      LIBS=[libgps_static],
                      parse_flags=gpsflags)
gpsdecode = env.Program('clients/gpsdecode', ['clients/gpsdecode.c'],
                        LIBS=[libgpsd_static, libgps_static],
                        parse_flags=gpsdflags + gpsflags)
# FIXME: gpsmon should not link to gpsd server sources!
gpsmon = env.Program('gpsmon/gpsmon', gpsmon_sources,
                     LIBS=[libgpsd_static, libgps_static],
                     parse_flags=gpsdflags + gpsflags + ncurseslibs)
gpspipe = env.Program('clients/gpspipe', ['clients/gpspipe.c'],
                      LIBS=[libgps_static],
                      parse_flags=gpsflags)
gpsrinex = env.Program('clients/gpsrinex', ['clients/gpsrinex.c'],
                       LIBS=[libgps_static],
                       parse_flags=gpsflags)
gpssnmp = env.Program('clients/gpssnmp', ['clients/gpssnmp.c'],
                      LIBS=[libgps_static],
                      parse_flags=gpsflags)
gpxlogger = env.Program('clients/gpxlogger', ['clients/gpxlogger.c'],
                        LIBS=[libgps_static],
                        parse_flags=gpsflags)
lcdgps = env.Program('clients/lcdgps', ['clients/lcdgps.c'],
                     LIBS=[libgps_static],
                     parse_flags=gpsflags)
ntpshmmon = env.Program('clients/ntpshmmon', ['clients/ntpshmmon.c'],
                        LIBS=[libgps_static],
                        parse_flags=gpsflags)
ppscheck = env.Program('clients/ppscheck', ['clients/ppscheck.c'],
                       LIBS=[libgps_static],
                       parse_flags=gpsflags)

bin_binaries = []
bin_scripts = []
sbin_binaries = []
if env["gpsd"]:
    sbin_binaries += [gpsd]

if env["gpsdclients"]:
    sbin_binaries += [gpsdctl]
    bin_binaries += [
        gps2udp,
        gpsctl,
        gpsdecode,
        gpspipe,
        gpsrinex,
        gpssnmp,
        gpxlogger,
        lcdgps
    ]
    bin_scripts += [
        'clients/gpsdebuginfo',
    ]

if env["timeservice"] or env["gpsdclients"]:
    bin_binaries += [ntpshmmon]
    if tiocmiwait:
        bin_binaries += [ppscheck]

    if env["ncurses"]:
        bin_binaries += [cgps, gpsmon]
    else:
        announce("WARNING: ncurses not found, not building cgps or gpsmon.",
                 end=True)

# Test programs - always link locally and statically
test_bits = env.Program('tests/test_bits',
                        [libgps_static, 'tests/test_bits.c'],
                        LIBS=[libgps_static])
test_float = env.Program('tests/test_float', ['tests/test_float.c'])
test_geoid = env.Program('tests/test_geoid',
                         [libgpsd_static, libgps_static, 'tests/test_geoid.c'],
                         LIBS=[libgpsd_static, libgps_static],
                         parse_flags=gpsdflags)
test_gpsdclient = env.Program('tests/test_gpsdclient',
                              [libgps_static, 'tests/test_gpsdclient.c'],
                              LIBS=[libgps_static, 'm'])
test_matrix = env.Program('tests/test_matrix',
                          [libgpsd_static, libgps_static, 'tests/test_matrix.c'],
                          LIBS=[libgpsd_static, libgps_static],
                          parse_flags=gpsdflags)
test_mktime = env.Program('tests/test_mktime',
                          [libgps_static, 'tests/test_mktime.c'],
                          LIBS=[libgps_static], parse_flags=mathlibs + rtlibs)
test_packet = env.Program('tests/test_packet',
                          [libgpsd_static, libgps_static,'tests/test_packet.c'],
                          LIBS=[libgpsd_static, libgps_static],
                          parse_flags=gpsdflags)
test_timespec = env.Program('tests/test_timespec', ['tests/test_timespec.c'],
                            LIBS=[libgpsd_static, libgps_static],
                            parse_flags=gpsdflags)
test_trig = env.Program('tests/test_trig', ['tests/test_trig.c'],
                        parse_flags=mathlibs)
# test_libgps for glibc older than 2.17
test_libgps = env.Program('tests/test_libgps',
                          [libgps_static, 'tests/test_libgps.c'],
                          LIBS=[libgps_static],
                          parse_flags=mathlibs + rtlibs + dbusflags)

if env['socket_export']:
    test_json = env.Program(
        'tests/test_json',
        [libgps_static, 'tests/test_json.c'],
        LIBS=[libgps_static],
        parse_flags=mathlibs + rtlibs + usbflags + dbusflags)
else:
    announce("test_json not building because socket_export is disabled")
    test_json = None

# duplicate below?
test_gpsmm = env.Program('tests/test_gpsmm',
                         [libgps_static, 'tests/test_gpsmm.cpp'],
                         LIBS=[libgps_static],
                         parse_flags=mathlibs + rtlibs + dbusflags)
testprogs = [test_bits,
             test_float,
             test_geoid,
             test_gpsdclient,
             test_libgps,
             test_matrix,
             test_mktime,
             test_packet,
             test_timespec,
             test_trig]
if env['socket_export'] or cleaning:
    testprogs.append(test_json)
if env["libgpsmm"] or cleaning:
    testprogs.append(test_gpsmm)

# Python programs
# python misc helpers and stuff, not to be installed
python_misc = [
    "libgps/jsongen.py",
    "maskaudit.py",
    "tests/test_clienthelpers.py",
    "tests/test_misc.py",
    "tests/test_xgps_deps.py",
    "www/gpscap.py",
    "valgrind-audit.py"
]

# Dependencies for imports in test programs
env.Depends('tests/test_clienthelpers.py',
            ['gps/__init__.py', 'gps/clienthelpers.py', 'gps/misc.py'])
env.Depends('tests/test_misc.py', ['gps/__init__.py', 'gps/misc.py'])
env.Depends('valgrind-audit.py', ['gps/__init__.py', 'gps/fake.py'])

# Symlink for the programs to find the 'gps' package in the build tree
env.Depends(['tests/test_clienthelpers.py', 'tests/test_misc.py'],
            env.Command('tests/gps', '', PylibLink))

# Glob() has to be run after all buildable objects defined.
# Glob(), by default, looks in the file tree, and current buildable objects.
python_modules = Glob('gps/*.py', strings=True) + ['gps/__init__.py',
                                                   'gps/gps.py',
                                                   'gps/packet.py']

# Remove the aiogps module if not configured
# Don't use Glob's exclude option, since it may not be available
if 'aiogps' in env and env['aiogps']:
    python_misc.extend(["example_aiogps.py", "example_aiogps_run"])
else:
    try:
        python_modules.remove('gps/aiogps.py')
    except ValueError:
        pass

# Make PEP 241 Metadata 1.0.
# Why not PEP 314 (V1.1) or PEP 345 (V1.2)?
# V1.1 and V1.2 require a Download-URL to an installable binary
python_egg_info_source = """Metadata-Version: 1.0
Name: gps
Version: %s
Summary: Python libraries for the gpsd service daemon
Home-page: %s
Author: the GPSD project
Author-email: %s
License: BSD
Keywords: GPS
Description: The gpsd service daemon can monitor one or more GPS devices \
connected to a host computer, making all data on the location and movements \
of the sensors available to be queried on TCP port 2947.
Platform: UNKNOWN
""" % (gpsd_version, website, devmail)
python_egg_info = env.Textfile(target="gps-%s.egg-info" % (gpsd_version, ),
                               source=python_egg_info_source)
python_targets = ([python_egg_info] + python_progs + python_modules)


env.Command(target="include/packet_names.h", source="include/packet_states.h",
            action="""
    rm -f $TARGET &&\
    sed -e '/^ *\\([A-Z][A-Z0-9_]*\\),/s//   \"\\1\",/' <$SOURCE >$TARGET &&\
    chmod a-w $TARGET""")

env.Textfile(target="include/gpsd_config.h", source=confdefs)

env.Command(target="libgps/gps_maskdump.c",
            source=["maskaudit.py", "include/gps.h", "include/gpsd.h"],
            action='''
    rm -f $TARGET &&\
        $SC_PYTHON $SOURCE -c "${SRCDIR}" > $TARGET &&\
        chmod a-w $TARGET''')

env.Command(target="libgps/ais_json.i", source="libgps/jsongen.py",
            action='''\
    rm -f $TARGET &&\
    $SC_PYTHON $SOURCE --ais --target=parser >$TARGET &&\
    chmod a-w $TARGET''')


if env['systemd']:
    udevcommand = 'TAG+="systemd", ENV{SYSTEMD_WANTS}="gpsdctl@%k.service"'
else:
    udevcommand = 'RUN+="%s/gpsd.hotplug"' % (env['udevdir'], )

# FIXME: why do this every time scons is called?
# $variantdir may not exist when this is run.
pythonize_header_match = re.compile(r'\s*#define\s+(\w+)\s+(\w+)\s*.*$[^\\]')
pythonized_header = ''
with open(env['SRCDIR'] + '/../include/gpsd.h') as sfp:
    for content in sfp:
        _match3 = pythonize_header_match.match(content)
        if _match3:
            if 'LOG' in content or 'PACKET' in content:
                pythonized_header += ('%s = %s\n' %
                                      (_match3.group(1), _match3.group(2)))

if ((env['python'] and
     not cleaning and
     not helping and
     def_target_python != env['target_python'])):
    # non-default target python.
    if def_python_shebang == env['python_shebang']:
        # default python shebang, update to match target python
        if os.sep == env['target_python'][0]:
            # full path, no need for env
            env['python_shebang'] = env['target_python']
        else:
            # partial path, need env
            env['python_shebang'] = "/usr/bin/env %s" % env['target_python']
        announce("Setting python_shebang to %s" % env['python_shebang'])

# tuples for Substfile.  To convert .in files to generated files.
substmap = (
    ('@ANNOUNCE@',   annmail),
    ('@BUGTRACKER@', bugtracker),
    ('@CGIUPLOAD@',  cgiupload),
    ('@CLONEREPO@',  clonerepo),
    ('@DEVMAIL@',    devmail),
    ('@DOWNLOAD@',   download),
    ('@FORMSERVER@', formserver),
    ('@GENERATED@',  "This code is generated by scons.  Do not hand-hack it!"),
    ('@GITREPO@',    gitrepo),
    ('@GPSAPIVERMAJ@', api_version_major),
    ('@GPSAPIVERMIN@', api_version_minor),
    ('@GPSPACKET@',  packet_ffi_shared[0].get_path()),
    ('@ICONPATH@',   installdir('icondir', add_destdir=False)),
    ('@INCLUDEDIR@', installdir('includedir', add_destdir=False)),
    ('@IRCCHAN@',    ircchan),
    ('@ISSUES@',     bugtracker),
    ('@LIBDIR@',     installdir('libdir', add_destdir=False)),
    ('@LIBGPSVERSION@', libgps_version),
    ('@MAILMAN@',    mailman),
    ('@MAINPAGE@',   mainpage),
    ('@MASTER@',     'DO NOT HAND_HACK! THIS FILE IS GENERATED'),
    ('@MIBPATH',     installdir('mibdir', add_destdir=False)),
    ('@PREFIX@',     env['prefix']),
    ('@PROJECTPAGE@', projectpage),
    # PEP 394 and 397 python shebang
    ('@PYSHEBANG@',  env['python_shebang']),
    ('@PYPACKETH@',  pythonized_header),
    ('@QTVERSIONED@', env['qt_versioned']),
    ('@RUNDIR@',     env['rundir']),
    ('@SBINDIR@',    installdir('sbindir', add_destdir=False)),
    ('@SCPUPLOAD@',  scpupload),
    ('@SHAREPATH@',  installdir('sharedir', add_destdir=False)),
    ('@SITENAME@',   sitename),
    ('@SITESEARCH@', sitesearch),
    ('@SUPPORT@',    'https://gpsd.io/SUPPORT.html'),
    ('@TIPLINK@',    tiplink),
    ('@TIPWIDGET@',  tipwidget),
    ('@UDEVCOMMAND@', udevcommand),
    ('@USERMAIL@',   usermail),
    ('@VERSION@',    gpsd_version),
    ('@WEBSITE@',    website),
)


# Keep time-dependent version separate
# FIXME: Come up with a better approach with reproducible builds
substmap_dated = substmap + (('@DATE@', time.asctime()),)

# explicit templated files
templated = {
    "android/gpsd_config": "android/gpsd_config.in",
    "clients/gegps": "clients/gegps.py.in",
    "clients/gpscat": "clients/gpscat.py.in",
    "clients/gpscsv": "clients/gpscsv.py.in",
    "clients/gpsd.php": "clients/gpsd.php.in",
    "clients/gpsplot": "clients/gpsplot.py.in",
    "clients/gpsprof": "clients/gpsprof.py.in",
    "clients/gpssubframe": "clients/gpssubframe.py.in",
    "clients/ubxtool": "clients/ubxtool.py.in",
    "clients/xgps": "clients/xgps.py.in",
    "clients/xgpsspeed": "clients/xgpsspeed.py.in",
    "clients/zerk": "clients/zerk.py.in",
    "contrib/ntpshmviz": "contrib/ntpshmviz.py.in",
    "contrib/skyview2svg.py": "contrib/skyview2svg.py.in",
    "contrib/webgps": "contrib/webgps.py.in",
    "control": "control.in",
    "gpsd.rules": "gpsd.rules.in",
    "gpsfake": "gpsfake.py.in",
    "gps/gps.py": "gps/gps.py.in",
    "gps/__init__.py": "gps/__init__.py.in",
    "gps/packet.py": "gps/packet.py.in",
    "libgps.pc": "libgps.pc.in",
    "libQgpsmm.prl": "libQgpsmm.prl.in",
    "packaging/deb/etc_default_gpsd": "packaging/deb/etc_default_gpsd.in",
    "packaging/deb/etc_init.d_gpsd": "packaging/deb/etc_init.d_gpsd.in",
    "packaging/gpsd-setup.py": "packaging/gpsd-setup.py.in",
    "packaging/rpm/gpsd.init": "packaging/rpm/gpsd.init.in",
    "packaging/rpm/gpsd.spec": "packaging/rpm/gpsd.spec.in",
    "packaging/X11/xgps.desktop": "packaging/X11/xgps.desktop.in",
    "packaging/X11/xgpsspeed.desktop": "packaging/X11/xgpsspeed.desktop.in",
    "Qgpsmm.pc": "Qgpsmm.pc.in",
    "systemd/gpsdctl@.service": "systemd/gpsdctl@.service.in",
    "systemd/gpsd.service": "systemd/gpsd.service.in",
    "systemd/gpsd.socket": "systemd/gpsd.socket.in",
    "www/faq.html": "www/faq.html.in",
    "www/gps_report.cgi": "www/gps_report.cgi.in",
    "www/gpscap.py": "www/gpscap.py.in",
    "www/hacking.html": "www/hacking.html.in",
    "www/hardware-head.html": "www/hardware-head.html.in",
    "www/index.html": "www/index.html.in",
    "www/troubleshooting.html": "www/troubleshooting.html.in",
    }

for (tgt, src) in templated.items():
    iswww = tgt.startswith('www/')
    # Only www pages need @DATE@ expansion, which forces rebuild every time
    subst = substmap_dated if iswww else substmap
    # use scons built-in Substfile()
    builder = env.Substfile(target=tgt, source=src, SUBST_DICT=subst)
    # default to building all built targets, except www
    # FIXME: Render this unnecessary
    if not iswww:
        env.Default(builder)
    # set read-only to alert people trying to edit the files.
    env.AddPostAction(builder, 'chmod -w $TARGET')
    if ((src.endswith(".py.in") or
         tgt in python_progs or
         tgt in ['contrib/ntpshmviz', 'contrib/webgps'])):
        # set python files to executable
        env.AddPostAction(builder, 'chmod +x $TARGET')

# When the URL declarations change, so must the generated web pages
for fn in glob.glob("www/*.in"):
    env.Depends(fn[:-3], ["SConstruct", "SConscript"])

# asciidoc documents
asciidocs = []

man_env = env.Clone()
if man_env.GetOption('silent'):
    man_env['SPAWN'] = filtered_spawn  # Suppress stderr chatter
manpage_targets = []
maninstall = []
if adoc_prog:
    adoc_args_m = ('-v -a gpsdweb=%s -a gpsdver=%s' % (website, gpsd_version))
    adoc_args = (adoc_args_m + ' -a docinfo=shared')
    for (man, src) in all_manpages.items():
        # build it
        # make nroff man page
        asciidocs.append(man)
        env.Command(man, src,
                    '%s -b manpage %s -o $TARGET $SOURCE' %
                    (adoc_prog, adoc_args_m))
        # install nroff man page
        section = man.split(".")[1]
        dest = os.path.join(installdir('mandir'), "man" + section)
        maninstall.append(env.Install(target=dest, source=man))

        # make html man page
        target = 'www/%s.html' % os.path.basename(man[:-2])
        env.Depends(src, ['www/docinfo.html', 'www/inc-menu.adoc'])
        tgt = env.Command(target, src,
            '%s -b html5 %s -a docinfodir=../www/ -o $TARGET $SOURCE' %
            (adoc_prog, adoc_args))
        asciidocs.append(tgt)
else:
    # can't build man pages, maybe we have pre-built ones?
    for man in Glob('man/*.?', strings=True):
        section = man.split(".")[1]
        dest = os.path.join(installdir('mandir'), "man" + section)
        maninstall.append(env.Install(target=dest, source=man))


# The hardware page
env.Command('www/hardware.html',
            ['www/gpscap.py',
             'www/hardware-head.html',
             'www/gpscap.ini',
             'www/hardware-tail.html'],
            ['cd %s/www; (cat hardware-head.html && PYTHONIOENCODING=utf-8 '
             '$SC_PYTHON gpscap.py && cat hardware-tail.html) '
             '> hardware.html' % variantdir])

# doc to install in 'docdir'
docinstall = env.Install(target=installdir('docdir'), source=doc_files)

if adoc_prog:
    adocfiles = (('build', 'www/building'),
                 ('INSTALL', 'www/installation'),
                 ('README', 'www/README'),
                 ('SUPPORT', 'www/SUPPORT'),
                 ('www/AIVDM', 'www/AIVDM'),
                 ('www/client-howto', 'www/client-howto'),
                 ('www/gpsd-numbers-matter',
                  'www/gpsd-numbers-matter'),
                 ('www/gpsd-client-example-code',
                  'www/gpsd-client-example-code'),
                 ('www/gpsd-time-service-howto',
                  'www/gpsd-time-service-howto'),
                 ('www/internals', 'www/internals'),
                 ('www/NMEA', 'www/NMEA'),
                 ('www/ppp-howto', 'www/ppp-howto'),
                 ('www/protocol-evolution', 'www/protocol-evolution'),
                 ('www/protocol-transition', 'www/protocol-transition'),
                 ('www/replacing-nmea', 'www/replacing-nmea'),
                 ('www/time-service-intro', 'www/time-service-intro'),
                 ('www/ubxtool-examples', 'www/ubxtool-examples'),
                 ('www/writing-a-driver', 'www/writing-a-driver'),
                 ('www/performance/performance',
                  'www/performance/performance'),
                 )
    for src, tgt in adocfiles:
        target = '%s.html' % tgt
        env.Depends(src, ['www/docinfo.html',
                          'www/example1.c.txt',
                          'www/example2.py.txt',
                          'www/inc-menu.adoc'])
        tgt = env.Command(target, '%s.adoc' % src,
            '%s -b html5 %s -o $TARGET $SOURCE' %
            (adoc_prog, adoc_args))
        asciidocs.append(tgt)

# Non-asciidoc, plain html webpages only
# example1.c has a .txt extension to avoid an scons bug where the
# install rule for gps.h is invoked during the build step when there
# is a CPPFLAGS matching gpsd's install prefix.
htmlpages = [
    'www/bt.html',
    'www/bu_303b.html',
    'www/example1.c.txt',
    'www/example2.py.txt',
    'www/excellence.html',
    'www/for-vendors.html',
    'www/future.html',
    'www/gps-hacking.html',
    'www/gypsy.html',
    'www/hall-of-shame.html',
    'www/hardware.html',       # built above
    'www/history.html',
    'www/references.html',
    'www/reliability.html',
    'www/upstream-bugs.html',
    'www/wishlist.html',
    'www/xgps-sample.html',
    ]

wwwpage_targets = []

# webapges from .in files
webpages_in = list(map(lambda f: f[3:-3], glob.glob("../www/*.in")))
webpages_in_not = ('www/hardware-tail.html')
for fn in webpages_in_not:
    if fn in webpages_in:
        webpages_in.remove(fn)

# webapges extras: images, css, js
webpages_x_list = ('../www/*.css',
                   '../www/*.gif',
                   '../www/*.ico',
                   '../www/*.js',
                   '../www/*.png',
                   '../www/*.svg',
                   '../www/performance/*css',
                   '../www/performance/*png',
                   '../www/performance/*txt',
                   )
webpages_x = []
for glb in webpages_x_list:
    webpages_x += list(map(lambda f: f[3:], glob.glob(glb)))

webpages_static = [('www/NEWS', 'NEWS'),
                   ('www/TODO', 'TODO'),
                   ('www/gpsdebuginfo', 'clients/gpsdebuginfo'),
                   ]
for page in webpages_static:
    targ = env.Command(page[0], page[1], 'cp $SOURCE $TARGET')
    webpages_x += targ

webpages = htmlpages + asciidocs + wwwpage_targets + webpages_in + webpages_x
www = env.Alias('www', webpages)

# On the Mac (at least), some X11 programs launch the X11 server even when
# they're not actually using the display.  Clearing DISPLAY in the
# environment avoids this.  We leave the main environment untouched just in
# case it might be needed.
nox11_env = env['ENV'].copy()
nox11_env['DISPLAY'] = ''

# The diagram editor dia is required in order to edit the diagram masters
if have_dia:
    env.Command("www/cycle.svg", ["www/cycle.dia"],
                ["cd %s; dia -e www/cycle.svg www/cycle.dia" % variantdir],
                ENV=nox11_env)

packing = [
    'packaging/deb/etc_default_gpsd',
    'packaging/deb/etc_init.d_gpsd',
    'packaging/gpsd-setup.py',
    'packaging/README.PACKAGERS',
    'packaging/rpm/gpsd.init',
    'packaging/rpm/gpsd.spec',
    'packaging/rpm/gpsd.sysconfig',
    'packaging/X11/xgps.desktop',
    'packaging/X11/xgpsspeed.desktop',
    ]

# Where it all comes together

build_src = [
    bin_binaries,
    bin_scripts,
    "clients/gpsd.php",
    "gpsd.rules",
    icon_files,
    "libgps.pc",
    libraries,
    manpage_targets,
    mib_files,
    packing,
    sbin_binaries,
    webpages,
    ]

if env['python']:
    build_src.append(python_targets)
build = env.Alias('build', build_src)

# For debug, to dump the build environment
# print(env.Dump())

if [] == COMMAND_LINE_TARGETS:
    # 'build' is default target
    Default('build')

if qt_env:
    # duplicate above?
    test_qgpsmm = env.Program('tests/test_qgpsmm', ['tests/test_gpsmm.cpp'],
                              LIBPATH=['.'],
                              OBJPREFIX='qt-',
                              LIBS=['Qgpsmm'])
    build_qt = qt_env.Alias('build', [compiled_qgpsmmlib, test_qgpsmm])
    qt_env.Default(*build_qt)
    testprogs.append(test_qgpsmm)

# Installation and deinstallation

# Not here because too distro-specific: udev rules, desktop files, init scripts

# It's deliberate that we don't install gpsd.h. It's full of internals that
# third-party client programs should not see.
headerinstall = [env.Install(installdir('includedir'), x)
                 for x in ("include/libgpsmm.h", "include/gps.h")]

binaryinstall = []
binaryinstall.append(env.Install(installdir('sbindir'), sbin_binaries))
binaryinstall.append(env.Install(installdir('bindir'), bin_binaries))
binaryinstall.append(GPSLibraryInstall(env, installdir('libdir'),
                                       libgps_shared,
                                       libgps_version))
# FFI library is always shared
binaryinstall.append(env.InstallVersionedLib(installdir('libdir'),
                                             packet_ffi_shared,
                                             SHLIBVERSION=libgps_version))

if qt_env:
    binaryinstall.append(GPSLibraryInstall(qt_env, installdir('libdir'),
                                           compiled_qgpsmmlib, libgps_version))

if ((not env['debug'] and
     not env['debug_opt'] and
     not env['profiling'] and
     not env['nostrip'] and
     not env['target_platform'].startswith('darwin'))):
    env.AddPostAction(binaryinstall, '$STRIP $TARGET')

binaryinstall.append(env.Install(installdir('bindir'), bin_scripts))

python_module_dir = str(python_libdir) + os.sep + 'gps'

python_modules_install = env.Install(DESTDIR + python_module_dir,
                                     python_modules)

python_progs_install = env.Install(installdir('bindir'), python_progs)

python_egg_info_install = env.Install(DESTDIR + str(python_libdir),
                                      python_egg_info)
python_install = [python_modules_install,
                  python_progs_install,
                  python_egg_info_install,
                  # We don't need the directory explicitly for the
                  # install, but we do need it for the uninstall
                  Dir(DESTDIR + python_module_dir)]

python_lint = (python_misc + python_modules + python_progs +
               ['SConstruct', 'SConscript'])

if env['python']:
    # Check that Python modules compile properly
    # FIXME: why not install some of the .pyc?
    check_compile = []
    for p in python_lint:
        # split in two lines for readability
        check_compile.append(
            'cp %s/%s tmp.py; %s -tt -m py_compile tmp.py;' %
            (variantdir, p, target_python_path))
        # tmp.py may have inherited non-writable permissions
        check_compile.append('rm -f tmp.py*')

    python_compilation_regress = Utility('python-compilation-regress',
                                         python_lint, check_compile)

    # get version from each python prog
    # this ensures they can run and gps_versions match
    vchk = ''
    pp = []
    for p in python_progs:
        if not env['xgps_deps']:
            if p in ['clients/xgps', 'clients/xgpsspeed']:
                # do not have xgps* dependencies, don't test
                # FIXME: make these do -V w/o dependencies.
                continue
        # need to run in variantdir to find libgpsdpacket
        tgt = Utility(
            'version-%s' % p, p,
            'cd %s; $PYTHON %s -V' % (variantdir, p),
            ENV=nox11_env)
        pp.append(tgt)
    python_versions = env.Alias('python-versions', pp)
else:
    python_install = []

pc_install = [env.Install(installdir('pkgconfig'), 'libgps.pc')]
if qt_env:
    pc_install.append(qt_env.Install(installdir('pkgconfig'), 'Qgpsmm.pc'))
    pc_install.append(qt_env.Install(installdir('libdir'), 'libQgpsmm.prl'))


# icons to install
docinstall += env.Install(target=installdir('icondir'), source=icon_files)

# MIB to install
mibinstall = env.Install(target=installdir('mibdir'), source=mib_files)

# and now we know everything to install
install_src = (binaryinstall +
               docinstall +
               headerinstall +
               maninstall +
               mibinstall +
               pc_install +
               python_install)

install = env.Alias('install', install_src)


def Uninstall(nodes):
    deletes = []
    for node in nodes:
        if node.__class__ == install[0].__class__:
            deletes.append(Uninstall(node.sources))
        else:
            deletes.append(Delete(str(node)))
    return deletes


uninstall = env.Command('uninstall', '',
                        Flatten(Uninstall(Alias("install"))) or "")
env.AlwaysBuild(uninstall)
env.Precious(uninstall)
env.Alias('uninstall', uninstall)

# Target selection for '.' is badly broken. This is a general scons problem,
# not a glitch in this particular recipe. Avoid triggering the bug.


def error_action(target, source, env):
    raise SCons.Error.UserError("Target selection for '.' is broken.")


AlwaysBuild(Alias(".", [], error_action))


#
# start audit checks section
#

# Putting in all these -U flags speeds up cppcheck and allows it to look
# at configurations we actually care about.
# https://github.com/danmar/cppcheck
cppcheck = Utility("cppcheck",
                   ['build', "include/gpsd.h", "include/packet_names.h"],
                   "cppcheck -U__UNUSED__ -UUSE_QT -U__COVERITY__ "
                   "-U__future__ "
                   "-ULIMITED_MAX_CLIENTS -ULIMITED_MAX_DEVICES -UAF_UNSPEC "
                   "-UINADDR_ANY -U_WIN32 -U__CYGWIN__ "
                   "-UPATH_MAX -UHAVE_STRLCAT -UHAVE_STRLCPY "
                   "-UIPTOS_LOWDELAY -UIPV6_TCLASS -UTCP_NODELAY -UTIOCMIWAIT "
                   "--template gcc "
                   "--enable=all --inline-suppr "
                   "--suppress='*:drivers/driver_proto.c' "
                   "--force '${SRCDIR}'")

# Conflicts with pycodestyle:
#   E121 continuation line under-indented for hanging indent
#   E123 closing bracket does not match indentation of opening bracket's line
# Conflist with gpsd style
#   W504 line break after binary operator
# --exit-zero always return success, so later audits will run
flake8 = Utility("flake8", python_lint,
                 ['flake8 --ignore=E121,E122,E123,E241,E401,E501,W504,W602 '
                  '--exit-zero $SOURCES'])

# Additional Python readability style checks
# Oddly these only happen when called this way?
#   E121 continuation line under-indented for hanging indent
#   E123 closing bracket does not match indentation of opening bracket's line
# Conflicts with gpsd style
#   W504 line break after binary operator
# exit 0 so the rest of the audit runs
pycodestyle = Utility("pep8", python_lint,
                      ['pycodestyle --ignore=E121,E122,E123,E241,W504,W602 '
                       '$SOURCES; exit 0'])
# pep8 was renamed to pycodestyle, same thing
env.Alias('pycodestyle', pycodestyle)

# Sanity-check Python code.
# Bletch.  We don't really want to suppress W0231 E0602 E0611 E1123,
# but Python 3 syntax confuses a pylint running under Python 2.
# There's an internal error in astroid that requires we disable some
# auditing. This is irritating as hell but there's no help for it short
# of an upstream fix.

# --exit-zero always return success, so later audits will run
pylint = Utility("pylint", python_lint, [
    'pylint --rcfile=/dev/null --dummy-variables-rgx=\'^_\' '
    '--exit-zero --msg-template='
    '"{path}:{line}: [{msg_id}({symbol}), {obj}] {msg}" '
    '--reports=n --disable=F0001,C0103,C0111,C1001,C0301,C0122,C0302,'
    'C0322,C0324,C0323,C0321,C0330,C0411,C0413,E1136,R0201,R0204,'
    'R0801,'
    'R0902,R0903,R0904,R0911,R0912,R0913,R0914,R0915,W0110,W0201,'
    'W0121,W0123,W0231,W0232,W0234,W0401,W0403,W0141,W0142,W0603,'
    'W0614,W0640,W0621,W1504,E0602,E0611,E1101,E1102,E1103,E1123,'
    'F0401,I0011 $SOURCES'])

# Try to make "scan-build" call the same scons
# executable that is currently executing this SConstruct.

# Check with scan-build, an analyzer, part of clang
scan_build = Utility("scan-build",
                     ["include/gpsd.h", "include/packet_names.h"],
                     "scan-build " + scons_executable_name)
env.Alias('scan_build', scan_build)  # For '_' vs. '-'

# Run a valgrind audit on the daemon  - not in normal tests
valgrind = Utility('valgrind', [
    'valgrind-audit.py', gpsd],
    '$PYTHON "${SRCDIR}/valgrind-audit.py"'
)

# Perform all (possible) local code-sanity checks (but not the Coverity scan).
audits = []
if have_cppcheck:
    audits.append(cppcheck)
if have_flake8:
    audits.append(flake8)
if have_pycodestyle:
    audits.append(pycodestyle)
if have_pylint:
    audits.append(pylint)
if have_scan_build:
    audits.append(scan_build)
if have_valgrind:
    audits.append(valgrind)
env.Alias('audit', audits)

#
# end audit checks section
#

# Regression tests begin here
#
# Note that the *-makeregress targets re-create the *.log.chk source
# files from the *.log source files.

# Unit-test the bitfield extractor
bits_regress = Utility('bits-regress', [test_bits], [
    '"${SRCDIR}/tests/test_bits" --quiet'
])

# Unit-test the deg_to_str() converter
deg_regress = Utility('deg-regress', [test_gpsdclient], [
    '"${SRCDIR}/tests/test_gpsdclient"'
])

# Unit-test the bitfield extractor
matrix_regress = Utility('matrix-regress', [test_matrix], [
    '"${SRCDIR}/tests/test_matrix" --quiet'
])

# MIB test
if have_smilint:
    mib_regress = Utility('mib-regress', [mib_lint], [
        'smilint -l 6 man/GPSD-MIB'
    ])
else:
    mib_regress = []

# Regression-test NMEA 2000
if ((env["nmea2000"] and
     have_canplayer)):
    # the log files must be dependencies so they get copied into variant_dir
    nmea2000_logs = Glob("test/nmea2000/*.log", strings=True)
    nmea2000_tests = []
    for nmea2000_log in nmea2000_logs:
        # oddly this runs in build root, but needs to run in variant_dir
        tgt = Utility(
            'nmea2000-regress-' + nmea2000_log[:-4],
            ['tests/test_nmea2000', nmea2000_log, nmea2000_log + '.chk'],
            'cd %s; "${SRCDIR}/tests/test_nmea2000" %s' %
            (variantdir, nmea2000_log))
        nmea2000_tests.append(tgt)
    nmea2000_regress = env.Alias('nmea2000-regress', nmea2000_tests)

else:
    nmea2000_regress = None
    if not cleaning and not helping:
        announce("NMEA2000 regression tests suppressed because nmea200 is off "
                 "or canplayer is missing.")

# using regress-drivers requires socket_export being enabled and Python
if env['socket_export'] and env['python']:
    # Regression-test the daemon.
    # But first dump the platform and its delay parameters.
    gps_herald = Utility(
        'gps-herald', [gpsd, gpsctl, '${SRCDIR}/gpsfake'],
        'cd %s; $PYTHON $PYTHON_COVERAGE "${SRCDIR}/gpsfake" -T' % variantdir)

    gps_log_pattern = "test/daemon/*.log"
    gps_logs = Glob(gps_log_pattern, strings=True)
    gps_tests = []
    for gps_log in gps_logs:
        # oddly this runs in build root, but needs to run in variant_dir
        tgt = Utility(
            'gps-regress-' + gps_log[:-4],
            [gps_herald, gps_log],
            'cd %s; ./regress-driver -q -o -t $REGRESSOPTS %s' %
            (variantdir, gps_log))
        gps_tests.append(tgt)
    gps_regress = env.Alias('gps-regress', gps_tests)

    # the log files must be dependencies so they get copied into variant_dir
    gpsfake_logs = Glob('test/daemon/*')
    # Run the passthrough log in all transport modes for better coverage
    gpsfake_tests = []
    for name, opts in [['pty', ''], ['udp', '-u'], ['tcp', '-o -t']]:
        # oddly this runs in build root, but needs to run in variant_dir
        tgt = Utility(
            'gpsfake-' + name,
            [gps_herald, gpsfake_logs],
            'cd %s; ./regress-driver $REGRESSOPTS -q %s %s' %
            (variantdir, opts, 'test/daemon/passthrough.log'))
        gpsfake_tests.append(tgt)
    env.Alias('gpsfake-tests', gpsfake_tests)

    # Build the regression tests for the daemon.
    # Note: You'll have to do this whenever the default leap second
    # changes in gpsd.h.  Many drivers rely on the default until they
    # get the current leap second.
    gps_rebuilds = []
    for gps_log in gps_logs:
        # oddly this runs in build root, but needs to run in variant_dir
        gps_rebuilds.append(Utility(
            'gps-makeregress-' + gps_log[:-4],
            [gps_herald, gps_log],
            'cd %s; ./regress-driver -bq -o -t '
            '$REGRESSOPTS %s ' % (variantdir, gps_log)))
    if GetOption('num_jobs') <= 1:
        Utility('gps-makeregress', gps_herald,
                'cd %s; ./regress-driver -b $REGRESSOPTS %s' %
                (variantdir, gps_log_pattern))
    else:
        env.Alias('gps-makeregress', gps_rebuilds)
else:
    announce("GPS regression tests suppressed because socket_export "
             "or python is off.")
    gps_regress = None
    gpsfake_tests = None

# To build an individual test for a load named foo.log, put it in
# test/daemon and do this:
#    regress-driver -b test/daemon/foo.log

# Regression-test the RTCM decoder.
rtcm2_logs = ['test/sample.rtcm2', 'test/sample.rtcm2.chk']
# the log files must be dependencies so they get copied into variant_dir
rtcm_regress = Utility('rtcm-regress', [gpsdecode, rtcm2_logs], [
    '@echo "Testing RTCM decoding..."',
    '@for f in "${SRCDIR}/test/"*.rtcm2; do '
    '    echo "\tTesting $${f}..."; '
    '    TMPFILE=`mktemp -t gpsd-test.chk-XXXXXXXXXXXXXX`; '
    '    "${SRCDIR}/clients/gpsdecode" -u -j <"$${f}" >$${TMPFILE}; '
    '    diff -ub "$${f}".chk $${TMPFILE} || echo "Test FAILED!"; '
    '    rm -f $${TMPFILE}; '
    'done;',
    '@echo "Testing idempotency of JSON dump/decode for RTCM2"',
    '@TMPFILE=`mktemp -t gpsd-test.chk-XXXXXXXXXXXXXX`; '
    '"${SRCDIR}/clients/gpsdecode" -u -e -j <test/synthetic-rtcm2.json '
    ' >$${TMPFILE}; '
    '    grep -v "^#" test/synthetic-rtcm2.json | diff -ub - $${TMPFILE} '
    '    || echo "Test FAILED!"; '
    '    rm -f $${TMPFILE}; ',
])

# Rebuild the RTCM regression tests.
Utility('rtcm-makeregress', [gpsdecode], [
    'for f in "${SRCDIR}/test/"*.rtcm2; do '
    '    "${SRCDIR}/clients/gpsdecode" -j <"$${f}" >"$${f}".chk; '
    'done'
])

# Regression-test the AIVDM decoder.
aivdm_chks = [['test/sample.aivdm', 'test/sample.aivdm.chk', '-u -c'],
              ['test/sample.aivdm', 'test/sample.aivdm.js.chk', '-j'],
              ['test/sample.aivdm', 'test/sample.aivdm.ju.chk', '-u -j'],
              # Parse the unscaled json reference, dump it as unscaled json,
              ['test/sample.aivdm.ju.chk', 'test/sample.aivdm.ju.chk',
               '-u -e -j'],
              # Parse the unscaled json reference, dump it as scaled json,
              ['test/sample.aivdm.ju.chk', 'test/sample.aivdm.js.chk',
               '-e -j'],
              ]
aivdm_regress = None
if env["aivdm"]:
    # the log files must be dependencies so they get copied into variant_dir
    aivdm_tests = []
    aivdm_cnt = 0
    for aivdm_log, aivdm_chk, aivdm_opt in aivdm_chks:
        # oddly this runs in build root, but needs to run in variant_dir
        tgt = Utility(
            'aivdm-regress-%d' % aivdm_cnt,
            [aivdm_log, aivdm_chk, gpsdecode],
            ['@echo "Testing AIVDM decoding w/  %s ..."' % aivdm_opt,
             '"${SRCDIR}/clients/gpsdecode" %s < %s | diff -ub %s -' %
             (aivdm_opt, aivdm_log, aivdm_chk)])
        aivdm_tests.append(tgt)
        aivdm_cnt += 1
    aivdm_regress = env.Alias('aivdm-regress', aivdm_tests)

else:
    announce("AIVDM regression tests suppressed because aivdm is off.")

# Rebuild the AIVDM regression tests.
# Use root dir copies so the new .chk is back into root.
Utility('aivdm-makeregress', [gpsdecode], [
    'for f in "${SRCDIR}/../test/*.aivdm"; do '
    '    "${SRCDIR}/clients/gpsdecode" -u -c <"$${f}" > "$${f}".chk; '
    '    "${SRCDIR}/clients/gpsdecode" -u -j <"$${f}" > "$${f}".ju.chk; '
    '    "${SRCDIR}/clients/gpsdecode" -j  <"$${f}" > "$${f}".js.chk; '
    'done', ])

# Regression-test the packet getter.
packet_regress = UtilityWithHerald(
    'Testing detection of invalid packets...',
    'packet-regress', [test_packet],
    ['"${SRCDIR}/tests/test_packet" | diff -u test/packet.test.chk -', ])

# Rebuild the packet-getter regression test
Utility('packet-makeregress', [test_packet], [
    '"${SRCDIR}/tests/test_packet" > test/packet.test.chk', ])

# Regression-test the geoid and variation tester.
geoid_regress = UtilityWithHerald(
    'Testing the geoid and variation models...',
    'geoid-regress', [test_geoid], ['"${SRCDIR}/tests/test_geoid"'])

# Regression-test the calendar functions
time_regress = Utility('time-regress', [test_mktime], [
    '"${SRCDIR}/tests/test_mktime"'
])

# Regression test the unpacking code in libgps
# the log files must be dependencies so they get copied into variant_dir
clientlib_logs = ['test/clientlib/multipacket.log',
                  'test/clientlib/multipacket.log.chk']
unpack_regress = UtilityWithHerald(
    'Testing the client-library sentence decoder...',
    'unpack-regress', [test_libgps, 'regress-driver', clientlib_logs], [
        '"${SRCDIR}/regress-driver" $REGRESSOPTS -c '
        '"${SRCDIR}/test/clientlib/"*.log', ])
# Unit-test the bitfield extractor
misc_regress = Utility('misc-regress', [
    'tests/test_clienthelpers.py',
    'tests/test_misc.py', ],
    [
    'cd %s; %s tests/test_clienthelpers.py' %
    (variantdir, target_python_path),
    'cd %s; %s tests/test_misc.py' % (variantdir, target_python_path), ])


# Build the regression test for the sentence unpacker
Utility('unpack-makeregress', [test_libgps], [
    '@echo "Rebuilding the client sentence-unpacker tests..."',
    '"${SRCDIR}/regress-driver" $REGRESSOPTS -c -b '
    '"${SRCDIR}/test/clientlib/*.log"'
])

# Unit-test the JSON parsing
if env['socket_export']:
    json_regress = Utility('json-regress', [test_json],
                           ['"${SRCDIR}/tests/test_json"'])
else:
    json_regress = None

# Unit-test timespec math
timespec_regress = Utility('timespec-regress', [test_timespec], [
    '"${SRCDIR}/tests/test_timespec"'
])

# Unit-test float math
float_regress = Utility('float-regress', [test_float], [
    '"${SRCDIR}/tests/test_float"'
])

# Unit-test trig math
trig_regress = Utility('trig-regress', [test_trig], [
    '"${SRCDIR}/tests/test_trig"'
])

# consistency-check the driver methods
method_regress = UtilityWithHerald(
    'Consistency-checking driver methods...',
    'method-regress', [test_packet], [
        '"${SRCDIR}/tests/test_packet" -c >/dev/null', ])

# Test the xgps/xgpsspeed dependencies
if env['xgps_deps']:
    test_xgps_deps = UtilityWithHerald(
        'Testing xgps/xgpsspeed dependencies (since xgps=yes)...',
        'test-xgps-deps', ['${SRCDIR}/tests/test_xgps_deps.py'], [
            '$PYTHON "${SRCDIR}/tests/test_xgps_deps.py"'])
else:
    test_xgps_deps = None

# Run all normal regression tests
describe = UtilityWithHerald(
    'Run normal regression tests for %s...' % gpsd_revision.strip(),
    'describe', [], [])

# Delete all test programs
testclean = Utility('testclean', [], 'rm -fr %s/tests' % variantdir)

test_nondaemon = [
    aivdm_regress,
    bits_regress,
    deg_regress,
    describe,
    float_regress,
    geoid_regress,
    json_regress,
    matrix_regress,
    method_regress,
    mib_regress,
    packet_regress,
    rtcm_regress,
    test_xgps_deps,
    time_regress,
    timespec_regress,
    # trig_regress,  # not ready
]
if env['python']:
    test_nondaemon.append(misc_regress)
    test_nondaemon.append(python_compilation_regress)
    test_nondaemon.append(python_versions)
    test_nondaemon.append(unpack_regress)

if env['socket_export']:
    test_nondaemon.append(test_json)
if env['libgpsmm']:
    test_nondaemon.append(test_gpsmm)
if qt_env:
    test_nondaemon.append(test_qgpsmm)

test_quick = test_nondaemon + [gpsfake_tests]
test_noclean = test_quick + [nmea2000_regress, gps_regress]

env.Alias('test-nondaemon', test_nondaemon)
env.Alias('test-quick', test_quick)
check = env.Alias('check', test_noclean)
env.Alias('testregress', check)
env.Alias('build-tests', testprogs)
build_all = env.Alias('build-all', build + testprogs)

# Remove all shared-memory segments.  Normally only needs to be run
# when a segment size changes.
shmclean = Utility('shmclean', [], ["ipcrm  -M 0x4e545030;"
                                    "ipcrm  -M 0x4e545031;"
                                    "ipcrm  -M 0x4e545032;"
                                    "ipcrm  -M 0x4e545033;"
                                    "ipcrm  -M 0x4e545034;"
                                    "ipcrm  -M 0x4e545035;"
                                    "ipcrm  -M 0x4e545036;"
                                    "ipcrm  -M 0x47505345;"
                                    ])

# The website directory
#
# None of these productions are fired by default.
# The content they handle is the GPSD website, not included in
# release tarballs.

# Documentation

# Paste 'scons --quiet validation-list' to a batch validator such as
# http://htmlhelp.com/tools/validator/batch.html.en


def validation_list(target, source, env):
    for page in glob.glob("www/*.html"):
        if '-head' not in page:
            fp = open(page)
            if "Valid HTML" in fp.read():
                print(os.path.join(website, os.path.basename(page)))
            fp.close()


Utility("validation-list", [www], validation_list)

# Experimenting with pydoc.  Not yet fired by any other productions.
# scons www/ dies with this

# # if env['python']:
# #     env.Alias('pydoc', "www/pydoc/index.html")
# #
# #     # We need to run epydoc with the Python version the modules built for.
# #     # So we define our own epydoc instead of using /usr/bin/epydoc
# #     EPYDOC = "python -c 'from epydoc.cli import cli; cli()'"
# #     env.Command('www/pydoc/index.html', python_progs + glob.glob("*.py")
# #                 + glob.glob("gps/*.py"), [
# #         'mkdir -p www/pydoc',
# #         EPYDOC + " -v --html --graph all -n GPSD $SOURCES -o www/pydoc",
# #             ])

# Productions for setting up and performing udev tests.
#
# Requires root. Do "udev-install", then "tail -f /var/log/syslog" in
# another window, then run 'scons udev-test', then plug and unplug the
# GPS ad libitum.  All is well when you get fix reports each time a GPS
# is plugged in.
#
# In case you are a systemd user you might also need to watch the
# journalctl output. Instead of the hotplug script the gpsdctl@.service
# unit will handle hotplugging together with the udev rules.
#
# Note that a udev event can be triggered with an invocation like:
# udevadm trigger --sysname-match=ttyUSB0 --action add

if env['systemd']:
    systemdinstall_target = [env.Install(DESTDIR + env['unitdir'],
                             "systemd/%s" % (x,)) for x in
                             ("gpsdctl@.service", "gpsd.service",
                              "gpsd.socket")]
    systemd_install = env.Alias('systemd_install', systemdinstall_target)
    systemd_uninstall = env.Command(
        'systemd_uninstall', '',
        Flatten(Uninstall(Alias("systemd_install"))) or "")

    env.AlwaysBuild(systemd_uninstall)
    env.Precious(systemd_uninstall)
    hotplug_wrapper_install = []
else:
    hotplug_wrapper_install = [
        'cp "${SRCDIR}/../gpsd.hotplug" ' + DESTDIR + env['udevdir'],
        'chmod a+x ' + DESTDIR + env['udevdir'] + '/gpsd.hotplug'
    ]

udev_install = Utility('udev-install', 'install', [
    'mkdir -p ' + DESTDIR + env['udevdir'] + '/rules.d',
    'cp "${SRCDIR}/gpsd.rules" ' + DESTDIR + env['udevdir'] +
    '/rules.d/25-gpsd.rules', ] + hotplug_wrapper_install)

if env['systemd']:
    env.Requires(udev_install, systemd_install)
    if not env["sysroot"]:
        systemctl_daemon_reload = Utility('systemctl-daemon-reload', '',
                                          ['systemctl daemon-reload || true'])
        env.AlwaysBuild(systemctl_daemon_reload)
        env.Precious(systemctl_daemon_reload)
        env.Requires(systemctl_daemon_reload, systemd_install)
        env.Requires(udev_install, systemctl_daemon_reload)


Utility('udev-uninstall', '', [
    'rm -f %s/gpsd.hotplug' % env['udevdir'],
    'rm -f %s/rules.d/25-gpsd.rules' % env['udevdir'],
])

Utility('udev-test', '',
        ['"${SRCDIR}/gpsd/gpsd" -N -n -F /var/run/gpsd.sock -D 5', ])

# Default targets

if not cleaning:
    # FIXME: redundant?
    env.Default(build)

# Tags for Emacs and vi
misc_sources = ['clients/cgps.c',
                'clients/gps2udp.c',
                'clients/gpsdctl.c',
                'clients/gpsdecode.c',
                'clients/gpspipe.c',
                'clients/gpxlogger.c',
                'clients/ntpshmmon.c',
                'clients/ppscheck.c',
                'gpsctl.c',
                ]
sources = libgpsd_sources + libgps_sources + gpsd_sources + gpsmon_sources + \
    misc_sources
env.Command('#TAGS', sources, ['etags ' + " ".join(sources)])

# Release machinery begins here
#
# We need to be in the actual project repo (i.e. not doing a -Y build)
# for these productions to work.

distfiles.sort()

# remove git and CI stuff from files to tar/zip
distfiles_ignore = [
    ".ci-build/build.sh",
    ".ci-build/test_options.sh",
    ".gitignore",
    ".gitlab-ci.yml",
    ".travis.yml",
    # remove contrib/ais-samples
    "contrib/ais-samples/ais-nmea-sample.log",
    "contrib/ais-samples/ais-nmea-sample.log.chk", ]
for fn in distfiles_ignore:
    if fn in distfiles:
        distfiles.remove(fn)

# tar balls do not need all generated files

# tar balls do need packaging
for f in packing:
    if f not in distfiles:
        # should not be in git, generated file, we need it
        distfiles.append(f)

# zip archive
target = '#gpsd-${VERSION}.zip'
dozip = env.Zip(target, distfiles)
ziptgt = env.Alias('zip', dozip)

if have_tar:
    target = '#gpsd-%s.tar' % gpsd_version
    # .tar.gz archive
    gzenv = Environment(TARFLAGS='-c -z')
    targz = gzenv.Tar(target + '.gz', distfiles)
    # .tar.xz archive
    xzenv = Environment(TARFLAGS='-c -J')
    tarxz = xzenv.Tar(target + '.xz', distfiles)
    env.Alias('tar', [targz, tarxz])
    env.Alias('dist', [ziptgt, targz, tarxz])

    Clean('build', [targz, tarxz, dozip])

    # Make sure build-from-tarball works.

    # Use possibly nonstandard name for scons
    scons_cmd = [scons_executable_name]

    # Inherit selected options from this scons run
    if GetOption('silent'):
        scons_cmd.append('-s')
    if GetOption('no_progress'):  # Undocumented name
        scons_cmd.append('-Q')
    njobs = GetOption('num_jobs')
    if njobs != 1:
        scons_cmd.append('-j%d' % njobs)

    testbuild = Utility('testbuild', [targz], [
        'rm -Rf testbuild',
        'mkdir testbuild',
        'cd testbuild;'
        'pwd;'
        '${TAR} -xzvf ../gpsd-${VERSION}.tar.gz;'
        'cd gpsd-${VERSION}; %s;' % ' '.join(scons_cmd),
    ])

    releasecheck = env.Alias('releasecheck', [
        testbuild,
        check,
        audits,
    ])

# The chmod copes with the fact that scp will give a
# replacement the permissions of the *original*...
upload_release = Utility('upload-release', ['dist'], [
    'rm -f gpsd-*.sig',
    'gpg -b gpsd-${VERSION}.tar.gz',
    'gpg -b gpsd-${VERSION}.tar.xz',
    'gpg -b gpsd-${VERSION}.zip',
    'chmod ug=rw,o=r gpsd-${VERSION}.tar.* gpsd-${VERSION}.zip',
    'scp gpsd-${VERSION}.tar.* gpsd-${VERSION}.zip* ' + scpupload,
])
env.Alias('upload_release', upload_release)  # For '_' vs. '-'

# How to tag a release
tag_release = Utility('tag-release', [], [
    'git tag -s -m "Tagged for external release ${VERSION}" \
     release-${VERSION}'])
env.Alias('tag_release', tag_release)  # For '_' vs. '-'

upload_tags = Utility('upload-tags', [], ['git push --tags'])
env.Alias('upload_tags', upload_tags)  # For '_' vs. '-'


# Local release preparation. This production will require Internet access,
# but it doesn't do any uploads or public repo mods.
#
# Note that tag_release has to fire early, otherwise the value of REVISION
# won't be right when gpsd_config.h is generated for the tarball.
# FIXME: this is confused
releaseprep = env.Alias("releaseprep",
                        [Utility("distclean", [],
                         ["rm -f include/gpsd_config.h"]),
                         tag_release,
                         'dist'])

# How to update the website.  Assumes a local GitLab pages setup.
# See "pages:" in .gitlab-ci.yml
www_dest = os.environ.get('WEBSITE', '.public')
website = Utility("website", www,
                  'rsync --exclude="*.in" -avz buildtmp/www/ %s ' % www_dest)

# All a buildup to this.
env.Alias("release", [releaseprep,
                      upload_release,
                      upload_tags,
                      website])

# Undo local release preparation
undoprep = Utility("undoprep", [],
                   ['rm -f gpsd-${VERSION}.tar.?z',
                    'rm -f gpsd-$VERSION}.zip',
                    'git tag -d release-${VERSION};'])

#######
# start Debian stuff
#######

# Make RPM from the specfile in packaging
# untested
dist_rpm = Utility('dist-rpm', 'dist', 'rpmbuild -ta gpsd-${VERSION}.tar.gz')
env.Pseudo(dist_rpm)            # mark as fake target.
env.Alias('distrpm', dist_rpm)  # For '_' vs. '-'

# Experimental release mechanics using shipper
# This will ship a freecode metadata update
# untested
ship = Utility("ship", ['dist', "control"],
               ['cd %s; shipper version=%s | sh -e -x' %
                (variantdir, gpsd_version)])
#######
# end Debian stuff
#######


# Release machinery ends here

# The following sets edit modes for GNU EMACS
# Local Variables:
# mode:python
# End:
# vim: set expandtab shiftwidth=4
