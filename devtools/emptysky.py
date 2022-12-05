#!/usr/bin/env python
"""Hack program to check GPSD .chk files for empty SKY reports."""
from __future__ import print_function

import ast
import os
import sys


class Error(Exception):
  """Base error class."""


class ParseError(Error):
  """Error for parsing error."""


def ProcessFile(name, withname=False,
                verbose=False, quiet=False, quieter=False):
  """Process contents of one file."""
  bad = []
  linenum = 0
  with open(name) as f:
    data = f.readlines()
    for line in data:
      linenum += 1
      sline = line.rstrip()
      rline = sline.replace('false', 'False').replace('true', 'True')
      if not rline.startswith('{'):
        continue
      try:
        parsed = ast.literal_eval(rline)
      except ValueError as exc:
        raise ParseError('line %d' % linenum, exc.message, rline)
      if parsed.get('class') == 'SKY' and not parsed.get('nSat', 0):
        bad.append([linenum, parsed])
  if quieter or quiet and not len(bad):
    return len(bad), linenum
  if withname:
    print(os.path.basename(name) + ': ', len(bad))
  else:
    print(len(bad))
  if verbose:
    for linenum, item in bad:
      print('  %d:  %s' %(linenum, repr(item)))
  return len(bad), linenum


def Plural(num, single='', multi='s'):
  """Get single/plural suffix for count."""
  return single if num == 1 else multi


def Usage(cmd, out):
  """Print usage text."""
  def uprint(msg):
    print(msg, file=out)

  uprint('Usage is: %s [options] <file>...' % os.path.basename(cmd))
  uprint('\tOptions are:')
  uprint('\t-h:\tHelp - print this text')
  uprint("\t-q:\tQuiet - don't list nonoffending file(s)")
  uprint("\t-Q:\tReally quiet - don't list any individual file(s)")
  uprint('\t-t:\tTotals - print totals at end')
  uprint('\t-v:\tVerbose - print each offending line')


def main(argv):
  """main function."""
  args = argv[1:]
  opts_ok = set(list('hqQtv'))
  opts = dict([(o, False) for o in opts_ok])
  while args and args[0].startswith('-'):
    optarg = args[0]
    newopts = set(list(optarg[1:]))
    badopts = newopts - opts_ok
    if badopts:
      print("Bad option%s: '%s' in '%s'"
             % (Plural(len(badopts)), ''.join(sorted(list(badopts))), optarg),
             file=sys.stderr)
      return 3
    opts.update(dict([(o, True) for o in list(newopts)]))
    args = args[1:]
  if opts['h']:
    Usage(argv[0], sys.stdout)
    return 0
  if not args:
    Usage(argv[0], sys.stderr)
    return 2
  withname = len(args) > 1
  badlines = totlines = badfiles = files = 0
  for name in args:
    bad, total = ProcessFile(name, withname=withname,
                             verbose=opts['v'],
                             quiet=opts['q'], quieter=opts['Q'])
    badlines += bad
    totlines += total
    badfiles += 1 if bad else 0
    files += 1
  if opts['t']:
    print('Totals: %d empty of %d SKY line%s in %d of %d file%s'
          % (badlines, totlines, Plural(totlines),
             badfiles, files, Plural(files)))
  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv))
