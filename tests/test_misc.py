#!/usr/bin/env python
#
# This code runs compatibly under Python 2 and 3.x for x >= 2.
# Preserve this property!

"""Test gps/misc.py."""

from __future__ import absolute_import, print_function, division

import sys

import gps.misc

errors = 0
# values from here: https://en.wikipedia.org/wiki/Decimal_degrees#Precision
# Note the wikipedia numbers are NOT ellipsoid corrected:
# EarthDistanceSmall() is ellipsoid corrected.
tests = [
    # slow convergence
    (0.00000000, 0.00000000, 0.50000000, 179.5000000, 19936288),
    # failure to converge, happens at antipodal points
    # fall back to EarthDistanceSmall()
    # the real answer should be 19944127
    (0.00000000, 0.00000000, 0.50000000, 179.7000000, 19870094),
    # boston to new york
    (42.3541165, -71.0693514, 40.7791472, -73.9680804, 298396),
    # equator 10 degree
    (0.00000000, 0.00000000, 10.00000000, 0.00000000, 1105854),
    (0.00000000, 0.00000000, 0.00000000, 10.00000000, 1113194),
    (0.00000000, 0.00000000, 10.00000000, 10.00000000, 1565109),
    # 23N 10 degree
    (23.00000000, 0.00000000, 33.00000000, 0.00000000, 1108210),
    (23.00000000, 0.00000000, 23.00000000, 10.00000000, 1025026),
    (23.00000000, 0.00000000, 33.00000000, 10.00000000, 1480049),
    # 45N 10 degree
    (45.00000000, 0.00000000, 55.00000000, 0.00000000, 1112285),
    (45.00000000, 0.00000000, 45.00000000, 10.00000000, 787967),
    (45.00000000, 0.00000000, 55.00000000, 10.00000000, 1320437),
    # 67N 10 degree
    (67.00000000, 0.00000000, 77.00000000, 0.00000000, 1115839),
    (67.00000000, 0.00000000, 67.00000000, 10.00000000, 435729),
    (67.00000000, 0.00000000, 77.00000000, 10.00000000, 1164036),
    # equator 5 degree
    (0.00000000, 0.00000000, 5.00000000, 0.00000000, 552885),
    (0.00000000, 0.00000000, 0.00000000, 5.00000000, 556597),
    (0.00000000, 0.00000000, 5.00000010, 5.00000000, 784028),
    # 23N 5 degree
    (23.00000000, 0.00000000, 28.00000000, 0.00000000, 553904),
    (23.00000000, 0.00000000, 23.00000000, 5.00000000, 512587),
    (23.00000000, 0.00000000, 28.00000000, 5.00000000, 747800),
    # 45N 5 degree
    (45.00000000, 0.00000000, 50.00000000, 0.00000000, 555902),
    (45.00000000, 0.00000000, 45.00000000, 5.00000000, 394171),
    (45.00000000, 0.00000000, 50.00000000, 5.00000000, 671179),
    # 67N 5 degree
    (67.00000000, 0.00000000, 72.00000000, 0.00000000, 557775),
    (67.00000000, 0.00000000, 67.00000000, 5.00000000, 218040),
    (67.00000000, 0.00000000, 72.00000000, 5.00000000, 590565),
    # equator 1 degree
    (0.00000000, 0.00000000, 1.00000000, 0.00000000, 110574),
    (0.00000000, 0.00000000, 0.00000000, 1.00000000, 111319),
    (0.00000000, 0.00000000, 1.00000010, 1.00000000, 156899),
    # 23N 1 degree
    (23.00000000, 0.00000000, 24.00000000, 0.00000000, 110751),
    (23.00000000, 0.00000000, 23.00000000, 1.00000000, 102522),
    (23.00000000, 0.00000000, 24.00000000, 1.00000000, 150659),
    # 45N 1 degree
    (45.00000000, 0.00000000, 46.00000000, 0.00000000, 111141),
    (45.00000000, 0.00000000, 45.00000000, 1.00000000,  78846),
    (45.00000000, 0.00000000, 46.00000000, 1.00000000, 135869),
    # 67N 1 degree
    (67.00000000, 0.00000000, 68.00000000, 0.00000000, 111528),
    (67.00000000, 0.00000000, 67.00000000, 1.00000000,  43619),
    (67.00000000, 0.00000000, 68.00000000, 1.00000000, 119427),
    # equator 10e-7
    (0.00000000, 0.00000000, 0.00000010, 0.00000000, 0.011057),
    (0.00000000, 0.00000000, 0.00000000, 0.00000010, 0.011132),
    (0.00000000, 0.00000000, 0.00000010, 0.00000010, 0.015690),
    # 23N 10e-7
    (23.00000000, 0.00000000, 23.00000010, 0.00000000, 0.011074),
    (23.00000000, 0.00000000, 23.00000000, 0.00000010, 0.010252),
    (23.00000000, 0.00000000, 23.00000010, 0.00000010, 0.015091),
    # 45N 10e-7
    (45.00000000, 0.00000000, 45.00000010, 0.00000000, 0.011113),
    (45.00000000, 0.00000000, 45.00000000, 0.00000010, 0.007885),
    (45.00000000, 0.00000000, 45.00000010, 0.00000010, 0.013626),
    # 67N 10e-7
    (67.00000000, 0.00000000, 67.00000010, 0.00000000, 0.011152),
    (67.00000000, 0.00000000, 67.00000000, 0.00000010, 0.004360),
    (67.00000000, 0.00000000, 67.00000010, 0.00000010, 0.011974),
]

# EarthDistanceSmall
for (lat1, lon1, lat2, lon2, dist) in tests:
    distance = gps.misc.EarthDistanceSmall((lat1, lon1), (lat2, lon2))
    # compare to 1%
    diff = dist - distance
    max_diff = dist * 0.01
    if abs(diff) > max_diff:
        sys.stderr.write(
            "misc small: %.8f %.8f, %.8f %.8f, expected %.7f got %.7f\n"
            % (lat1, lon1, lat2, lon2, dist, distance))
        errors += 1

# EarthDistance
for (lat1, lon1, lat2, lon2, dist) in tests:
    distance = gps.misc.EarthDistance((lat1, lon1), (lat2, lon2))
    # compare to 0.001%
    diff = dist - distance
    max_diff = dist * 0.00001
    if abs(diff) > max_diff:
        sys.stderr.write(
            "misc large: %.8f %.8f, %.8f %.8f, expected %.7f got %.7f\n"
            % (lat1, lon1, lat2, lon2, dist, distance))
        errors += 1

tests1 = [
    # posix, leap, gps_time, gps_week, gps_tow
    (315964800, 0, 0, 0, 0),   # Start of gps epoch
    (935280000, 13, 619315213, 1024, 13),   # 1999-08-22T00:00:00
    (1609459200, 18, 1293494418, 2138, 432018),   # 1/Jan/2021 0:0:0
]

for (posix, leap, gps_time, gps_week, gps_tow) in tests1:
    (gpstime, gpsweek, gpstow) = gps.posix2gps(posix, leap)
    if ((gps_time != gpstime or
         gps_week != gpsweek or
         gps_tow != gpstow)):
        sys.stderr.write(
           "posix2gps(%d, %d) = (%d, %d, %d) s/b (%d, %d, %d)\n" %
           (posix, leap, gpstime, gpsweek, gpstow,
            gps_time, gps_week, gps_tow))
        errors += 1

if errors:
    print("test_misc.py: failed")
    sys.exit(1)
else:
    print("test_misc.py: OK")
    sys.exit(0)
