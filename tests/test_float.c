/*
 * Copyright 2006 Chris Kuethe <chris.kuethe@gmail.com>
 *
 * This file is Copyright 2010 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */
#include <stdio.h>        // for puts(), printf()
#include <string.h>       // for strncmp()

/*
 * this simple program tests to see whether your system can do proper
 * single and double precision floating point. This is apparently Very
 * Hard To Do(tm) on embedded systems, judging by the number of broken
 * ARM toolchains I've seen... :(
 *
 * Added in 2015 by ESR: Test for C99 behavior on negative operand(s)
 * of %, that is the result should have the sign of the left operand.
 *
 * Added in 2021 by GEM, Test for printf() rounding
 *
 * compile with: gcc -O -o test_float test_float.c
 *     (use whatever -O level you like)
 */

static int test_single(void) {
	static float f;
	static int i;
	static int e = 0;

	/* addition test */
	f = 1.0;
	for(i = 0; i < 10; i++)
		f += (1<<i);
	if (f != 1024.0) {
		puts("s1 ");
		e++;
	}

	/* subtraction test */
	f = 1024.0;
	for(i = 0; i < 10; i++)
		f -= (1<<i);
	if (f != 1.0) {
		puts("s2 ");
		e++;
	}

	/* multiplication test */
	f = 1.0;
	for(i = 1; i < 10; i++)
		f *= i;
	if (f != 362880.0) {
		puts("s3 ");
		e++;
	}

	/* division test */
	f = 362880.0;
	for(i = 1; i < 10; i++)
		f /= i;
	if (f != 1.0) {
		puts("s4 ");
		e++;
	}

	/* multiply-accumulate test */
	f = 0.5;
	for(i = 1; i < 1000000; i++) {
		f += 2.0;
		f *= 0.5;
	}
	if (f != 2.0) {
		puts("s5 ");
		e++;
	}

	/* divide-subtract test */
	f = 2.0;
	for(i = 1; i < 1000000; i++) {
		f /= 0.5;
		f -= 2.0;
	}
	if (f != 2.0) {
		puts("s6 ");
		e++;
	}

	/* add-multiply-subtract-divide test */
	f = 1000000.0;
	for(i = 1; i < 1000000; i++)
		f = ((((f + 1.5) * 0.5) - 1.25) / 0.5);
	if (f != 1.0) {
		puts("s7 ");
		e++;
	}

	/* multiply-add-divide-subtract test */
	f = 1.0;
	for(i = 1; i < 1000000; i++)
		f = ((((f * 5.0) + 3.0) / 2.0) - 3.0);
	if (f != 1.0)
		puts("s8 ");

	/* subtract-divide-add-multiply test */
	f = 8.0;
	for(i = 1; i < 1000000; i++)
		f = ((((f - 5.0) / 2.0) + 2.5) * 2.0);
	if (f != 8.0) {
		puts("s9 ");
		e++;
	}

	/* divide-subtract-multiply-add test */
	f = 42.0;
	for(i = 1; i < 1000000; i++)
		f = ((((f / 6.0) - 5.0) * 19.75 ) + 2.5);
	if (f != 42.0) {
		puts("s10 ");
		e++;
	}
	if (e) {
		puts("\n");
	}
	return e;
}

static int test_double(void) {
	static double f;
	static int i;
	static int e = 0;

	/* addition test */
	f = 1.0;
	for(i = 0; i < 10; i++)
		f += (1<<i);
	if (f != 1024.0) {
		puts("d1 ");
		e++;
	}

	/* subtraction test */
	f = 1024.0;
	for(i = 0; i < 10; i++)
		f -= (1<<i);
	if (f != 1.0) {
		puts("d2 ");
		e++;
	}

	/* multiplication test */
	f = 1.0;
	for(i = 1; i < 10; i++)
		f *= i;
	if (f != 362880.0) {
		puts("d3 ");
		e++;
	}

	/* division test */
	f = 362880.0;
	for(i = 1; i < 10; i++)
		f /= i;
	if (f != 1.0) {
		puts("d4 ");
		e++;
	}

	/* multiply-accumulate test */
	f = 0.5;
	for(i = 1; i < 1000000; i++) {
		f += 2.0;
		f *= 0.5;
	}
	if (f != 2.0) {
		puts("d5 ");
		e++;
	}

	/* divide-subtract test */
	f = 2.0;
	for(i = 1; i < 1000000; i++) {
		f /= 0.5;
		f -= 2.0;
	}
	if (f != 2.0) {
		puts("d6 ");
		e++;
	}

	/* add-multiply-subtract-divide test */
	f = 1000000.0;
	for(i = 1; i < 1000000; i++)
		f = ((((f + 1.5) * 0.5) - 1.25) / 0.5);
	if (f != 1.0) {
		puts("d7 ");
		e++;
	}

	/* multiply-add-divide-subtract test */
	f = 1.0;
	for(i = 1; i < 1000000; i++)
		f = ((((f * 5.0) + 3.0) / 2.0) - 3.0);
	if (f != 1.0)
		puts("d8 ");

	/* subtract-divide-add-multiply test */
	f = 8.0;
	for(i = 1; i < 1000000; i++)
		f = ((((f - 5.0) / 2.0) + 2.5) * 2.0);
	if (f != 8.0) {
		puts("d9 ");
		e++;
	}

	/* divide-subtract-multiply-add test */
	f = 42.0;
	for(i = 1; i < 1000000; i++)
		f = ((((f / 6.0) - 5.0) * 19.75 ) + 2.5);
	if (f != 42.0) {
		puts("d10 ");
		e++;
	}
	if (e) {
		puts("\n");
	}
	return e;
}

static int test_modulo(void) {
    int e = 0;

    /* make sure that gcc does not optimize these away */
    volatile int a;
    volatile int b;

    a = -5;
    b = 2;
    //cppcheck-suppress knownConditionTrueFalse
    if (a % b != -1) {
	puts("m1 ");
	e++;
    }

    a = -5;
    b = -2;
    //cppcheck-suppress knownConditionTrueFalse
    if (a % b != -1) {
	puts("m2 ");
	e++;
    }

    a = 5;
    b = -2;
    //cppcheck-suppress knownConditionTrueFalse
    if (a % b != 1) {
	puts("m3 ");
	e++;
    }

    if (e) {
	puts("\n");
    }
    return e;
}

struct printf_test
{
    double d;
    const char *expected;
};
struct printf_test printf_test_tests[] = {
    {-0.0015 - 1e-10, "-0.002"},
    {-0.0015, "-0.002"},
    {-0.0015 + 1e-10, "-0.001"},
    {-0.0005 - 1e-10, "-0.001"},
    {-0.0005, "-0.001"},
    {-0.0005 + 1e-10, "-0.000"},
    {0.0005 - 1e-10, "0.000"},
    {0.0005, "0.001"},
    {0.0005 + 1e-10, "0.001"},
    {0.0015 - 1e-10, "0.001"},
    {0.0015, "0.002"},
    {0.0015 + 1e-10, "0.002"},
    {0, NULL},
};

// POSIX just says "round", not which of the 4 possible POSIX rounding modes.
static int test_printf(void)
{
    int e = 0;
    char check[20];
    int test_num;

    for (test_num = 0; NULL != printf_test_tests[test_num].expected;
         test_num++) {
        snprintf(check, sizeof(check), "%.3f", printf_test_tests[test_num].d);
        if (0 == strncmp(printf_test_tests[test_num].expected,
                         check, sizeof(check))) {
            continue;
        }
        if (0 == strncmp("0.000", check, sizeof(check)) &&
            0 == strncmp("-0.000", printf_test_tests[test_num].expected,
                         sizeof(check))) {
            // special case 0.000 ok for -0.000
            continue;
        }
        printf("p%d expected %s got %s\n", test_num,
               printf_test_tests[test_num].expected, check);
        e++;
    }

    return e;
}

int main(void) {
    int errcnt = 0;

    if (0 != test_single()) {
        puts("WARNING: Single-precision floating point math might be broken\n");
        errcnt++;
    }

    if (0 != test_double()) {
        puts("WARNING: Double-precision floating point math might be broken\n");
        errcnt++;
    }

    if (0 != test_modulo()) {
        puts("WARNING: Modular arithmetic is broken\n");
        errcnt++;
    }

    if (0 != test_printf()) {
        puts("WARNING: printf() rounding is broken\n");
        errcnt++;
    }

    if (0 == errcnt) {
        puts("floating point and modular math appears to work\n");
    }
    return errcnt;
}
// vim: set expandtab shiftwidth=4
