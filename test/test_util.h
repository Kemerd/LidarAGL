/**
 * @file    test_util.h
 * @brief   Zero-dependency assertion macros for the host unit tests.
 *
 * @details No test framework, no allocation — just a couple of macros that
 *          print a clear PASS/FAIL line and bump global counters. Each test
 *          file defines its own main() and calls TEST_SUMMARY() at the end.
 */
#ifndef LIDARAGL_TEST_UTIL_H
#define LIDARAGL_TEST_UTIL_H

#include <stdio.h>
#include <math.h>

/* Global pass/fail tallies, defined once per test executable via TEST_GLOBALS. */
extern int g_tests_run;
extern int g_tests_failed;
#define TEST_GLOBALS int g_tests_run = 0; int g_tests_failed = 0;

/* Boolean assertion. */
#define ASSERT_TRUE(cond, msg)                                                 \
    do {                                                                       \
        ++g_tests_run;                                                         \
        if (cond) {                                                            \
            printf("  [PASS] %s\n", (msg));                                    \
        } else {                                                               \
            ++g_tests_failed;                                                  \
            printf("  [FAIL] %s  (%s:%d)\n", (msg), __FILE__, __LINE__);       \
        }                                                                      \
    } while (0)

/* Floating-point near-equality assertion within an absolute tolerance. */
#define ASSERT_NEAR(actual, expected, tol, msg)                                \
    do {                                                                       \
        ++g_tests_run;                                                         \
        double _a = (double)(actual);                                          \
        double _e = (double)(expected);                                        \
        if (fabs(_a - _e) <= (double)(tol)) {                                  \
            printf("  [PASS] %s\n", (msg));                                    \
        } else {                                                               \
            ++g_tests_failed;                                                  \
            printf("  [FAIL] %s  expected %g got %g  (%s:%d)\n",               \
                   (msg), _e, _a, __FILE__, __LINE__);                         \
        }                                                                      \
    } while (0)

/* Print the tally and yield a process exit code (0 == all passed). */
#define TEST_SUMMARY()                                                         \
    do {                                                                       \
        printf("\n%s: %d run, %d failed\n",                                    \
               (g_tests_failed == 0) ? "OK " : "ERR",                          \
               g_tests_run, g_tests_failed);                                   \
    } while (0)

#define TEST_EXIT_CODE() (g_tests_failed == 0 ? 0 : 1)

#endif /* LIDARAGL_TEST_UTIL_H */
