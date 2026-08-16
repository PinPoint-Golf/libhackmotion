/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hm_test.h — a ~150-line test framework, vendored on purpose.
 *
 * libhackmotion links nothing but libm, and the test suite inherits that: a
 * consumer should be able to build and run every test on a machine with no
 * package manager, no radio and no sensor.  A test dependency is still a
 * dependency.
 *
 *   HM_TEST(name)          declare a test
 *   HM_ASSERT(cond)        fail and continue to the next test
 *   HM_ASSERT_EQ(a, b)     integer equality, prints both sides
 *   HM_ASSERT_NEAR(a,b,t)  floating point within tolerance
 *   HM_ASSERT_STR(a, b)    string equality
 *   HM_TEST_MAIN()         run everything registered in this translation unit
 */
#ifndef HM_TEST_H
#define HM_TEST_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct hm_test_case {
    const char *name;
    void (*fn)(void);
} hm_test_case;

#ifndef HM_TEST_MAX
#define HM_TEST_MAX 256
#endif

static hm_test_case hm_test_cases[HM_TEST_MAX];
static int hm_test_count = 0;
static int hm_test_failures = 0;
static int hm_test_current_failed = 0;
static int hm_test_assertions = 0;

static void hm_test_register(const char *name, void (*fn)(void))
{
    /*
     * ⚠ ABORT RATHER THAN DROP.  This used to silently ignore anything past
     * HM_TEST_MAX, so passing the cap would have retired tests one at a time
     * with the suite still reporting green — a check that has stopped running,
     * which is the failure this library is built around and the one its own
     * review (I1) walked into.  test_session.c is already at 89 of 256.
     */
    if (hm_test_count >= HM_TEST_MAX) {
        fprintf(stderr, "hm_test: more than %d cases; '%s' would be DROPPED\n", HM_TEST_MAX,
                name);
        abort();
    }
    hm_test_cases[hm_test_count].name = name;
    hm_test_cases[hm_test_count].fn = fn;
    hm_test_count++;
}

#define HM_TEST(name)                                                                        \
    static void name(void);                                                                  \
    __attribute__((constructor)) static void hm_register_##name(void)                        \
    {                                                                                        \
        hm_test_register(#name, name);                                                       \
    }                                                                                        \
    static void name(void)

#define HM_FAIL_(fmt, ...)                                                                   \
    do {                                                                                     \
        hm_test_current_failed = 1;                                                          \
        fprintf(stderr, "    FAIL %s:%d: " fmt "\n", __FILE__, __LINE__, __VA_ARGS__);        \
    } while (0)

#define HM_ASSERT(cond)                                                                      \
    do {                                                                                     \
        hm_test_assertions++;                                                                \
        if (!(cond)) {                                                                       \
            HM_FAIL_("%s", #cond);                                                           \
        }                                                                                    \
    } while (0)

#define HM_ASSERT_MSG(cond, msg)                                                             \
    do {                                                                                     \
        hm_test_assertions++;                                                                \
        if (!(cond)) {                                                                       \
            HM_FAIL_("%s  (%s)", #cond, (msg));                                              \
        }                                                                                    \
    } while (0)

#define HM_ASSERT_EQ(a, b)                                                                   \
    do {                                                                                     \
        long long va_ = (long long)(a);                                                      \
        long long vb_ = (long long)(b);                                                      \
        hm_test_assertions++;                                                                \
        if (va_ != vb_) {                                                                    \
            HM_FAIL_("%s == %s  (%lld != %lld)", #a, #b, va_, vb_);                           \
        }                                                                                    \
    } while (0)

#define HM_ASSERT_NEAR(a, b, tol)                                                            \
    do {                                                                                     \
        double va_ = (double)(a);                                                            \
        double vb_ = (double)(b);                                                            \
        double t_ = (double)(tol);                                                           \
        hm_test_assertions++;                                                                \
        if (!(fabs(va_ - vb_) <= t_)) {                                                      \
            HM_FAIL_("%s ~= %s  (%.9g vs %.9g, tol %.9g, diff %.9g)", #a, #b, va_, vb_, t_,   \
                     fabs(va_ - vb_));                                                       \
        }                                                                                    \
    } while (0)

#define HM_ASSERT_STR(a, b)                                                                  \
    do {                                                                                     \
        const char *sa_ = (a);                                                               \
        const char *sb_ = (b);                                                               \
        hm_test_assertions++;                                                                \
        if (sa_ == NULL || sb_ == NULL || strcmp(sa_, sb_) != 0) {                            \
            HM_FAIL_("%s == %s  (\"%s\" != \"%s\")", #a, #b, sa_ ? sa_ : "(null)",            \
                     sb_ ? sb_ : "(null)");                                                  \
        }                                                                                    \
    } while (0)

#define HM_TEST_MAIN()                                                                       \
    int main(int argc, char **argv)                                                          \
    {                                                                                        \
        const char *filter = (argc > 1) ? argv[1] : NULL;                                    \
        int ran = 0;                                                                         \
        for (int i = 0; i < hm_test_count; ++i) {                                            \
            if (filter && strstr(hm_test_cases[i].name, filter) == NULL) {                    \
                continue;                                                                    \
            }                                                                                \
            hm_test_current_failed = 0;                                                      \
            hm_test_cases[i].fn();                                                           \
            ran++;                                                                           \
            if (hm_test_current_failed) {                                                    \
                hm_test_failures++;                                                          \
                printf("[FAIL] %s\n", hm_test_cases[i].name);                                \
            } else {                                                                         \
                printf("[ ok ] %s\n", hm_test_cases[i].name);                                \
            }                                                                                \
        }                                                                                    \
        printf("%s: %d test(s), %d assertion(s), %d failure(s)\n", argv[0], ran,             \
               hm_test_assertions, hm_test_failures);                                        \
        return hm_test_failures == 0 ? 0 : 1;                                                \
    }

#endif /* HM_TEST_H */
