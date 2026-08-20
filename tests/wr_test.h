/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * wr_test.h — a ~150-line test framework, vendored on purpose.
 *
 * libwrist links nothing but libm, and the test suite inherits that: a
 * consumer should be able to build and run every test on a machine with no
 * package manager, no radio and no sensor.  A test dependency is still a
 * dependency.
 *
 *   WR_TEST(name)          declare a test
 *   WR_ASSERT(cond)        fail and continue to the next test
 *   WR_ASSERT_EQ(a, b)     integer equality, prints both sides
 *   WR_ASSERT_NEAR(a,b,t)  floating point within tolerance
 *   WR_ASSERT_STR(a, b)    string equality
 *   WR_TEST_MAIN()         run everything registered in this translation unit
 */
#ifndef WR_TEST_H
#define WR_TEST_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct wr_test_case {
    const char *name;
    void (*fn)(void);
} wr_test_case;

#ifndef WR_TEST_MAX
#define WR_TEST_MAX 256
#endif

static wr_test_case wr_test_cases[WR_TEST_MAX];
static int wr_test_count = 0;
static int wr_test_failures = 0;
static int wr_test_current_failed = 0;
static int wr_test_assertions = 0;

static void wr_test_register(const char *name, void (*fn)(void))
{
    /*
     * ⚠ ABORT RATHER THAN DROP.  This used to silently ignore anything past
     * WR_TEST_MAX, so passing the cap would have retired tests one at a time
     * with the suite still reporting green — a check that has stopped running,
     * which is the failure this library is built around and the one its own
     * review (I1) walked into.  test_session.c is already at 89 of 256.
     */
    if (wr_test_count >= WR_TEST_MAX) {
        fprintf(stderr, "wr_test: more than %d cases; '%s' would be DROPPED\n", WR_TEST_MAX,
                name);
        abort();
    }
    wr_test_cases[wr_test_count].name = name;
    wr_test_cases[wr_test_count].fn = fn;
    wr_test_count++;
}

/*
 * ⚠ AUTO-REGISTRATION IS THE ONE PART OF THIS HARNESS THAT IS NOT PORTABLE C.
 * A test registers itself by running before main(), so that adding a case needs
 * no edit to a list somewhere else — a list is exactly the thing that silently
 * stops covering what it forgot.
 *
 * There is no standard way to do that.  GCC and clang have
 * __attribute__((constructor)); MSVC has no equivalent, so the pointer goes in
 * the CRT's own pre-main initialiser section (.CRT$XCU) instead, which is the
 * mechanism the C runtime itself uses.  Both arrive at the same place.
 *
 * ⚠ THE FAILURE MODE HERE IS SILENT, WHICH IS WHY WR_TEST_MAIN() COUNTS.  If a
 * registration mechanism does not fire — a section renamed, a linker discarding
 * what it thinks is unreferenced — the case simply never runs, and a suite that
 * ran nothing prints no failures and exits 0.  That reads exactly like a suite
 * that passed, which is the shape this project has a rule about.  So the runner
 * refuses to exit 0 having run no cases; the guard is portable even though the
 * mechanism it guards is not.
 */
#if defined(_MSC_VER)
#pragma section(".CRT$XCU", read)
#define WR_TEST_CTOR_(name)                                                                  \
    static void wr_register_##name(void);                                                    \
    __declspec(allocate(".CRT$XCU")) void (*wr_ctor_##name)(void) = wr_register_##name;      \
    static void wr_register_##name(void)
#else
#define WR_TEST_CTOR_(name)                                                                  \
    __attribute__((constructor)) static void wr_register_##name(void)
#endif

#define WR_TEST(name)                                                                        \
    static void name(void);                                                                  \
    WR_TEST_CTOR_(name)                                                                      \
    {                                                                                        \
        wr_test_register(#name, name);                                                       \
    }                                                                                        \
    static void name(void)

#define WR_FAIL_(fmt, ...)                                                                   \
    do {                                                                                     \
        wr_test_current_failed = 1;                                                          \
        fprintf(stderr, "    FAIL %s:%d: " fmt "\n", __FILE__, __LINE__, __VA_ARGS__);        \
    } while (0)

#define WR_ASSERT(cond)                                                                      \
    do {                                                                                     \
        wr_test_assertions++;                                                                \
        if (!(cond)) {                                                                       \
            WR_FAIL_("%s", #cond);                                                           \
        }                                                                                    \
    } while (0)

#define WR_ASSERT_MSG(cond, msg)                                                             \
    do {                                                                                     \
        wr_test_assertions++;                                                                \
        if (!(cond)) {                                                                       \
            WR_FAIL_("%s  (%s)", #cond, (msg));                                              \
        }                                                                                    \
    } while (0)

#define WR_ASSERT_EQ(a, b)                                                                   \
    do {                                                                                     \
        long long va_ = (long long)(a);                                                      \
        long long vb_ = (long long)(b);                                                      \
        wr_test_assertions++;                                                                \
        if (va_ != vb_) {                                                                    \
            WR_FAIL_("%s == %s  (%lld != %lld)", #a, #b, va_, vb_);                           \
        }                                                                                    \
    } while (0)

#define WR_ASSERT_NEAR(a, b, tol)                                                            \
    do {                                                                                     \
        double va_ = (double)(a);                                                            \
        double vb_ = (double)(b);                                                            \
        double t_ = (double)(tol);                                                           \
        wr_test_assertions++;                                                                \
        if (!(fabs(va_ - vb_) <= t_)) {                                                      \
            WR_FAIL_("%s ~= %s  (%.9g vs %.9g, tol %.9g, diff %.9g)", #a, #b, va_, vb_, t_,   \
                     fabs(va_ - vb_));                                                       \
        }                                                                                    \
    } while (0)

#define WR_ASSERT_STR(a, b)                                                                  \
    do {                                                                                     \
        const char *sa_ = (a);                                                               \
        const char *sb_ = (b);                                                               \
        wr_test_assertions++;                                                                \
        if (sa_ == NULL || sb_ == NULL || strcmp(sa_, sb_) != 0) {                            \
            WR_FAIL_("%s == %s  (\"%s\" != \"%s\")", #a, #b, sa_ ? sa_ : "(null)",            \
                     sb_ ? sb_ : "(null)");                                                  \
        }                                                                                    \
    } while (0)

#define WR_TEST_MAIN()                                                                       \
    int main(int argc, char **argv)                                                          \
    {                                                                                        \
        const char *filter = (argc > 1) ? argv[1] : NULL;                                    \
        int ran = 0;                                                                         \
        for (int i = 0; i < wr_test_count; ++i) {                                            \
            if (filter && strstr(wr_test_cases[i].name, filter) == NULL) {                    \
                continue;                                                                    \
            }                                                                                \
            wr_test_current_failed = 0;                                                      \
            wr_test_cases[i].fn();                                                           \
            ran++;                                                                           \
            if (wr_test_current_failed) {                                                    \
                wr_test_failures++;                                                          \
                printf("[FAIL] %s\n", wr_test_cases[i].name);                                \
            } else {                                                                         \
                printf("[ ok ] %s\n", wr_test_cases[i].name);                                \
            }                                                                                \
        }                                                                                    \
        printf("%s: %d test(s), %d assertion(s), %d failure(s)\n", argv[0], ran,             \
               wr_test_assertions, wr_test_failures);                                        \
        /* ⚠ A suite that registered nothing must not report success — see the                \
         * note on WR_TEST_CTOR_.  Without this, a broken registration mechanism             \
         * is indistinguishable from a clean run. */                                         \
        if (wr_test_count == 0) {                                                            \
            fprintf(stderr, "wr_test: NO CASES REGISTERED — the registration "               \
                            "mechanism did not fire; this is a failure, not a pass\n");      \
            return 2;                                                                        \
        }                                                                                    \
        return wr_test_failures == 0 ? 0 : 1;                                                \
    }

#endif /* WR_TEST_H */
