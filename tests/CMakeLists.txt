# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Mark Liversedge
# Tests link the library's sources directly rather than the built target, so
# they can reach internal headers (wr_codec.h, wr_fit.h, wr_unwrap.h) without
# widening the public API to make them testable.

set(WR_TEST_SOURCES
    test_codec.c
    test_command.c
    test_unwrap.c
    test_clock.c
    test_coverage.c
    test_density.c
    test_quat.c
    test_presence.c
    test_overlap.c
    test_device.c
    test_session.c
)

# The `.wrwire` container lives in a separate target because it opens files; the
# purity gate below is why the core cannot.
if(WR_BUILD_RECORD)
    list(APPEND WR_TEST_SOURCES test_record.c)
endif()

foreach(src ${WR_TEST_SOURCES})
    get_filename_component(name ${src} NAME_WE)
    add_executable(${name} ${src})
    if(name STREQUAL "test_record")
        target_link_libraries(${name} PRIVATE wrist_record wr_build_flags)
    else()
        target_link_libraries(${name} PRIVATE wrist wr_build_flags)
    endif()
    target_include_directories(${name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/src)
    # The vendored harness registers cases with __attribute__((constructor)) and
    # deliberately leaves some helpers unused per translation unit.
    if(NOT MSVC)
        target_compile_options(${name} PRIVATE -Wno-unused-function)
    endif()
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES TIMEOUT 120)
endforeach()

# ⚠ Two implementations of one container format — tools/wr_capture.py writes it
# and record/wr_record.c reads it — pinned against each other.  A drift there
# would produce a capture that only turns out to be unreadable later, by which
# time the sensor and the session are gone.  Skipped, loudly, with no python3.
if(WR_BUILD_TOOLS AND WR_BUILD_RECORD)
    # ⚠ macOS: PREFER THE INTERPRETER ON PATH OVER THE FRAMEWORK ONE, because the
    # framework FindPython3 picks by default is the Command Line Tools' Python
    # 3.9 — a SIGNED PLATFORM BINARY, and that is not a cosmetic difference here.
    # Any sanitizer runtime dlopen()ed into such a process is refused outright
    # (`Sanitizer load violates platform policy`), so a UBSan build's four Python
    # tests fail on the interpreter that was chosen for them rather than on
    # anything in this repository.  A PATH python (Homebrew, python.org, a venv)
    # is not a platform binary and loads it; measured under -DWR_ENABLE_UBSAN=ON,
    # all 23 tests pass there against 19-and-4-failures on the framework one.  It
    # is also an end-of-life 3.9 against whatever the developer actually runs.
    # No effect off Apple: FRAMEWORK is only consulted on macOS.
    if(APPLE AND NOT DEFINED Python3_FIND_FRAMEWORK)
        set(Python3_FIND_FRAMEWORK LAST)
    endif()
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
        add_test(NAME capture_format
                 COMMAND ${Python3_EXECUTABLE}
                         ${CMAKE_CURRENT_SOURCE_DIR}/test_capture_format.py
                         $<TARGET_FILE:wrwire>)
        # See the note further down: cp1252 stdout on Windows cannot encode this
        # project's own report glyphs.
        set_tests_properties(capture_format PROPERTIES ENVIRONMENT
                             "PYTHONIOENCODING=utf-8")
    else()
        message(STATUS "no python3: the capture-harness format test will not run")
    endif()
endif()

# ---------------------------------------------------------------------------
# The Python binding
#
# ⚠ SAME SHAPE AGAIN, ONE REGISTER UP.  python/wrist/_types.py declares
# every public struct and enum a SECOND time, in ctypes.  wr_abi_check() guards
# nine struct SIZES at load and nothing else — not one field offset, not one
# enumerator — so a `skew_us` one slot out decodes every sample into plausible
# nonsense with no error anywhere.
#
# `wrwire abi` reads the real layout out of the compiler and this compares the
# two.  It found two genuine transcription errors on the day it was written:
# wr_gap_kind's first two members were the wrong way round, and four of eleven
# history statuses were on the wrong numbers.
# ---------------------------------------------------------------------------
if(WR_BUILD_TOOLS AND WR_BUILD_RECORD AND WR_BUILD_FFI AND Python3_Interpreter_FOUND)
    # ⚠ The .so is passed explicitly rather than discovered.  This must pin the
    # layout of the object THIS build produced — a search that found build/dev
    # while testing build/san would report on the wrong artefact and pass.
    add_test(NAME python_abi
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/test_python_abi.py
                     $<TARGET_FILE:wrwire>
                     $<TARGET_FILE:wrist_ffi>)

    # ⚠ THE ONLY CHECK THE BINDING GETS AGAINST DATA RATHER THAN DECLARATIONS.
    # The same gather is driven twice over the same real capture — once from C,
    # once through ctypes — and the two per-block tables must be EQUAL.  A layout
    # can agree with the compiler while the wrapper around it still hands back
    # the wrong thing: a block released too early, a sample array walked with the
    # wrong stride, a pointer read as an int.
    #
    # ⚠ It runs over BOTH regimes and the empty case.  Equality alone would be
    # satisfied by two runs that reached nothing, so the test also asserts the
    # density signature by value: 1.000 over the swings, 0.125 over the still
    # wrist.  A number that never varies is a check that has stopped running.
    add_test(NAME python_replay_parity
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/test_python_replay.py
                     $<TARGET_FILE:wr_gather_replay>
                     $<TARGET_FILE:wrist_ffi>)
    set_tests_properties(python_replay_parity PROPERTIES TIMEOUT 300)

    # ⚠ THE ONE HAZARD CTYPES ADDS THAT C DOES NOT HAVE.  A history block owns
    # its samples and frees them on release; in Python that is a raw address in
    # a ctypes array, and a view kept past the release returns PLAUSIBLE SAMPLE
    # DATA rather than crashing.  A sanitizer does not see it — see the note
    # below.  So the guard that turns it into an exception is pinned here.
    add_test(NAME python_lifetime
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/test_python_lifetime.py
                     $<TARGET_FILE:wrist_ffi>
                     ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/swings.wrwire)

    # ⚠ THE REFERENCE TRANSPORT, WITH THE RADIO REPLACED AND NOTHING ELSE.
    # `python/wrist/bleak_transport.py` is the one piece of this project a
    # user's first hour runs through, and an untested transport that LOOKS
    # finished is worse than an honestly partial one.  The loopback drives the
    # transport's own notification callback, its own pump, its own drains and
    # its own write path over both fixtures.
    #
    # ⚠ It needs no `bleak` and no adapter: the client is supplied, which is the
    # documented seam.  What it therefore CANNOT see is bleak itself — the BlueZ
    # MTU trap, whether a backend really preserves ATT PDU boundaries — and that
    # is written at the top of the file rather than left to be assumed.
    #
    # ⚠ Both regimes again, and for the same reason as `gather_replay`: a check
    # that only ever sees `swings.wrwire` has seen half the device.
    add_test(NAME python_transport_loopback
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/test_python_transport.py
                     $<TARGET_FILE:wrist_ffi>
                     ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/swings.wrwire
                     ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/session1.wrwire)
    set_tests_properties(python_transport_loopback PROPERTIES TIMEOUT 300)

    # ⚠ These harnesses report in the house style — ⚠, ⛔, ≈, § — and Windows
    # gives CPython a cp1252 stdout, where writing any of them raises
    # UnicodeEncodeError and fails the test on its OUTPUT rather than its
    # subject.  Forcing UTF-8 fixes the reporting without thinning the reports,
    # which is the trade this project makes everywhere else too.
    # ⚠ The sanitizer block below REPLACES this ENVIRONMENT wholesale rather
    # than adding to it, so it repeats the variable.  Dropping it there silently
    # reintroduces this on any Windows sanitizer build.
    set_tests_properties(python_abi python_replay_parity python_lifetime
        python_transport_loopback
        PROPERTIES ENVIRONMENT "PYTHONIOENCODING=utf-8")

    # ⚠ ASan lives in the shared object, not in the interpreter, so CPython has
    # to preload the runtime before dlopening it.  `detect_leaks=0` because the
    # interpreter's own arenas are not what this build is looking for.  If the
    # preload becomes unavailable the tests are left to FAIL rather than skip: a
    # sanitizer build that quietly stops checking is the thing this project has a
    # rule about.
    #
    # ⚠⚠ AND HERE IS WHAT IT DOES NOT BUY, because assuming otherwise is worse
    # than not running it.  ASan instruments THE CODE IT COMPILED.  A ctypes read
    # happens inside CPython, which it did not — so a Python-side use-after-free,
    # a `Samples` view kept past its block's release, returns plausible sample
    # data here with NO REPORT AT ALL.  Measured, not assumed.
    #
    # What this run does catch is a fault the LIBRARY commits at the binding's
    # request — a double release, a stale pointer handed back in, a wrong length
    # into wr_session_on_bytes.  Those are real and this is the only thing that
    # sees them; a deliberate double release reports heap-use-after-free here.
    #
    # The stale-view half is covered instead by the guard in Samples._alive()
    # (python/wrist/session.py), which raises rather than reading.
    #
    # ⚠ THE RUNTIME IS NAMED DIFFERENTLY BY EACH COMPILER, and getting it wrong
    # is not a skip — it is four failing tests.  gcc ships a shared libasan.so
    # and finds it with -print-file-name.  clang links its runtime STATICALLY
    # into executables by default, so nothing resolves the __asan_*/__ubsan_*
    # symbols in libwrist_ffi.so when ctypes dlopen()s it; its shared
    # runtime lives in -print-runtime-dir under an architecture-specific name.
    # This is why the sanitizer job passed under gcc here and failed under clang
    # in CI on the same source.
    if((WR_ENABLE_ASAN OR WR_ENABLE_UBSAN) AND NOT MSVC)
        #
        # ⛔ macOS + ASan IS THE ONE CASE WHERE THE PRELOAD CANNOT EXIST, so it is
        # the one case that skips instead of failing.  The rule above — a missing
        # preload FAILS — is right when the preload is merely absent, because that
        # is a misconfiguration someone can fix.  Here it is refused by the OS:
        #
        #   dyld: `Sanitizer load violates platform policy`
        #
        # The Command Line Tools' libclang_rt.asan_osx_dynamic.dylib is a signed
        # platform binary that recent macOS will only load into an image LINKED
        # against it.  ctypes dlopen()s ours instead, and DYLD_INSERT_LIBRARIES —
        # the LD_PRELOAD equivalent — is stripped for it no matter what the host
        # process is.  Measured on macOS 27 / AppleClang 21: stripped even for a
        # freshly built, unsigned, non-hardened local executable, so this is the
        # DYLIB being restricted and not the interpreter being protected.  A
        # non-Apple runtime (full Xcode, Homebrew LLVM) is not subject to it and
        # would let these run; the Command Line Tools alone cannot.
        #
        # ⚠ DISABLED, not unregistered.  ctest then lists them as disabled on every
        # run, so the coverage this platform is missing stays legible instead of a
        # macOS sanitizer build looking as green as a Linux one.
        #
        # ⚠ What is lost here is only the ASan half, and CI covers it: the
        # sanitizer job runs under gcc on Linux, where the preload is an ordinary
        # LD_PRELOAD.
        #
        # ⚠ UBSan IS DELIBERATELY NOT IN THIS SKIP, and the difference is the
        # preload rather than the policy.  UBSan needs no interceptors, so its
        # runtime arrives by ordinary dlopen of the rpath — which the same policy
        # refuses ONLY when the host process is a platform binary.  That is exactly
        # the interpreter chosen above, so `-DWR_ENABLE_UBSAN=ON` runs all four
        # tests on a PATH python and would fail all four on the framework one.
        # ASan cannot be recovered that way: it needs DYLD_INSERT_LIBRARIES, which
        # is stripped for this runtime whatever the host.
        if(APPLE AND WR_ENABLE_ASAN)
            set_tests_properties(python_abi python_replay_parity python_lifetime
                python_transport_loopback
                PROPERTIES DISABLED TRUE)
            message(STATUS
                "python binding tests: DISABLED under ASan on macOS — dyld refuses "
                "to preload the Command Line Tools' ASan runtime into a ctypes host "
                "(`Sanitizer load violates platform policy`). The Linux/gcc CI "
                "sanitizer job is what covers them.")
        else()
            set(WR_SAN_RUNTIME "")
            execute_process(COMMAND ${CMAKE_C_COMPILER} -print-file-name=libasan.so
                            OUTPUT_VARIABLE WR_SAN_RUNTIME
                            OUTPUT_STRIP_TRAILING_WHITESPACE
                            ERROR_QUIET)
            if(NOT EXISTS "${WR_SAN_RUNTIME}")
                execute_process(COMMAND ${CMAKE_C_COMPILER} -print-runtime-dir
                                OUTPUT_VARIABLE WR_SAN_RTDIR
                                OUTPUT_STRIP_TRAILING_WHITESPACE
                                ERROR_QUIET)
                file(GLOB WR_SAN_CANDIDATES "${WR_SAN_RTDIR}/libclang_rt.asan*.so")
                if(WR_SAN_CANDIDATES)
                    list(GET WR_SAN_CANDIDATES 0 WR_SAN_RUNTIME)
                endif()
            endif()
            if(EXISTS "${WR_SAN_RUNTIME}")
                set_tests_properties(python_abi python_replay_parity python_lifetime
                    python_transport_loopback
                    PROPERTIES ENVIRONMENT
                    "LD_PRELOAD=${WR_SAN_RUNTIME};ASAN_OPTIONS=detect_leaks=0;PYTHONIOENCODING=utf-8")
            elseif(WR_ENABLE_ASAN)
                # ⚠ ASan only.  A UBSan-only build needs no preload on any platform
                # this reaches, so warning there would be crying wolf.
                message(STATUS
                    "python binding tests: no sanitizer runtime from ${CMAKE_C_COMPILER}; "
                    "they will run without the preload and are expected to fail")
            endif()
        endif()
    endif()
endif()

# ---------------------------------------------------------------------------
# Replay fixtures — real bytes off a real wG3 (tests/fixtures/README.md)
#
# ⚠ THE ONLY VALIDATION HISTORY RETRIEVAL GETS.  §7.3's buffer is
# motion-adaptive: a still wrist replays at ~100 Hz whether the gather is right
# or not, so a synthetic full-rate reply proves only that the code HANDLES one.
# Every other test in this directory drives a fake transport that was told what
# to return; these two drive the device's own bytes.
# ---------------------------------------------------------------------------
if(WR_BUILD_TOOLS AND WR_BUILD_RECORD)
    set(WR_FIXTURE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/fixtures)

    # The container itself, so a fixture corrupted in transit is caught on the
    # commit that corrupts it rather than a phase later.
    foreach(capture smoke session1 swings)
        add_test(NAME capture_verify_${capture}
                 COMMAND wrwire verify ${WR_FIXTURE_DIR}/${capture}.wrwire)
    endforeach()

    # Six real mid-stream pulls through the shipping gather.  Exit 0 means every
    # verdict passed WITH observations behind it; exit 3 means nothing was
    # checked, which the tool reports rather than passing quietly.
    add_test(NAME gather_replay
             COMMAND wr_gather_replay ${WR_FIXTURE_DIR}/swings.wrwire)
    set_tests_properties(gather_replay PROPERTIES TIMEOUT 120)

    # ⚠ AND THE OTHER REGIME, WHICH swings.wrwire CANNOT REACH.  Its five pulls
    # are over swings and come back at 657-728 Hz; this one is a calibration
    # choreography, so its single pull is over a wrist that was held still and
    # comes back at §7.3's 100 Hz floor — the shape a desk produces and the one
    # that is indistinguishable from a broken full-rate path unless you know to
    # look at the motion.  It also sits at 173 s, PAST TWO INDEX WRAPS, which is
    # the only real-hardware evidence this project has that "unwrap internally,
    # re-wrap when asking" (§7.4) holds above 131,072.
    add_test(NAME gather_replay_at_the_hundred_hertz_floor
             COMMAND wr_gather_replay ${WR_FIXTURE_DIR}/session1.wrwire)
    set_tests_properties(gather_replay_at_the_hundred_hertz_floor PROPERTIES TIMEOUT 120)

    # ⚠ And the same tool over a capture with NO retrieval in it must say so.
    # "No evidence" and "agreement" are different answers, and a report that
    # renders them identically is the failure this library exists to avoid — so
    # the empty case is pinned exactly like the full one.  Exit 3, loudly.
    #
    # ⚠ PASS_REGULAR_EXPRESSION and not WILL_FAIL: the two conflict, because a
    # pass regex replaces the exit-code check rather than adding to it.  The
    # regex is the LAST thing the tool prints, so a crash on the way there
    # cannot satisfy it — which is what makes it a real assertion and not just
    # "some output appeared".
    add_test(NAME gather_replay_reports_no_evidence_when_nothing_was_pulled
             COMMAND wr_gather_replay ${WR_FIXTURE_DIR}/smoke.wrwire)
    set_tests_properties(gather_replay_reports_no_evidence_when_nothing_was_pulled
                         PROPERTIES PASS_REGULAR_EXPRESSION
                                    "NOTHING WAS CHECKED: no block was produced")
endif()

# A standing check that the core stayed sans-I/O: no clock, no sockets, no
# threads, no file I/O.  api-request §2.0 is the whole reason the library is
# adoptable, and it is the kind of property that erodes one convenience at a
# time unless something fails the build.
if(UNIX AND NOT APPLE)
    add_test(NAME purity
             COMMAND ${CMAKE_COMMAND}
                     -DLIB=$<TARGET_FILE:wrist>
                     -P ${CMAKE_CURRENT_SOURCE_DIR}/purity.cmake)
endif()
