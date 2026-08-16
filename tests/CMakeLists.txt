# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2026 Mark Liversedge
# Tests link the library's sources directly rather than the built target, so
# they can reach internal headers (hm_codec.h, hm_fit.h, hm_unwrap.h) without
# widening the public API to make them testable.

set(HM_TEST_SOURCES
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

# The `.hmwire` container lives in a separate target because it opens files; the
# purity gate below is why the core cannot.
if(HM_BUILD_RECORD)
    list(APPEND HM_TEST_SOURCES test_record.c)
endif()

foreach(src ${HM_TEST_SOURCES})
    get_filename_component(name ${src} NAME_WE)
    add_executable(${name} ${src})
    if(name STREQUAL "test_record")
        target_link_libraries(${name} PRIVATE hackmotion_record hm_build_flags)
    else()
        target_link_libraries(${name} PRIVATE hackmotion hm_build_flags)
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

# ⚠ Two implementations of one container format — tools/hm_capture.py writes it
# and record/hm_record.c reads it — pinned against each other.  A drift there
# would produce a capture that only turns out to be unreadable later, by which
# time the sensor and the session are gone.  Skipped, loudly, with no python3.
if(HM_BUILD_TOOLS AND HM_BUILD_RECORD)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
        add_test(NAME capture_format
                 COMMAND ${Python3_EXECUTABLE}
                         ${CMAKE_CURRENT_SOURCE_DIR}/test_capture_format.py
                         $<TARGET_FILE:hmwire>)
    else()
        message(STATUS "no python3: the capture-harness format test will not run")
    endif()
endif()

# ---------------------------------------------------------------------------
# The Python binding
#
# ⚠ SAME SHAPE AGAIN, ONE REGISTER UP.  python/hackmotion/_types.py declares
# every public struct and enum a SECOND time, in ctypes.  hm_abi_check() guards
# nine struct SIZES at load and nothing else — not one field offset, not one
# enumerator — so a `skew_us` one slot out decodes every sample into plausible
# nonsense with no error anywhere.
#
# `hmwire abi` reads the real layout out of the compiler and this compares the
# two.  It found two genuine transcription errors on the day it was written:
# hm_gap_kind's first two members were the wrong way round, and four of eleven
# history statuses were on the wrong numbers.
# ---------------------------------------------------------------------------
if(HM_BUILD_TOOLS AND HM_BUILD_RECORD AND HM_BUILD_FFI AND Python3_Interpreter_FOUND)
    # ⚠ The .so is passed explicitly rather than discovered.  This must pin the
    # layout of the object THIS build produced — a search that found build/dev
    # while testing build/san would report on the wrong artefact and pass.
    add_test(NAME python_abi
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/test_python_abi.py
                     $<TARGET_FILE:hmwire>
                     $<TARGET_FILE:hackmotion_ffi>)

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
                     $<TARGET_FILE:hm_gather_replay>
                     $<TARGET_FILE:hackmotion_ffi>)
    set_tests_properties(python_replay_parity PROPERTIES TIMEOUT 300)

    # ⚠ THE ONE HAZARD CTYPES ADDS THAT C DOES NOT HAVE.  A history block owns
    # its samples and frees them on release; in Python that is a raw address in
    # a ctypes array, and a view kept past the release returns PLAUSIBLE SAMPLE
    # DATA rather than crashing.  A sanitizer does not see it — see the note
    # below.  So the guard that turns it into an exception is pinned here.
    add_test(NAME python_lifetime
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/test_python_lifetime.py
                     $<TARGET_FILE:hackmotion_ffi>
                     ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/swings.hmwire)

    # ⚠ THE REFERENCE TRANSPORT, WITH THE RADIO REPLACED AND NOTHING ELSE.
    # `python/hackmotion/bleak_transport.py` is the one piece of this project a
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
    # that only ever sees `swings.hmwire` has seen half the device.
    add_test(NAME python_transport_loopback
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/test_python_transport.py
                     $<TARGET_FILE:hackmotion_ffi>
                     ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/swings.hmwire
                     ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/session1.hmwire)
    set_tests_properties(python_transport_loopback PROPERTIES TIMEOUT 300)

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
    # into hm_session_on_bytes.  Those are real and this is the only thing that
    # sees them; a deliberate double release reports heap-use-after-free here.
    #
    # The stale-view half is covered instead by the guard in Samples._alive()
    # (python/hackmotion/session.py), which raises rather than reading.
    if(HM_ENABLE_ASAN AND NOT MSVC)
        execute_process(COMMAND ${CMAKE_C_COMPILER} -print-file-name=libasan.so
                        OUTPUT_VARIABLE HM_ASAN_RUNTIME
                        OUTPUT_STRIP_TRAILING_WHITESPACE
                        ERROR_QUIET)
        if(EXISTS "${HM_ASAN_RUNTIME}")
            set_tests_properties(python_abi python_replay_parity python_lifetime
                python_transport_loopback
                PROPERTIES ENVIRONMENT
                "LD_PRELOAD=${HM_ASAN_RUNTIME};ASAN_OPTIONS=detect_leaks=0")
        else()
            message(STATUS
                "python binding tests: no libasan.so from ${CMAKE_C_COMPILER}; "
                "they will run without the preload and are expected to fail")
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
if(HM_BUILD_TOOLS AND HM_BUILD_RECORD)
    set(HM_FIXTURE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/fixtures)

    # The container itself, so a fixture corrupted in transit is caught on the
    # commit that corrupts it rather than a phase later.
    foreach(capture smoke session1 swings)
        add_test(NAME capture_verify_${capture}
                 COMMAND hmwire verify ${HM_FIXTURE_DIR}/${capture}.hmwire)
    endforeach()

    # Six real mid-stream pulls through the shipping gather.  Exit 0 means every
    # verdict passed WITH observations behind it; exit 3 means nothing was
    # checked, which the tool reports rather than passing quietly.
    add_test(NAME gather_replay
             COMMAND hm_gather_replay ${HM_FIXTURE_DIR}/swings.hmwire)
    set_tests_properties(gather_replay PROPERTIES TIMEOUT 120)

    # ⚠ AND THE OTHER REGIME, WHICH swings.hmwire CANNOT REACH.  Its five pulls
    # are over swings and come back at 657-728 Hz; this one is a calibration
    # choreography, so its single pull is over a wrist that was held still and
    # comes back at §7.3's 100 Hz floor — the shape a desk produces and the one
    # that is indistinguishable from a broken full-rate path unless you know to
    # look at the motion.  It also sits at 173 s, PAST TWO INDEX WRAPS, which is
    # the only real-hardware evidence this project has that "unwrap internally,
    # re-wrap when asking" (§7.4) holds above 131,072.
    add_test(NAME gather_replay_at_the_hundred_hertz_floor
             COMMAND hm_gather_replay ${HM_FIXTURE_DIR}/session1.hmwire)
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
             COMMAND hm_gather_replay ${HM_FIXTURE_DIR}/smoke.hmwire)
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
                     -DLIB=$<TARGET_FILE:hackmotion>
                     -P ${CMAKE_CURRENT_SOURCE_DIR}/purity.cmake)
endif()
