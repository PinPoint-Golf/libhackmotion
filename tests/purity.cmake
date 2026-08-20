# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Mark Liversedge
#
# purity.cmake — assert that the core is still sans-I/O.
#
# api-request §2.0 is the request that most shapes adoption: the library must
# not own a radio, a thread, a timer or a clock.  That property is easy to state
# and easy to erode one convenience at a time, so it is a test.
#
# Every symbol below would mean the library had started doing something the host
# is supposed to do.
#
# ⚠⚠ THIS RUNS ON `hackmotion` AND MUST NEVER BE POINTED AT `hackmotion_ffi`.
# That second target is one shared object holding the core AND the record
# module, so a language binding can dlopen a single file (CMakeLists.txt); it
# opens files by construction and would fail every check below.  That is not a
# hole in the gate — the gated target is the one a C consumer links and the one
# api-request §2.0's promise is about.  Widening the gate to cover the ffi
# object would mean deleting the record module from it; narrowing the gate to
# let the ffi object pass would mean deleting this test.  Neither is wanted.

execute_process(COMMAND nm -u --defined-only=no "${LIB}"
                OUTPUT_VARIABLE nm_out
                ERROR_VARIABLE nm_err
                RESULT_VARIABLE nm_rc)

if(NOT nm_rc EQUAL 0)
    execute_process(COMMAND nm -u "${LIB}"
                    OUTPUT_VARIABLE nm_out
                    RESULT_VARIABLE nm_rc)
endif()

if(NOT nm_rc EQUAL 0)
    message(STATUS "purity: nm unavailable, skipping")
    return()
endif()

set(forbidden
    "clock_gettime" "gettimeofday" "time@" " time$" "nanosleep" "usleep" "sleep@"
    "pthread_create" "thrd_create"
    "socket" "connect@" "bind@" "recv" "send@" "sendto" "poll@" "epoll_create"
    "fopen" "open@" "open64" "read@" "write@" "close@"
    "dbus_" "hci_" "sd_bus"
    "rand@" "srand" "getenv"
)

set(violations "")
foreach(sym ${forbidden})
    string(REPLACE "@" "" bare "${sym}")
    string(REPLACE "$" "" bare "${bare}")
    string(STRIP "${bare}" bare)
    if(nm_out MATCHES "U +${bare}\n" OR nm_out MATCHES "U +${bare}$")
        list(APPEND violations "${bare}")
    endif()
endforeach()

if(violations)
    message(FATAL_ERROR
        "libhackmotion must stay sans-I/O (api-request §2.0) but references: ${violations}")
endif()

message(STATUS "purity: core references no clock, thread, socket or file API")
