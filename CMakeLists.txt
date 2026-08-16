# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2026 Mark Liversedge
cmake_minimum_required(VERSION 3.16)

project(libhackmotion
    VERSION 0.1.0
    DESCRIPTION "Sans-I/O C library for the HackMotion wG3 BLE sensor"
    LANGUAGES C)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
option(HM_BUILD_TESTS      "Build the automated test suite"          ON)
option(HM_BUILD_RECORD     "Build the optional wire record/replay module" ON)
option(HM_BUILD_TOOLS      "Build the command line tools"            ON)
option(HM_BUILD_FFI        "Build the shared object the Python binding loads" ON)
option(HM_ENABLE_ASAN      "Build with AddressSanitizer"             OFF)
option(HM_ENABLE_UBSAN     "Build with UndefinedBehaviorSanitizer"   OFF)
option(HM_ENABLE_COVERAGE  "Build with gcov coverage instrumentation" OFF)
option(HM_WERROR           "Treat warnings as errors"                ON)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
endif()

# ---------------------------------------------------------------------------
# Warning / instrumentation flags, applied through an INTERFACE target so the
# core library never leaks them to consumers.
# ---------------------------------------------------------------------------
add_library(hm_build_flags INTERFACE)

if(MSVC)
    target_compile_options(hm_build_flags INTERFACE /W4 /permissive-)
    # The core touches none of these, but the record module and the tests open
    # files, and MSVC's C4996 on the standard C11 spellings would fail /WX.
    # We are not switching to fopen_s: the container has to read identically on
    # every platform that will ever open a capture.
    target_compile_definitions(hm_build_flags INTERFACE _CRT_SECURE_NO_WARNINGS)
    if(HM_WERROR)
        target_compile_options(hm_build_flags INTERFACE /WX)
    endif()
else()
    target_compile_options(hm_build_flags INTERFACE
        -Wall -Wextra -Wpedantic
        -Wshadow -Wconversion -Wsign-conversion
        -Wcast-qual -Wcast-align -Wpointer-arith
        -Wstrict-prototypes -Wmissing-prototypes -Wold-style-definition
        -Wredundant-decls -Wswitch-enum -Wundef
        -Wdouble-promotion -Wformat=2 -Wvla)
    if(HM_WERROR)
        target_compile_options(hm_build_flags INTERFACE -Werror)
    endif()
endif()

set(_hm_san "")
if(HM_ENABLE_ASAN)
    list(APPEND _hm_san -fsanitize=address -fno-omit-frame-pointer)
endif()
if(HM_ENABLE_UBSAN)
    list(APPEND _hm_san -fsanitize=undefined -fno-sanitize-recover=undefined)
endif()
if(_hm_san)
    target_compile_options(hm_build_flags INTERFACE ${_hm_san})
    target_link_options(hm_build_flags INTERFACE ${_hm_san})
    #
    # ⚠ CLANG LINKS ITS SANITIZER RUNTIME STATICALLY INTO THE EXECUTABLE, and
    # that breaks the one target here that is dlopen()ed rather than linked.
    # libhackmotion_ffi.so is loaded by ctypes from a plain, uninstrumented
    # CPython, so nothing has ever resolved its __asan_* / __ubsan_* references
    # and the load fails outright with `undefined symbol`.  -shared-libsan makes
    # the runtimes real shared libraries the object can depend on, and the rpath
    # is what lets the loader find them without the caller setting a variable.
    #
    # ⚠ gcc needs neither: its libasan/libubsan are already shared, which is why
    # this build was green on the development machine and red in CI on exactly
    # the same source.  Link-only — clang warns the flag is unused when
    # compiling, and -Werror would make that fatal.
    if(CMAKE_C_COMPILER_ID MATCHES "Clang")
        target_link_options(hm_build_flags INTERFACE -shared-libsan)
        execute_process(COMMAND ${CMAKE_C_COMPILER} -print-runtime-dir
                        OUTPUT_VARIABLE _hm_rtdir
                        OUTPUT_STRIP_TRAILING_WHITESPACE
                        ERROR_QUIET)
        if(IS_DIRECTORY "${_hm_rtdir}")
            target_link_options(hm_build_flags INTERFACE
                                "-Wl,-rpath,${_hm_rtdir}")
        endif()
    endif()
endif()

if(HM_ENABLE_COVERAGE)
    target_compile_options(hm_build_flags INTERFACE --coverage -O0 -g)
    target_link_options(hm_build_flags INTERFACE --coverage)
endif()

# ---------------------------------------------------------------------------
# hackmotion — the sans-I/O core.
#
# Hard rule, enforced by tests/test_purity.sh: this target links nothing but
# libm, opens no file, touches no socket, reads no clock and starts no thread.
# ---------------------------------------------------------------------------
# ⚠ Named rather than inline because a second target compiles the same files:
# `hackmotion_ffi` below is one shared object holding the core AND the record
# module, which is the only shape a language binding can load.  Two lists that
# had to be kept in step by hand would be one more thing that drifts.
set(HM_CORE_SOURCES
    src/hm_status.c
    src/hm_device.c
    src/hm_config.c
    src/hm_codec.c
    src/hm_command.c
    src/hm_unwrap.c
    src/hm_clock.c
    src/hm_coverage.c
    src/hm_density.c
    src/hm_quat.c
    src/hm_presence.c
    src/hm_overlap.c
    src/hm_session.c
    src/hm_defaults.c
    src/hm_version.c
)

set(HM_RECORD_SOURCES
    record/hm_record.c
    record/hm_reconcile.c
    record/hm_report.c
)

add_library(hackmotion ${HM_CORE_SOURCES})

target_include_directories(hackmotion
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(hackmotion PRIVATE hm_build_flags)

if(NOT WIN32)
    target_link_libraries(hackmotion PUBLIC m)
endif()

set_target_properties(hackmotion PROPERTIES
    VERSION   ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    C_VISIBILITY_PRESET hidden
    POSITION_INDEPENDENT_CODE ON)

target_compile_definitions(hackmotion PRIVATE HM_BUILDING_LIBRARY=1)
if(BUILD_SHARED_LIBS)
    target_compile_definitions(hackmotion PUBLIC HM_SHARED=1)
endif()

# ---------------------------------------------------------------------------
# hackmotion_record — the optional `.hmwire` container and the reconciliation
# that reads a capture back against docs/specification.md.
#
# ⚠ NOT part of the core and deliberately outside the purity gate: this target
# opens files.  The core copies wire chunks into a caller-supplied ring and the
# host drains them (design §5.6); this is one host that does the draining.
#
# It reaches the internal headers (hm_codec.h, hm_unwrap.h, hm_fit.h) the same
# way the tests do — by linking the static archive, where those symbols are
# ordinary globals.  A shared core hides them (C_VISIBILITY_PRESET hidden), so
# the module is disabled there rather than failing to link.
# ---------------------------------------------------------------------------
if(HM_BUILD_RECORD AND BUILD_SHARED_LIBS)
    message(STATUS
        "hackmotion_record needs the core's internal symbols; disabled for a shared build")
    set(HM_BUILD_RECORD OFF)
endif()

if(HM_BUILD_RECORD)
    add_library(hackmotion_record STATIC ${HM_RECORD_SOURCES})
    target_include_directories(hackmotion_record
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(hackmotion_record PUBLIC hackmotion)
    target_link_libraries(hackmotion_record PRIVATE hm_build_flags)
    target_compile_definitions(hackmotion_record PRIVATE HM_BUILDING_LIBRARY=1)
    set_target_properties(hackmotion_record PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# ---------------------------------------------------------------------------
# hackmotion_ffi — ONE shared object holding the core and the record module,
# for a language binding to dlopen.  python/hackmotion loads this.
#
# ⚠ WHY IT EXISTS RATHER THAN REUSING THE TARGETS ABOVE.  `hackmotion_record`
# reaches the core's internal headers, so a shared core hides the symbols it
# needs and the block above disables it for BUILD_SHARED_LIBS.  Compiling both
# source lists into ONE shared object sidesteps that entirely: hidden visibility
# hides symbols from OUTSIDE a shared object, and references within one are
# resolved at link time regardless.  A binding therefore gets the session and
# the `.hmwire` container from a single file, with only HM_API exported.
#
# ⚠⚠ THE PURITY GATE MUST NEVER BE POINTED AT THIS TARGET, and that is not an
# oversight to be tidied up.  This object opens files BY CONSTRUCTION — the
# record module is what does the draining the core refuses to do (design §5.6).
# The gate stays on `hackmotion`, which is what every C consumer links and the
# artefact the api-request §2.0 promise is about.  See tests/purity.cmake.
#
# ⚠ Both defines are required, and on Windows neither is sufficient alone:
# HM_API is `__declspec(dllexport)` only when HM_SHARED and HM_BUILDING_LIBRARY
# are both set (include/hackmotion/types.h).  Miss one and the shared object
# exports nothing, which fails at dlopen rather than at build.
# ---------------------------------------------------------------------------
if(HM_BUILD_FFI)
    add_library(hackmotion_ffi SHARED ${HM_CORE_SOURCES} ${HM_RECORD_SOURCES})
    target_include_directories(hackmotion_ffi
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(hackmotion_ffi PRIVATE hm_build_flags)
    if(NOT WIN32)
        target_link_libraries(hackmotion_ffi PUBLIC m)
    endif()
    target_compile_definitions(hackmotion_ffi PRIVATE
        HM_BUILDING_LIBRARY=1
        HM_SHARED=1)
    set_target_properties(hackmotion_ffi PROPERTIES
        VERSION   ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}
        C_VISIBILITY_PRESET hidden
        POSITION_INDEPENDENT_CODE ON)
endif()

# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------
if(HM_BUILD_TOOLS AND HM_BUILD_RECORD)
    add_executable(hmwire tools/hm_wire_tool.c tools/hm_abi_table.c)
    target_link_libraries(hmwire PRIVATE hackmotion_record hm_build_flags)
    target_include_directories(hmwire PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

    # ⚠ The only validation history retrieval ever gets.  §7.3's buffer is
    # motion-adaptive, so a synthetic full-rate reply in the test suite proves
    # the gather HANDLES one and nothing more — at a desk a correct
    # implementation and a broken one return the same even one-in-eight.  This
    # drives the gather with a real device's own bytes.
    #
    # ⚠ It IS a ctest — three of them, over the tracked fixtures in
    # tests/fixtures/ (tests/CMakeLists.txt).  This comment said "deliberately
    # NOT a ctest" until the fixtures were tracked and it was never updated,
    # which is the drift class implementation-review I10-I14 is about.
    #
    # It also has to work on a capture nobody curated: `.gitignore` excludes
    # *.hmwire everywhere else, so anyone taking a recording off a sensor points
    # it there directly, and a capture with no retrieval must say NOTHING WAS
    # CHECKED rather than print green verdicts over zero samples.
    #     ./build/dev/hm_gather_replay swings.hmwire
    add_executable(hm_gather_replay tools/hm_gather_replay.c)
    target_link_libraries(hm_gather_replay PRIVATE hackmotion_record hm_build_flags)
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(HM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS hackmotion
        EXPORT hackmotionTargets
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/hackmotion DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/tools/hackmotion.pc.in
               ${CMAKE_CURRENT_BINARY_DIR}/hackmotion.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/hackmotion.pc
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
