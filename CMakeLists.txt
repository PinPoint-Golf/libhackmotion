# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Mark Liversedge
cmake_minimum_required(VERSION 3.16)

project(libwrist
    VERSION 0.2.0
    DESCRIPTION "Sans-I/O C library for HackMotion wrist BLE sensors"
    LANGUAGES C)

# ---------------------------------------------------------------------------
# Are we the top-level project, or embedded in someone else's build?
#
# ⚠ THE CACHE IS NOT DIRECTORY-SCOPED.  Nearly every other CMake variable is —
# a subdirectory can set CMAKE_C_STANDARD and only its own targets see it — but
# there is ONE cache per build tree, so a `set(... CACHE ... FORCE)` down here
# rewrites the *consumer's* settings.  Everything below that could reach outside
# this directory is gated on this variable.
#
# The library is meant to be embedded (api-request §2.0), so the rule is: adding
# libwrist to a project must change NOTHING about that project.  Not its
# build type, not its test registry, not its install manifest.
#
# ⚠ Spelled as a CMAKE_SOURCE_DIR comparison rather than PROJECT_IS_TOP_LEVEL
# deliberately: that variable is CMake 3.21+ and this project promises 3.16,
# which is the system CMake on Ubuntu 20.04 LTS.  The comparison is exact and
# works on every version, so the floor does not have to move for this.
# ---------------------------------------------------------------------------
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(WR_IS_TOP_LEVEL ON)
else()
    set(WR_IS_TOP_LEVEL OFF)
endif()

# ---------------------------------------------------------------------------
# Options
#
# The four developer-facing ones default to WR_IS_TOP_LEVEL: building this
# repository directly gets the tests, the tools, the FFI object and -Werror
# exactly as before, while a consumer that add_subdirectory()s it gets the
# library and nothing else.  A consumer should not have to read this file to
# discover it must switch four things off, and would otherwise inherit ~24 test
# binaries, two CLI tools, a shared object it will never load, and 24 ctest
# registrations in its own test registry.
#
# WR_BUILD_RECORD stays ON either way — it is a library target rather than
# developer tooling, and an embedder linking the .wrwire container is an
# ordinary use (PinPoint Studio's offline tools do exactly that).
# ---------------------------------------------------------------------------
option(WR_BUILD_TESTS      "Build the automated test suite"          ${WR_IS_TOP_LEVEL})
option(WR_BUILD_RECORD     "Build the optional wire record/replay module" ON)
option(WR_BUILD_TOOLS      "Build the command line tools"            ${WR_IS_TOP_LEVEL})
option(WR_BUILD_FFI        "Build the shared object the Python binding loads" ${WR_IS_TOP_LEVEL})
option(WR_ENABLE_ASAN      "Build with AddressSanitizer"             OFF)
option(WR_ENABLE_UBSAN     "Build with UndefinedBehaviorSanitizer"   OFF)
option(WR_ENABLE_COVERAGE  "Build with gcov coverage instrumentation" OFF)
# ⚠ -Werror is a CI gate, not a property of the code.  Imposing it on a consumer
# means their build breaks because THEIR compiler version warns where ours does
# not — a failure they cannot fix and did not cause.
option(WR_WERROR           "Treat warnings as errors"                ${WR_IS_TOP_LEVEL})
option(WR_INSTALL          "Generate the install rules"              ${WR_IS_TOP_LEVEL})

# Directory-scoped, so these reach no further than this subtree — unlike the
# cache write below, they are safe to set unconditionally.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ⚠ Top-level only: this is a cache FORCE, so embedded it would silently rewrite
# the consumer's build type and hand them an unoptimised everything.
if(WR_IS_TOP_LEVEL AND NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
endif()

# ---------------------------------------------------------------------------
# Warning / instrumentation flags, applied through an INTERFACE target so the
# core library never leaks them to consumers.
# ---------------------------------------------------------------------------
add_library(wr_build_flags INTERFACE)

if(MSVC)
    target_compile_options(wr_build_flags INTERFACE /W4 /permissive-)
    # The core touches none of these, but the record module and the tests open
    # files, and MSVC's C4996 on the standard C11 spellings would fail /WX.
    # We are not switching to fopen_s: the container has to read identically on
    # every platform that will ever open a capture.
    target_compile_definitions(wr_build_flags INTERFACE _CRT_SECURE_NO_WARNINGS)
    if(WR_WERROR)
        target_compile_options(wr_build_flags INTERFACE /WX)
    endif()
else()
    target_compile_options(wr_build_flags INTERFACE
        -Wall -Wextra -Wpedantic
        -Wshadow -Wconversion -Wsign-conversion
        -Wcast-qual -Wcast-align -Wpointer-arith
        -Wstrict-prototypes -Wmissing-prototypes -Wold-style-definition
        -Wredundant-decls -Wswitch-enum -Wundef
        -Wdouble-promotion -Wformat=2 -Wvla)
    if(WR_WERROR)
        target_compile_options(wr_build_flags INTERFACE -Werror)
    endif()
endif()

set(_wr_san "")
if(WR_ENABLE_ASAN)
    list(APPEND _wr_san -fsanitize=address -fno-omit-frame-pointer)
endif()
if(WR_ENABLE_UBSAN)
    list(APPEND _wr_san -fsanitize=undefined -fno-sanitize-recover=undefined)
endif()
if(_wr_san)
    target_compile_options(wr_build_flags INTERFACE ${_wr_san})
    target_link_options(wr_build_flags INTERFACE ${_wr_san})
    #
    # ⚠ CLANG LINKS ITS SANITIZER RUNTIME STATICALLY INTO THE EXECUTABLE, and
    # that breaks the one target here that is dlopen()ed rather than linked.
    # libwrist_ffi.so is loaded by ctypes from a plain, uninstrumented
    # CPython, so nothing has ever resolved its __asan_* / __ubsan_* references
    # and the load fails outright with `undefined symbol`.  -shared-libsan makes
    # the runtimes real shared libraries the object can depend on, and the rpath
    # is what lets the loader find them without the caller setting a variable.
    #
    # ⚠ gcc needs neither: its libasan/libubsan are already shared, which is why
    # this build was green on the development machine and red in CI on exactly
    # the same source.  Link-only — clang warns the flag is unused when
    # compiling, and -Werror would make that fatal.
    #
    # ⚠ APPLE IS THE EXCEPTION: ld64 warns `duplicate -rpath ... ignored` on
    # every link, because -shared-libsan already had the driver add exactly this
    # path.  The warning is harmless and the rpath is right either way, but it
    # fires once per target and trains the eye to skim linker output — which is
    # the opposite of what -Werror is for.  ld on Linux adds nothing of its own,
    # so there the explicit flag is load-bearing and must stay.
    if(CMAKE_C_COMPILER_ID MATCHES "Clang")
        target_link_options(wr_build_flags INTERFACE -shared-libsan)
        if(NOT APPLE)
            execute_process(COMMAND ${CMAKE_C_COMPILER} -print-runtime-dir
                            OUTPUT_VARIABLE _wr_rtdir
                            OUTPUT_STRIP_TRAILING_WHITESPACE
                            ERROR_QUIET)
            if(IS_DIRECTORY "${_wr_rtdir}")
                target_link_options(wr_build_flags INTERFACE
                                    "-Wl,-rpath,${_wr_rtdir}")
            endif()
        endif()
    endif()
endif()

if(WR_ENABLE_COVERAGE)
    target_compile_options(wr_build_flags INTERFACE --coverage -O0 -g)
    target_link_options(wr_build_flags INTERFACE --coverage)
endif()

# ---------------------------------------------------------------------------
# wrist — the sans-I/O core.
#
# Hard rule, enforced by tests/test_purity.sh: this target links nothing but
# libm, opens no file, touches no socket, reads no clock and starts no thread.
# ---------------------------------------------------------------------------
# ⚠ Named rather than inline because a second target compiles the same files:
# `wrist_ffi` below is one shared object holding the core AND the record
# module, which is the only shape a language binding can load.  Two lists that
# had to be kept in step by hand would be one more thing that drifts.
set(WR_CORE_SOURCES
    src/wr_status.c
    src/wr_device.c
    src/wr_config.c
    src/wr_codec.c
    src/wr_command.c
    src/wr_unwrap.c
    src/wr_clock.c
    src/wr_coverage.c
    src/wr_density.c
    src/wr_quat.c
    src/wr_presence.c
    src/wr_overlap.c
    src/wr_session.c
    src/wr_defaults.c
    src/wr_version.c
)

set(WR_RECORD_SOURCES
    record/wr_record.c
    record/wr_reconcile.c
    record/wr_report.c
)

add_library(wrist ${WR_CORE_SOURCES})
add_library(wrist::wrist ALIAS wrist)

target_include_directories(wrist
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(wrist PRIVATE $<BUILD_INTERFACE:wr_build_flags>)

if(NOT WIN32)
    target_link_libraries(wrist PUBLIC m)
endif()

set_target_properties(wrist PROPERTIES
    VERSION   ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    C_VISIBILITY_PRESET hidden
    POSITION_INDEPENDENT_CODE ON)

target_compile_definitions(wrist PRIVATE WR_BUILDING_LIBRARY=1)
if(BUILD_SHARED_LIBS)
    target_compile_definitions(wrist PUBLIC WR_SHARED=1)
endif()

# ---------------------------------------------------------------------------
# wrist_record — the optional `.wrwire` container and the reconciliation
# that reads a capture back against docs/specification.md.
#
# ⚠ NOT part of the core and deliberately outside the purity gate: this target
# opens files.  The core copies wire chunks into a caller-supplied ring and the
# host drains them (design §5.6); this is one host that does the draining.
#
# It reaches the internal headers (wr_codec.h, wr_unwrap.h, wr_fit.h) the same
# way the tests do — by linking the static archive, where those symbols are
# ordinary globals.  A shared core hides them (C_VISIBILITY_PRESET hidden), so
# the module is disabled there rather than failing to link.
# ---------------------------------------------------------------------------
if(WR_BUILD_RECORD AND BUILD_SHARED_LIBS)
    message(STATUS
        "wrist_record needs the core's internal symbols; disabled for a shared build")
    set(WR_BUILD_RECORD OFF)
endif()

if(WR_BUILD_RECORD)
    add_library(wrist_record STATIC ${WR_RECORD_SOURCES})
    target_include_directories(wrist_record
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(wrist_record PUBLIC wrist)
    target_link_libraries(wrist_record PRIVATE wr_build_flags)
    target_compile_definitions(wrist_record PRIVATE WR_BUILDING_LIBRARY=1)
    set_target_properties(wrist_record PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# ---------------------------------------------------------------------------
# wrist_ffi — ONE shared object holding the core and the record module,
# for a language binding to dlopen.  python/wrist loads this.
#
# ⚠ WHY IT EXISTS RATHER THAN REUSING THE TARGETS ABOVE.  `wrist_record`
# reaches the core's internal headers, so a shared core hides the symbols it
# needs and the block above disables it for BUILD_SHARED_LIBS.  Compiling both
# source lists into ONE shared object sidesteps that entirely: hidden visibility
# hides symbols from OUTSIDE a shared object, and references within one are
# resolved at link time regardless.  A binding therefore gets the session and
# the `.wrwire` container from a single file, with only WR_API exported.
#
# ⚠⚠ THE PURITY GATE MUST NEVER BE POINTED AT THIS TARGET, and that is not an
# oversight to be tidied up.  This object opens files BY CONSTRUCTION — the
# record module is what does the draining the core refuses to do (design §5.6).
# The gate stays on `wrist`, which is what every C consumer links and the
# artefact the api-request §2.0 promise is about.  See tests/purity.cmake.
#
# ⚠ Both defines are required, and on Windows neither is sufficient alone:
# WR_API is `__declspec(dllexport)` only when WR_SHARED and WR_BUILDING_LIBRARY
# are both set (include/wrist/types.h).  Miss one and the shared object
# exports nothing, which fails at dlopen rather than at build.
# ---------------------------------------------------------------------------
if(WR_BUILD_FFI)
    add_library(wrist_ffi SHARED ${WR_CORE_SOURCES} ${WR_RECORD_SOURCES})
    target_include_directories(wrist_ffi
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(wrist_ffi PRIVATE wr_build_flags)
    if(NOT WIN32)
        target_link_libraries(wrist_ffi PUBLIC m)
    endif()
    target_compile_definitions(wrist_ffi PRIVATE
        WR_BUILDING_LIBRARY=1
        WR_SHARED=1)
    set_target_properties(wrist_ffi PROPERTIES
        VERSION   ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}
        C_VISIBILITY_PRESET hidden
        POSITION_INDEPENDENT_CODE ON)
endif()

# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------
if(WR_BUILD_TOOLS AND WR_BUILD_RECORD)
    add_executable(wrwire tools/wr_wire_tool.c tools/wr_abi_table.c)
    target_link_libraries(wrwire PRIVATE wrist_record wr_build_flags)
    target_include_directories(wrwire PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

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
    # *.wrwire everywhere else, so anyone taking a recording off a sensor points
    # it there directly, and a capture with no retrieval must say NOTHING WAS
    # CHECKED rather than print green verdicts over zero samples.
    #     ./build/dev/wr_gather_replay swings.wrwire
    add_executable(wr_gather_replay tools/wr_gather_replay.c)
    target_link_libraries(wr_gather_replay PRIVATE wrist_record wr_build_flags)
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(WR_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------
# Install
#
# ⚠ WR_INSTALL, because an install rule declared here lands in the CONSUMER's
# install manifest.  Embedded and ungated, `make install` or a CPack run in the
# consuming project ships libwrist.a, this entire header tree and
# wrist.pc into a bundle nobody meant to put them in.
#
# add_subdirectory(... EXCLUDE_FROM_ALL) does suppress these — but that is the
# consumer remembering to pass a flag whose name says nothing about installing,
# which is not a contract worth resting on.
# ---------------------------------------------------------------------------
if(WR_INSTALL)
    include(GNUInstallDirs)
    install(TARGETS wrist
            EXPORT wristTargets
            LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})
    install(DIRECTORY include/wrist DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/tools/wrist.pc.in
                   ${CMAKE_CURRENT_BINARY_DIR}/wrist.pc @ONLY)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/wrist.pc
            DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

    # The CMake half of the same offer, so `find_package(wrist CONFIG)` yields
    # the same wrist::wrist that add_subdirectory() does and a consumer need not
    # know which way the library arrived.  The export set above is inert without
    # this — it was declared and never installed, which looked like a package
    # config existed when none did.
    #
    # ⚠ SameMinorVersion, not SameMajorVersion.  At 0.x the major is zero and
    # SameMajorVersion would declare 0.2 and 0.9 interchangeable, which is the
    # opposite of what a pre-1.0 version number means.
    include(CMakePackageConfigHelpers)
    install(EXPORT wristTargets
            FILE      wristTargets.cmake
            NAMESPACE wrist::
            DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/wrist)
    configure_package_config_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/tools/wrist-config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/wrist-config.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/wrist)
    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/wrist-config-version.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMinorVersion)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/wrist-config.cmake
                  ${CMAKE_CURRENT_BINARY_DIR}/wrist-config-version.cmake
            DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/wrist)
endif()
