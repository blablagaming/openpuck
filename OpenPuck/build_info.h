// build_info.h -- firmware build provenance, surfaced in the WebUSB panel so you can confirm EXACTLY which
// build is flashed on a board (semantic version when available, otherwise git hash, plus dirty flag).
//
// The values come from one of two sources, in priority order:
//   1) -D defines on the compile line, e.g. build.extra_flags += -DOPK_BUILD_VERSION=\"0.9.1\"
//      -DOPK_GIT_HASH=\"abc12345\" -DOPK_GIT_DIRTY=1
//   2) a generated git_version.h (written by gen_version.sh before compiling), picked up via __has_include.
// If neither is present (a bare checkout compiled by hand) the values fall back to empty-version/"unknown"/
// not-dirty, so the build always succeeds.
#pragma once

#if !defined(OPK_GIT_HASH) && defined(__has_include)
#if __has_include("git_version.h")
#include "git_version.h"
#endif
#endif

#ifndef OPK_BUILD_VERSION
#define OPK_BUILD_VERSION ""
#endif
#ifndef OPK_GIT_HASH
#define OPK_GIT_HASH "unknown"
#endif
#ifndef OPK_GIT_DIRTY
#define OPK_GIT_DIRTY 0
#endif
