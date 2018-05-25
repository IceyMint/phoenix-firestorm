# BugSplat is engaged by setting environment variable BUGSPLAT_DB to the
# target BugSplat database name prior to running CMake (and during autobuild
# build).
if (DEFINED ENV{BUGSPLAT_DB})
  if (USESYSTEMLIBS)
    set(BUGSPLAT_FIND_QUIETLY ON)
    set(BUGSPLAT_FIND_REQUIRED ON)
    include(FindBUGSPLAT)
  else (USESYSTEMLIBS)
    include(Prebuilt)
    use_prebuilt_binary(bugsplat)
    if (WINDOWS)
      set(BUGSPLAT_LIBRARIES 
        ${ARCH_PREBUILT_DIRS_RELEASE}/bugsplat.lib
        )
    elseif (DARWIN)
      find_library(BUGSPLAT_LIBRARIES BugsplatMac
        PATHS "${ARCH_PREBUILT_DIRS_RELEASE}")
    else (WINDOWS)

    endif (WINDOWS)
    set(BUGSPLAT_INCLUDE_DIR ${LIBS_PREBUILT_DIR}/include/bugsplat)
  endif (USESYSTEMLIBS)
endif (DEFINED ENV{BUGSPLAT_DB})
