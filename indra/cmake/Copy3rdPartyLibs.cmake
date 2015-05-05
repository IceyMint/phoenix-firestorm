# -*- cmake -*-

# The copy_win_libs folder contains file lists and a script used to
# copy dlls, exes and such needed to run the SecondLife from within
# VisualStudio.

include(CMakeCopyIfDifferent)
include(Linking)

###################################################################
# set up platform specific lists of files that need to be copied
###################################################################
if(WINDOWS)
    set(SHARED_LIB_STAGING_DIR_DEBUG            "${SHARED_LIB_STAGING_DIR}/Debug")
    set(SHARED_LIB_STAGING_DIR_RELWITHDEBINFO   "${SHARED_LIB_STAGING_DIR}/RelWithDebInfo")
    set(SHARED_LIB_STAGING_DIR_RELEASE          "${SHARED_LIB_STAGING_DIR}/Release")

    #*******************************
    # VIVOX - *NOTE: no debug version
    set(vivox_src_dir "${ARCH_PREBUILT_DIRS_RELEASE}")
    set(vivox_files
        SLVoice.exe
        ca-bundle.crt
        libsndfile-1.dll
        vivoxplatform.dll
        vivoxsdk.dll
        ortp.dll
        zlib1.dll
        vivoxoal.dll
        )

    #*******************************
    # Misc shared libs 

    set(debug_src_dir "${ARCH_PREBUILT_DIRS_DEBUG}")
    set(debug_files
        openjpegd.dll
        libapr-1.dll
        libaprutil-1.dll
        libapriconv-1.dll
        ssleay32.dll
        libeay32.dll
        glod.dll    
        libhunspell.dll
        )

    set(release_src_dir "${ARCH_PREBUILT_DIRS_RELEASE}")
    set(release_files
        openjpeg.dll
        libapr-1.dll
        libaprutil-1.dll
        libapriconv-1.dll
        ssleay32.dll
        libeay32.dll
        glod.dll
        libhunspell.dll
        )
    
#    set(debug_files ${debug_files} growl++.dll growl.dll )
#    set(release_files ${release_files} growl++.dll growl.dll )

    if(USE_TCMALLOC)
      set(debug_files ${debug_files} libtcmalloc_minimal-debug.dll)
      set(release_files ${release_files} libtcmalloc_minimal.dll)
    endif(USE_TCMALLOC)

    if (FMODEX)
      if( NOT ND_BUILD64BIT_ARCH )
        set(debug_files ${debug_files} fmodexL.dll)
        set(release_files ${release_files} fmodex.dll)
      else( NOT ND_BUILD64BIT_ARCH )
        set(debug_files ${debug_files} fmodexL64.dll)
        set(release_files ${release_files} fmodex64.dll)
      endif( NOT ND_BUILD64BIT_ARCH )
    endif (FMODEX)

    if (LEAPMOTION)
      if( NOT ND_BUILD64BIT_ARCH )
        set(debug_files ${debug_files} x86/Leapd.dll)
        set(release_files ${release_files} x86/Leap.dll)
      else( NOT ND_BUILD64BIT_ARCH )
        set(debug_files ${debug_files} x64/Leapd.dll)
        set(release_files ${release_files} x64/Leap.dll)
      endif( NOT ND_BUILD64BIT_ARCH )
    endif (LEAPMOTION)

# <FS:ND> Copy pdb files for symbol generation too
   if( NOT ND_BUILD64BIT_ARCH )
     #set(debug_files ${debug_files} ssleay32.pdb libeay32.pdb apr-1.pdb aprutil-1.pdb growl.pdb growl++.pdb )
     #set(release_files ${release_files} ssleay32.pdb libeay32.pdb apr-1.pdb aprutil-1.pdb growl.pdb growl++.pdb )
     set(debug_files ${debug_files} ssleay32.pdb libeay32.pdb apr-1.pdb aprutil-1.pdb )
     set(release_files ${release_files} ssleay32.pdb libeay32.pdb apr-1.pdb aprutil-1.pdb )
   endif( NOT ND_BUILD64BIT_ARCH )
# </FS:ND>

    #*******************************
    # Copy MS C runtime dlls, required for packaging.
    # *TODO - Adapt this to support VC9
    if (MSVC80)
        set(MSVC_VER 80)
        set(MSVC_VERDOT 8.0)
    elseif (MSVC_VERSION EQUAL 1600) # VisualStudio 2010
        set(MSVC_VER 100)
        set(MSVC_VERDOT 10.0)
    elseif (MSVC_VERSION EQUAL 1800) # VisualStudio 2013, which is (sigh) VS 12
        set(MSVC_VER 120)
        set(MSVC_VERDOT 12.0)
    else (MSVC80)
        MESSAGE(WARNING "New MSVC_VERSION ${MSVC_VERSION} of MSVC: adapt Copy3rdPartyLibs.cmake")
    endif (MSVC80)

    FIND_PATH(debug_msvc_redist_path msvcr${MSVC_VER}d.dll
        PATHS
        ${MSVC_DEBUG_REDIST_PATH}
         [HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\VisualStudio\\${MSVC_VERDOT}\\Setup\\VC;ProductDir]/redist/Debug_NonRedist/x86/Microsoft.VC${MSVC_VER}.DebugCRT
        [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Windows;Directory]/SysWOW64
        [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Windows;Directory]/System32
        NO_DEFAULT_PATH
        )

    if(EXISTS ${debug_msvc_redist_path})
        set(debug_msvc_files
            msvcr${MSVC_VER}d.dll
            msvcp${MSVC_VER}d.dll
            )

        copy_if_different(
            ${debug_msvc_redist_path}
            "${SHARED_LIB_STAGING_DIR_DEBUG}"
            out_targets
            ${debug_msvc_files}
            )
        set(third_party_targets ${third_party_targets} ${out_targets})

    endif ()

    FIND_PATH(release_msvc_redist_path msvcr${MSVC_VER}.dll
        PATHS
        ${MSVC_REDIST_PATH}
         [HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\VisualStudio\\${MSVC_VERDOT}\\Setup\\VC;ProductDir]/redist/x86/Microsoft.VC${MSVC_VER}.CRT
        [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Windows;Directory]/SysWOW64
        [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Windows;Directory]/System32
        NO_DEFAULT_PATH
        )

    if(EXISTS ${release_msvc_redist_path})
        set(release_msvc_files
            msvcr${MSVC_VER}.dll
            msvcp${MSVC_VER}.dll
            )

        copy_if_different(
            ${release_msvc_redist_path}
            "${SHARED_LIB_STAGING_DIR_RELEASE}"
            out_targets
            ${release_msvc_files}
            )
        set(third_party_targets ${third_party_targets} ${out_targets})

        copy_if_different(
            ${release_msvc_redist_path}
            "${SHARED_LIB_STAGING_DIR_RELWITHDEBINFO}"
            out_targets
            ${release_msvc_files}
            )
        set(third_party_targets ${third_party_targets} ${out_targets})
          
    endif ()


elseif(DARWIN)
    set(SHARED_LIB_STAGING_DIR_DEBUG            "${SHARED_LIB_STAGING_DIR}/Debug/Resources")
    set(SHARED_LIB_STAGING_DIR_RELWITHDEBINFO   "${SHARED_LIB_STAGING_DIR}/RelWithDebInfo/Resources")
    set(SHARED_LIB_STAGING_DIR_RELEASE          "${SHARED_LIB_STAGING_DIR}/Release/Resources")

    set(vivox_src_dir "${ARCH_PREBUILT_DIRS_RELEASE}")
    set(vivox_files
        SLVoice
        ca-bundle.crt
        libsndfile.dylib
        libvivoxoal.dylib
        libortp.dylib
        libvivoxplatform.dylib
        libvivoxsdk.dylib
       )
    set(debug_src_dir "${ARCH_PREBUILT_DIRS_DEBUG}")
    set(debug_files
       )
    set(release_src_dir "${ARCH_PREBUILT_DIRS_RELEASE}")
    set(release_files
        libapr-1.0.dylib
        libapr-1.dylib
        libaprutil-1.0.dylib
        libaprutil-1.dylib
        #libexception_handler.dylib
        libexpat.1.5.2.dylib
        libexpat.dylib
        libGLOD.dylib
        libhunspell-1.3.0.dylib
        libndofdev.dylib
	libgrowl.dylib
        libgrowl++.dylib
       )

    # <FS:ND> We only ever need google breakpad when crash reporting is used
    if(RELEASE_CRASH_REPORTING OR NON_RELEASE_CRASH_REPORTING)
      set(release_files ${release_files} "libexception_handler.dylib")
    endif(RELEASE_CRASH_REPORTING OR NON_RELEASE_CRASH_REPORTING)
    
    if (FMODEX)
      set(debug_files ${debug_files} libfmodexL.dylib)
      set(release_files ${release_files} libfmodex.dylib)
    endif (FMODEX)

    if (LEAPMOTION)
      set(release_files ${release_files} libLeap.dylib)
    endif (LEAPMOTION)

elseif(LINUX)
    # linux is weird, multiple side by side configurations aren't supported
    # and we don't seem to have any debug shared libs built yet anyways...
    set(SHARED_LIB_STAGING_DIR_DEBUG            "${SHARED_LIB_STAGING_DIR}")
    set(SHARED_LIB_STAGING_DIR_RELWITHDEBINFO   "${SHARED_LIB_STAGING_DIR}")
    set(SHARED_LIB_STAGING_DIR_RELEASE          "${SHARED_LIB_STAGING_DIR}")

    set(vivox_src_dir "${ARCH_PREBUILT_DIRS_RELEASE}")
    set(vivox_files
        libsndfile.so.1
        libortp.so
        libvivoxoal.so.1
        libvivoxplatform.so
        libvivoxsdk.so
        SLVoice
        # ca-bundle.crt   #No cert for linux.  It is actually still 3.2SDK.
       )
    # *TODO - update this to use LIBS_PREBUILT_DIR and LL_ARCH_DIR variables
    # or ARCH_PREBUILT_DIRS
    set(debug_src_dir "${ARCH_PREBUILT_DIRS_DEBUG}")
    set(debug_files
       )
    # *TODO - update this to use LIBS_PREBUILT_DIR and LL_ARCH_DIR variables
    # or ARCH_PREBUILT_DIRS
    set(release_src_dir "${ARCH_PREBUILT_DIRS_RELEASE}")
    # *FIX - figure out what to do with duplicate libalut.so here -brad
    set(release_files
        libapr-1.so.0
        libaprutil-1.so.0
        libatk-1.0.so
        libdb-5.1.so
        libexpat.so
        libexpat.so.1
        libfreetype.so.6.6.2
        libfreetype.so.6
        libGLOD.so
        libgmodule-2.0.so
        libgobject-2.0.so
        libhunspell-1.3.so.0.0.0
        libopenal.so
        libopenjpeg.so
        libuuid.so.16
        libuuid.so.16.0.22
        libfontconfig.so.1.8.0
        libfontconfig.so.1
       )

    if (USE_TCMALLOC)
      set(release_files ${release_files} "libtcmalloc_minimal.so")
    endif (USE_TCMALLOC)

    if (FMODEX)
      set(debug_files ${debug_files} "libfmodexL.so")
      set(release_files ${release_files} "libfmodex.so")
    endif (FMODEX)

    if (LEAPMOTION)
      if( NOT ND_BUILD64BIT_ARCH )
        set(release_files ${release_files} x86/libLeap.so)
      else( NOT ND_BUILD64BIT_ARCH )
        set(release_files ${release_files} x64/libLeap.so)
      endif( NOT ND_BUILD64BIT_ARCH )
    endif (LEAPMOTION)

else(WINDOWS)
    message(STATUS "WARNING: unrecognized platform for staging 3rd party libs, skipping...")
    set(vivox_src_dir "${CMAKE_SOURCE_DIR}/newview/vivox-runtime/i686-linux")
    set(vivox_files "")
    # *TODO - update this to use LIBS_PREBUILT_DIR and LL_ARCH_DIR variables
    # or ARCH_PREBUILT_DIRS
    set(debug_src_dir "${CMAKE_SOURCE_DIR}/../libraries/i686-linux/lib/debug")
    set(debug_files "")
    # *TODO - update this to use LIBS_PREBUILT_DIR and LL_ARCH_DIR variables
    # or ARCH_PREBUILT_DIRS
    set(release_src_dir "${CMAKE_SOURCE_DIR}/../libraries/i686-linux/lib/release")
    set(release_files "")

    set(debug_llkdu_src "")
    set(debug_llkdu_dst "")
    set(release_llkdu_src "")
    set(release_llkdu_dst "")
    set(relwithdebinfo_llkdu_dst "")
endif(WINDOWS)


################################################################
# Done building the file lists, now set up the copy commands.
################################################################

copy_if_different(
    ${vivox_src_dir}
    "${SHARED_LIB_STAGING_DIR_DEBUG}"
    out_targets 
    ${vivox_files}
    )
set(third_party_targets ${third_party_targets} ${out_targets})

copy_if_different(
    ${vivox_src_dir}
    "${SHARED_LIB_STAGING_DIR_RELEASE}"
    out_targets
    ${vivox_files}
    )
set(third_party_targets ${third_party_targets} ${out_targets})

copy_if_different(
    ${vivox_src_dir}
    "${SHARED_LIB_STAGING_DIR_RELWITHDEBINFO}"
    out_targets
    ${vivox_files}
    )
set(third_party_targets ${third_party_targets} ${out_targets})



copy_if_different(
    ${debug_src_dir}
    "${SHARED_LIB_STAGING_DIR_DEBUG}"
    out_targets
    ${debug_files}
    )
set(third_party_targets ${third_party_targets} ${out_targets})

copy_if_different(
    ${release_src_dir}
    "${SHARED_LIB_STAGING_DIR_RELEASE}"
    out_targets
    ${release_files}
    )
set(third_party_targets ${third_party_targets} ${out_targets})

copy_if_different(
    ${release_src_dir}
    "${SHARED_LIB_STAGING_DIR_RELWITHDEBINFO}"
    out_targets
    ${release_files}
    )
set(third_party_targets ${third_party_targets} ${out_targets})

if(NOT USESYSTEMLIBS)
  add_custom_target(
      stage_third_party_libs ALL
      DEPENDS ${third_party_targets}
      )
endif(NOT USESYSTEMLIBS)
