# -*- cmake -*-
include(Prebuilt)

if( TARGET boost::boost )
  return()
endif()
create_target( boost::boost )

set(Boost_FIND_QUIETLY ON)
set(Boost_FIND_REQUIRED ON)

use_prebuilt_binary(boost)
set(Boost_INCLUDE_DIRS ${LIBS_PREBUILT_DIR}/include)

# As of sometime between Boost 1.67 and 1.72, Boost libraries are suffixed
# with the address size.
set(addrsfx "-x${ADDRESS_SIZE}")

if (WINDOWS)
  set_target_libraries( boost::boost
          libboost_context-mt${addrsfx}
          libboost_fiber-mt${addrsfx}
          libboost_filesystem-mt${addrsfx}
          libboost_program_options-mt${addrsfx}
          libboost_regex-mt${addrsfx}
          libboost_system-mt${addrsfx}
          libboost_thread-mt${addrsfx})
elseif (LINUX)
  set_target_libraries( boost::boost
          boost_context-mt${addrsfx}
          boost_fiber-mt${addrsfx}
          boost_filesystem-mt${addrsfx}
          boost_program_options-mt${addrsfx}
          boost_regex-mt${addrsfx}
          boost_signals-mt${addrsfx}
          boost_system-mt${addrsfx}
          boost_thread-mt${addrsfx})
elseif (DARWIN)
  set_target_libraries( boost::boost
          boost_context-mt${addrsfx}
          boost_fiber-mt${addrsfx}
          boost_filesystem-mt${addrsfx}
          boost_program_options-mt${addrsfx}
          boost_regex-mt${addrsfx}
          boost_system-mt${addrsfx}
          boost_thread-mt${addrsfx})
endif (WINDOWS)

if (LINUX)
    set(BOOST_SYSTEM_LIBRARY ${BOOST_SYSTEM_LIBRARY} rt)
    set(BOOST_THREAD_LIBRARY ${BOOST_THREAD_LIBRARY} rt)
endif (LINUX)

