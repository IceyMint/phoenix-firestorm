# -*- cmake -*-

# these should be moved to their own cmake file
include(Prebuilt)
include(Boost)

include_guard()

use_prebuilt_binary(colladadom)
use_prebuilt_binary(minizip-ng) # needed for colladadom
use_prebuilt_binary(pcre)
use_prebuilt_binary(libxml2)

create_target( ll::pcre )
target_link_libraries( ll::pcre INTERFACE pcrecpp pcre )

create_target( ll::minizip-ng )
if (WINDOWS)
    target_link_libraries( ll::minizip-ng INTERFACE libminizip )
else()
    target_link_libraries( ll::minizip-ng INTERFACE minizip )
endif()

create_target( ll::libxml )
if (WINDOWS)
    target_link_libraries( ll::libxml INTERFACE libxml2_a)
else()
    target_link_libraries( ll::libxml INTERFACE xml2)
endif()

create_target( ll::colladadom )
target_include_directories( ll::colladadom SYSTEM INTERFACE
        ${LIBS_PREBUILT_DIR}/include/collada
        ${LIBS_PREBUILT_DIR}/include/collada/1.4
        )
if (WINDOWS)
    target_link_libraries(ll::colladadom INTERFACE libcollada14dom23-s ll::libxml ll::minizip-ng )
elseif (DARWIN)
    target_link_libraries(ll::colladadom INTERFACE collada14dom ll::libxml ll::minizip-ng)
elseif (LINUX)
    target_link_libraries(ll::colladadom INTERFACE collada14dom ll::libxml ll::minizip-ng)
endif()