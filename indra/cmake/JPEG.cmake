# -*- cmake -*-
include(Prebuilt)

include(Linking)

include_guard()
create_target(ll::jpeglib)

use_prebuilt_binary(jpeglib)
if (LINUX)
  target_link_libraries( ll::jpeglib INTERFACE jpeg)
elseif (DARWIN)
  target_link_libraries( ll::jpeglib INTERFACE jpeg)
elseif (WINDOWS)
  target_link_libraries( ll::jpeglib INTERFACE jpeglib)
endif (LINUX)
target_include_directories( ll::jpeglib SYSTEM INTERFACE ${LIBS_PREBUILT_DIR}/include)
