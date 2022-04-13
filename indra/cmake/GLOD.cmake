# -*- cmake -*-
include(Prebuilt)

if( TARGET glod::glod )
  return()
endif()
create_target( glod::glod )

use_prebuilt_binary(glod)

set(GLODLIB ON CACHE BOOL "Using GLOD library")

set_target_include_dirs( glod::glod ${LIBS_PREBUILT_DIR}/include)
set_target_libraries( glod::glod GLOD )
