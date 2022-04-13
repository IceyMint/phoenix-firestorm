# -*- cmake -*-
include(Prebuilt)

if( TARGET hunspell::hunspell )
  return()
endif()
create_target( hunspell::hunspell )

use_prebuilt_binary(libhunspell)
if (WINDOWS)
  set_target_libraries( hunspell::hunspell libhunspell)
elseif(DARWIN)
  set_target_libraries( hunspell::hunspell hunspell-1.3)
elseif(LINUX)
  set_target_libraries( hunspell::hunspell hunspell-1.3)
else()
  message(FATAL_ERROR "Invalid platform")
endif()
set_target_include_dirs( hunspell::hunspell ${LIBS_PREBUILT_DIR}/include/hunspell)
use_prebuilt_binary(dictionaries)
