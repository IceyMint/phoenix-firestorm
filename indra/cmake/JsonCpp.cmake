# -*- cmake -*-

include(Prebuilt)
if( TARGET jsoncpp::jsoncpp )
  return()
endif()
create_target( jsoncpp::jsoncpp)

use_prebuilt_binary(jsoncpp)
if (WINDOWS)
  set_target_libraries( jsoncpp::jsoncpp json_libmd.lib )
elseif (DARWIN)
  set_target_libraries( jsoncpp::jsoncpp libjson_darwin_libmt.a )
elseif (LINUX)
  set_target_libraries( jsoncpp::jsoncpp libjson_linux-gcc-4.1.3_libmt.a )
endif (WINDOWS)
set_target_include_dirs( jsoncpp::jsoncpp ${LIBS_PREBUILT_DIR}/include/json)
