include(Prebuilt)

include_guard()
create_target( ll::nghttp2 )

use_prebuilt_binary(nghttp2)
if (WINDOWS)
  set_target_libraries( ll::nghttp2 ${ARCH_PREBUILT_DIRS_RELEASE}/nghttp2.lib)
elseif (DARWIN)
  set_target_libraries( ll::nghttp2 libnghttp2.dylib)
else (WINDOWS)
  set_target_libraries( ll::nghttp2 libnghttp2.a )
endif (WINDOWS)
set_target_include_dirs( ll::nghttp2 ${LIBS_PREBUILT_DIR}/include/nghttp2)
