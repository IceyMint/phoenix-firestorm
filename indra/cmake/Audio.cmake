# -*- cmake -*-
include(Prebuilt)

include_guard()
add_library( ll::vorbis INTERFACE IMPORTED )

use_system_binary(vorbis)
use_prebuilt_binary(ogg_vorbis)
target_include_directories( ll::vorbis SYSTEM INTERFACE ${LIBS_PREBUILT_DIR}/include )

if (WINDOWS)
  target_link_libraries(ll::vorbis INTERFACE ogg_static vorbis_static vorbisenc_static vorbisfile_static )
else (WINDOWS)
  # <FS:Zi> These must be in this order, or it won't link: vorbisenc vorbisfile vorbis ogg
  target_link_libraries(ll::vorbis INTERFACE vorbisenc vorbisfile vorbis ogg)
endif (WINDOWS)

