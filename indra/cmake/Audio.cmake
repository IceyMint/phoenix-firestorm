# -*- cmake -*-
include(Prebuilt)

include_guard()
create_target(ll::vorbis)

use_prebuilt_binary(ogg_vorbis)
target_include_directories( ll::vorbis SYSTEM INTERFACE ${LIBS_PREBUILT_DIR}/include )

if (WINDOWS)
  target_link_libraries(ll::vorbis INTERFACE ogg_static vorbis_static vorbisenc_static vorbisfile_static )
else (WINDOWS)
  target_link_libraries(ll::vorbis INTERFACE ogg vorbis vorbisenc vorbisfile )
endif (WINDOWS)

