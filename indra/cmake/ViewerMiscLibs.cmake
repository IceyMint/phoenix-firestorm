# -*- cmake -*-
include(Prebuilt)

if (LINUX)
  use_prebuilt_binary(libuuid)
  use_prebuilt_binary(fontconfig)
endif (LINUX)
use_prebuilt_binary(libhunspell)
use_prebuilt_binary(slvoice)

