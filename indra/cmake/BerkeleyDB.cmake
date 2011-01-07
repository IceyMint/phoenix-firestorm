# -*- cmake -*-

set(DB_FIND_QUIETLY ON)
set(DB_FIND_REQUIRED ON)

if (STANDALONE)
  include(FindBerkeleyDB)
else (STANDALONE)
  if (LINUX)
    # Need to add dependency pthread explicitely to support ld.gold.
    set(DB_LIBRARIES db-4.2 pthread)
  else (LINUX)
    set(DB_LIBRARIES db-4.2)
  endif (LINUX)
  set(DB_INCLUDE_DIRS ${LIBS_PREBUILT_DIR}/include)
endif (STANDALONE)
