# Locate AVIFILE library and include paths

# AVIFILE (http://avifile.sourceforge.net/ )is a set of library for i386 machines
# to use various AVI codecs. Support is limited beyond Linux. Windows
# provides native AVI support, and so doesn't need this library.

# This module defines
# AVIFILE_INCLUDE_DIR, where to find avifile/avifile.h , etc.
# AVIFILE_LIBRARIES, the libraries to link against to use AVIFILE
# AVIFILE_DEFINITIONS, definitions to use when compiling code that uses AVIFILE.
# AVIFILE_FOUND, If false, don't try to use AVIFILE.

IF (UNIX)

  FIND_PATH(AVIFILE_INCLUDE_DIR avifile/avifile.h
    /usr/local/avifile/include
    /usr/local/include
    /usr/include
  )

  FIND_LIBRARY(AVIFILE_AVIPLAY_LIBRARY aviplay
    /usr/local/avifile/lib
    /usr/local/lib
    /usr/lib
  )

ENDIF (UNIX)

SET (AVIFILE_FOUND "NO")

IF(AVIFILE_INCLUDE_DIR)
  IF(AVIFILE_AVIPLAY_LIBRARY)
    SET( AVIFILE_LIBRARIES  ${AVIFILE_AVIPLAY_LIBRARY} )
    SET( AVIFILE_FOUND "YES" )
    SET( AVIFILE_DEFINITIONS "")

  ENDIF(AVIFILE_AVIPLAY_LIBRARY)
ENDIF(AVIFILE_INCLUDE_DIR)

