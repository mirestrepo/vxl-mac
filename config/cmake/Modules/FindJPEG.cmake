#
# Find the native JPEG includes and library
#


FIND_PATH(NATIVE_JPEG_INCLUDE_PATH jpeglib.h
/usr/local/include
/usr/include
)

FIND_LIBRARY(NATIVE_JPEG_LIB_PATH jpeg
/usr/lib
/usr/local/lib
)

IF(NATIVE_JPEG_INCLUDE_PATH)
  SET(NATIVE_JPEG_LIBRARY "-ljpeg" CACHE)
ENDIF(NATIVE_JPEG_INCLUDE_PATH)

