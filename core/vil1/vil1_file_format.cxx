#ifdef __GNUC__
#pragma implementation
#endif

// This is vxl/vil/vil_file_format.cxx

#include "vil_file_format.h"

vil_file_format::~vil_file_format()
{
}

#define HAS_PNM  1
#define HAS_VIFF 1
#define HAS_IRIS 1
#define HAS_MIT  1
#define HAS_BMP  1
#define HAS_GIF  0
#define HAS_JPEG 1
#define HAS_GEN  1

#if HAS_PNM
#include <vil/file_formats/vil_pnm.h>
#endif

#if HAS_IRIS
#include <vil/file_formats/vil_iris.h>
#endif

#if HAS_MIT
#include <vil/file_formats/vil_mit.h>
#endif

#if HAS_VIFF
#include <vil/file_formats/vil_viff.h>
#endif

#if HAS_PNG
#include <vil/file_formats/vil_png.h>
#endif

#if HAS_JPEG
#include <vil/file_formats/vil_jpeg.h>
#endif

#if HAS_TIFF
#include <vil/file_formats/vil_tiff.h>
#endif

#if HAS_BMP
#include <vil/file_formats/vil_bmp.h>
#endif

#if HAS_GIF
#include <vil/file_formats/vil_gif.h>
#endif

#if HAS_GEN
#include <vil/file_formats/vil_gen.h>
#endif


static vil_file_format** storage = 0;
vil_file_format** vil_file_format::all()
{
  if (storage == 0) {
    storage = new vil_file_format*[256];
    int c = 0;

#if HAS_JPEG
    storage[c++] = new vil_jpeg_file_format;
#endif
#if HAS_PNG
    storage[c++] = new vil_png_file_format;
#endif
#if HAS_TIFF
    storage[c++] = new vil_tiff_file_format;
#endif
#if HAS_PNM
    storage[c++] = new vil_pnm_file_format;
#endif
#if HAS_IRIS
    storage[c++] = new vil_iris_file_format;
#endif
#if HAS_MIT
    storage[c++] = new vil_mit_file_format;
#endif
#if HAS_VIFF
    storage[c++] = new vil_viff_file_format;
#endif
#if HAS_BMP
    storage[c++] = new vil_bmp_file_format;
#endif
#if HAS_GIF
    storage[c++] = new vil_gif_file_format;
#endif
#if HAS_GEN
    storage[c++] = new vil_gen_file_format;
#endif

    storage[c++] = 0;
  }
  return storage;
}
