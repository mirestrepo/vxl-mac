#include "ImageWarp.h"

#include <vcl_cassert.h>
#include <vcl_iostream.h>
#include <vnl/vnl_math.h>
#include <vil/vil_file_image.h>
#include <vil/vil_memory_image_of.h>

#include <oxp/Mapping_2d_2d.h>

template <class PixelType>
void ImageWarp<PixelType>::mean_nz_intensity(const vil_memory_image_of<PixelType>& in,
                                             int x, int y, int window_size,
                                             ImageWarp<PixelType>::real_t* out,
                                             //typename vnl_numeric_traits<PixelType>::real_t* out,
                                             int * nnzp)
{
  // taking a const reference is slower and causes missing symbols (SunPro 5.0)
  PixelType const /*&*/ zero = vnl_numeric_traits<PixelType>::zero;
  PixelType total_nz = zero;
  int nnz = 0;
  if (in.in_range_window(x, y, window_size)) {
    for (int iy = y - window_size; iy < y + window_size; ++iy)
      for (int ix = x - window_size; ix < x + window_size; ++ix) {
        PixelType v = in(ix, iy);
        if (v != zero) {
          total_nz += v;
          ++nnz;
        }
      }
  } else {
    for (int iy = y - window_size; iy < y + window_size; ++iy)
      for (int ix = x - window_size; ix < x + window_size; ++ix)
        if (in.in_range(ix, iy)) {
          PixelType v = in(ix, iy);
          if (v != zero) {
            total_nz += v;
            ++nnz;
          }
        }
  }

  if (nnzp)
    *nnzp = nnz;
  if (nnz > 0)
    *out = total_nz * (1.0 / nnz);
}

template <class PixelType>
void ImageWarp<PixelType>::gapfill(vil_memory_image_of<PixelType>& out, int ngaps)
{
  int w = out.width();
  int h = out.height();
  // Fill gaps
  int minnz = 4;
  while (ngaps > 0) {
    while (1) {
      vcl_cerr << "Gapfilling " << ngaps << " pixels\n";
      int old_ngaps = ngaps;
      for(int oy = 0; oy < h; ++oy)
        for(int ox = 0; ox < w; ++ox)
          if (out(ox, oy) == vnl_numeric_traits<PixelType>::zero) {
            int nnz;
            real_t tmp;
            mean_nz_intensity(out, ox, oy, 1, &tmp, &nnz);
            if (nnz >= minnz) {
              --ngaps;
              out(ox, oy) = PixelType(tmp);
            }
          }
      if (ngaps == old_ngaps)
        break;
      old_ngaps = ngaps;
    }
    --minnz;
  }
}

template <class PixelType>
void ImageWarp<PixelType>::warp(Mapping_2d_2d& map,
                                const vil_memory_image_of<PixelType>& in,
                                vil_memory_image_of<PixelType>& out)
{
  // out.Clear();
//abort(); // is not defined without #include <vcl_cstdlib.h>
  int w = in.width();
  int h = in.height();
  int ngaps = 0;
  for(int iy = 0; iy < h; ++iy)
    for(int ix = 0; ix < w; ++ix) {
      // *** Correct (ix, iy) to (ox,oy)

      // rdc correct
      double oxd, oyd;
      map.map(double(ix), double(iy), &oxd, &oyd);

      // decondition
      int ox = vnl_math_rnd(oxd);
      int oy = vnl_math_rnd(oyd);

      if (out.in_range(ox, oy))
        out(ox, oy) = in(ix,iy);
      else
        ++ngaps;
    }
  // return ngaps;
}

template <class PixelType>
void ImageWarp<PixelType>::warp_inverse(Mapping_2d_2d& map,
                                        const vil_memory_image_of<PixelType>&in,
                                        vil_memory_image_of<PixelType>& out)
{
  int w = in.width();
  int h = in.height();

  int out_w = out.width();
  int out_h = out.height();

  int out_offset_x = (w - out_w) / 2;
  int out_offset_y = (h - out_h) / 2;

  for(int oy = 0; oy < out_h; ++oy)
    for(int ox = 0; ox < out_w; ++ox) {
      // *** Find (ix, iy) from (ox,oy)

      // rdc correct
      double ixd, iyd;
      map.inverse_map(double(ox + out_offset_x), double(oy + out_offset_y), &ixd, &iyd);

//    switch (1) {
//    case 1: {
        // nearest neigbour
        int ix = vnl_math_rnd(ixd);
        int iy = vnl_math_rnd(iyd);
        if (in.in_range(ix, iy))
          out(ox, oy) = in(ix,iy);
        else
          out(ox, oy) = vnl_numeric_traits<PixelType>::zero;
//	break;
//    }
//    case 2: {
//	// bilinear
//	out(ox, oy) = vbl_clamp(in.bilinear(ixd, iyd), (PixelType*)0);
//	break;
//      }
//    case 3: {
//	out(ox, oy) = clamp(in.bicubic(ixd, iyd), (PixelType*)0);
//	break;
//    }
//    }
    }
}
