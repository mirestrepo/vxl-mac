
#include <vcl/vcl_cassert.h>

#include <vil/vil_warp.h>
#include <vil/vil_warp.txx>

#include <vil/vil_rgb_byte.h>
#include <vil/vil_pixel.h>

VIL_WARP_INSTANTIATE(unsigned char, vil_warp_mapping);
VIL_WARP_INSTANTIATE(vil_rgb_byte, vil_warp_mapping);

vil_image vil_warp(vil_image const& in, vil_warp_mapping const& mapper,
		   vil_warp_interpolation_type i)
{
  if (vil_pixel_type(in) == VIL_BYTE) {
    vil_memory_image_of<unsigned char> inimg(in);
    vil_memory_image_of<unsigned char> outimg(in.width(), in.height());
    vil_warp_output_driven(inimg, outimg, mapper, i);
    return outimg;

  } else if (vil_pixel_type(in) == VIL_RGB_BYTE) {
    vil_memory_image_of<vil_rgb_byte> inimg(in);
    vil_memory_image_of<vil_rgb_byte> outimg(in.width(), in.height());
    vil_warp_output_driven(inimg, outimg, mapper, i);
    return outimg;
    
  } else {
    assert(0);
  }
}
