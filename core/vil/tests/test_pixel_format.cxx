// This is core/vil/tests/test_pixel_format.cxx
#include <testlib/testlib_test.h>
#include <vcl_iostream.h>
#include <vxl_config.h> // for vxl_byte
#include <vil/vil_rgb.h>
#include <vil/vil_pixel_format.h>

int test_pixel_format_main( int /*argc*/, char* /*argv*/[] )
{
  vcl_cout << "**************************\n"
           << " Testing vil_pixel_format\n"
           << "**************************\n";

  TEST("vil_pixel_format_of(bool)", vil_pixel_format_of(bool()), VIL_PIXEL_FORMAT_BOOL);

  TEST("vil_pixel_format_of(float)", vil_pixel_format_of(float()), VIL_PIXEL_FORMAT_FLOAT);

  TEST("vil_pixel_format_of(vil_rgb<float>) has 3 components",
       vil_pixel_format_num_components(vil_pixel_format_of(vil_rgb<float>())), 3);

  TEST("vil_pixel_format_component_format(vxl_byte)",
       vil_pixel_format_component_format(vil_pixel_format_of(vxl_byte())),
       VIL_PIXEL_FORMAT_BYTE);

  TEST("vil_pixel_format_component_format(vil_rgba<vxl_byte>)",
       vil_pixel_format_component_format(vil_pixel_format_of(vil_rgba<vxl_byte>())),
       VIL_PIXEL_FORMAT_BYTE);

  vcl_cout << VIL_PIXEL_FORMAT_RGBA_DOUBLE << '\n' <<
              VIL_PIXEL_FORMAT_RGBA_INT_32 << '\n' <<
              VIL_PIXEL_FORMAT_RGBA_UINT_32 << '\n' <<
              VIL_PIXEL_FORMAT_RGBA_INT_16 << '\n' <<
              VIL_PIXEL_FORMAT_RGBA_UINT_16 << '\n' <<
              VIL_PIXEL_FORMAT_RGBA_BYTE << '\n' <<
              VIL_PIXEL_FORMAT_RGBA_SBYTE << '\n' <<
              VIL_PIXEL_FORMAT_RGBA_FLOAT << '\n' <<
              VIL_PIXEL_FORMAT_RGBA_DOUBLE << vcl_endl;
  return testlib_test_summary();
}
