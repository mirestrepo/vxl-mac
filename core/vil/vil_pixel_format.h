// This is core/vil2/vil2_pixel_format.h
#ifndef vil2_pixel_format_h_
#define vil2_pixel_format_h_
//:
// \file
// \author Ian Scott.

#include <vil2/vil2_rgb.h>
#include <vil2/vil2_rgba.h>
#include <vxl_config.h> // for vxl_uint_32 etc.
#include <vcl_iosfwd.h>
#include <vcl_complex.h>

//: Describes the type of the concrete data.
enum vil2_pixel_format {
  VIL2_PIXEL_FORMAT_UNKNOWN = 0,

//  VIL2_PIXEL_FORMAT_UINT_64 = 1,
//  VIL2_PIXEL_FORMAT_INT_64 = 2,
  VIL2_PIXEL_FORMAT_UINT_32 = 3,
  VIL2_PIXEL_FORMAT_INT_32 = 4,
  VIL2_PIXEL_FORMAT_UINT_16 = 5,
  VIL2_PIXEL_FORMAT_INT_16 = 6,
  VIL2_PIXEL_FORMAT_BYTE = 7,
  VIL2_PIXEL_FORMAT_SBYTE = 8,
  VIL2_PIXEL_FORMAT_FLOAT = 9,
  VIL2_PIXEL_FORMAT_DOUBLE = 10,
//  VIL2_PIXEL_FORMAT_LONG_DOUBLE = 11,
  VIL2_PIXEL_FORMAT_BOOL = 12,

//  VIL2_PIXEL_FORMAT_RGB_UINT_64 = 13,
//  VIL2_PIXEL_FORMAT_RGB_INT_64 = 14,
  VIL2_PIXEL_FORMAT_RGB_UINT_32 = 15,
  VIL2_PIXEL_FORMAT_RGB_INT_32 = 16,
  VIL2_PIXEL_FORMAT_RGB_UINT_16 = 17,
  VIL2_PIXEL_FORMAT_RGB_INT_16 = 18,
  VIL2_PIXEL_FORMAT_RGB_BYTE = 19,
  VIL2_PIXEL_FORMAT_RGB_SBYTE = 20,
  VIL2_PIXEL_FORMAT_RGB_FLOAT = 21,
  VIL2_PIXEL_FORMAT_RGB_DOUBLE = 22,
//  VIL2_PIXEL_FORMAT_RGB_LONG_DOUBLE = 23,

//  VIL2_PIXEL_FORMAT_RGBA_UINT_64 = 24,
//  VIL2_PIXEL_FORMAT_RGBA_INT_64 = 25,
  VIL2_PIXEL_FORMAT_RGBA_UINT_32 = 26,
  VIL2_PIXEL_FORMAT_RGBA_INT_32 = 27,
  VIL2_PIXEL_FORMAT_RGBA_UINT_16 = 28,
  VIL2_PIXEL_FORMAT_RGBA_INT_16 = 29,
  VIL2_PIXEL_FORMAT_RGBA_BYTE = 30,
  VIL2_PIXEL_FORMAT_RGBA_SBYTE = 31,
  VIL2_PIXEL_FORMAT_RGBA_FLOAT = 32,
  VIL2_PIXEL_FORMAT_RGBA_DOUBLE = 33,
//  VIL2_PIXEL_FORMAT_RGBA_LONG_DOUBLE = 34,

  VIL2_PIXEL_FORMAT_COMPLEX_FLOAT = 35,
  VIL2_PIXEL_FORMAT_COMPLEX_DOUBLE = 36,

// Add values here and be careful to keep values in vil2_pixel_format.cxx in sync
// Don't forget to increase the end value. Also add to vil2_convert_cast in vil2_convert.h

  VIL2_PIXEL_FORMAT_ENUM_END = 37
};


//: The pixel format enumeration corresponding to the C++ type.
//
template <class T>
inline vil2_pixel_format vil2_pixel_format_of(T dummy) { return VIL2_PIXEL_FORMAT_UNKNOWN;}


//: The C++ type corresponding to a pixel format enumeration.
// Use like
// \code
//    typedef vil2_pixel_format_type_of<VIL2_PIXEL_FORMAT_BYTE>::type byte_type;
// \endcode
// This is specialized for each pixel type enumeration for which a C++
// type exists.
//
template <vil2_pixel_format pix_type>
struct vil2_pixel_format_type_of {
};


VCL_DEFINE_SPECIALIZATION
struct vil2_pixel_format_type_of<VIL2_PIXEL_FORMAT_UNKNOWN> {
  // no type associated with unknown
  // typedef void type;
};

//: Get the vil2_pixel_format value for a given type.
#define vil2_pixel_format_macro(T,V)\
VCL_DEFINE_SPECIALIZATION inline vil2_pixel_format vil2_pixel_format_of(T /*dummy*/) { return V; }\
VCL_DEFINE_SPECIALIZATION struct vil2_pixel_format_type_of<V> { typedef T type; };

vil2_pixel_format_macro(vxl_uint_32, VIL2_PIXEL_FORMAT_UINT_32)
vil2_pixel_format_macro(vxl_int_32,  VIL2_PIXEL_FORMAT_INT_32)
vil2_pixel_format_macro(vxl_uint_16, VIL2_PIXEL_FORMAT_UINT_16)
vil2_pixel_format_macro(vxl_int_16,  VIL2_PIXEL_FORMAT_INT_16)
vil2_pixel_format_macro(vxl_byte,    VIL2_PIXEL_FORMAT_BYTE)
vil2_pixel_format_macro(vxl_sbyte,   VIL2_PIXEL_FORMAT_SBYTE)
vil2_pixel_format_macro(float,       VIL2_PIXEL_FORMAT_FLOAT)
vil2_pixel_format_macro(double,      VIL2_PIXEL_FORMAT_DOUBLE)
vil2_pixel_format_macro(bool,        VIL2_PIXEL_FORMAT_BOOL)

vil2_pixel_format_macro(vil2_rgb<vxl_uint_32>, VIL2_PIXEL_FORMAT_RGB_UINT_32)
vil2_pixel_format_macro(vil2_rgb<vxl_int_32>,  VIL2_PIXEL_FORMAT_RGB_INT_32)
vil2_pixel_format_macro(vil2_rgb<vxl_uint_16>, VIL2_PIXEL_FORMAT_RGB_UINT_16)
vil2_pixel_format_macro(vil2_rgb<vxl_int_16>,  VIL2_PIXEL_FORMAT_RGB_INT_16)
vil2_pixel_format_macro(vil2_rgb<vxl_byte>,    VIL2_PIXEL_FORMAT_RGB_BYTE)
vil2_pixel_format_macro(vil2_rgb<vxl_sbyte>,   VIL2_PIXEL_FORMAT_RGB_SBYTE)
vil2_pixel_format_macro(vil2_rgb<float>,       VIL2_PIXEL_FORMAT_RGB_FLOAT)
vil2_pixel_format_macro(vil2_rgb<double>,      VIL2_PIXEL_FORMAT_RGB_DOUBLE)

vil2_pixel_format_macro(vil2_rgba<vxl_uint_32>, VIL2_PIXEL_FORMAT_RGBA_UINT_32)
vil2_pixel_format_macro(vil2_rgba<vxl_int_32>,  VIL2_PIXEL_FORMAT_RGBA_INT_32)
vil2_pixel_format_macro(vil2_rgba<vxl_uint_16>, VIL2_PIXEL_FORMAT_RGBA_UINT_16)
vil2_pixel_format_macro(vil2_rgba<vxl_int_16>,  VIL2_PIXEL_FORMAT_RGBA_INT_16)
vil2_pixel_format_macro(vil2_rgba<vxl_byte>,    VIL2_PIXEL_FORMAT_RGBA_BYTE)
vil2_pixel_format_macro(vil2_rgba<vxl_sbyte>,   VIL2_PIXEL_FORMAT_RGBA_SBYTE)
vil2_pixel_format_macro(vil2_rgba<float>,       VIL2_PIXEL_FORMAT_RGBA_FLOAT)
vil2_pixel_format_macro(vil2_rgba<double>,      VIL2_PIXEL_FORMAT_RGBA_DOUBLE)

vil2_pixel_format_macro(vcl_complex<float>,   VIL2_PIXEL_FORMAT_COMPLEX_FLOAT)
vil2_pixel_format_macro(vcl_complex<double>,  VIL2_PIXEL_FORMAT_COMPLEX_DOUBLE)

#undef vil2_pixel_format_macro

//: Return the number of bytes used by each component of pixel format f
unsigned vil2_pixel_format_sizeof_components(enum vil2_pixel_format f);

//: Return the number of components in pixel format f
unsigned vil2_pixel_format_num_components(enum vil2_pixel_format f);

//: Return the format of each component of pixel format f
vil2_pixel_format vil2_pixel_format_component_format(enum vil2_pixel_format f);

//: Output a pretty string representing the pixel format.
vcl_ostream & operator << (vcl_ostream &os, vil2_pixel_format f);

#endif // vil2_pixel_format_h_
