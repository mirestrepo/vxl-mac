// This is mul/vil2/tests/test_file_format_read.cxx
#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_vector.h>
#include <vcl_cassert.h>

#include <vxl_config.h> // for vxl_uint_16 etc.

#include <testlib/testlib_test.h>

#include <vil/vil_rgb.h>
#include <vil2/vil2_load.h>
#include <vil2/vil2_image_view.h>
#include <vil2/vil2_print.h>

#define DEBUG

// Amitha Perera
// Apr 2002

// Compare the results of loading different files with the true data
// that's supposed to be in those files. Only deals with single plane
// images so far.

vcl_string image_base;

typedef vxl_uint_32 TruePixelType;

class CheckPixel
{
 public:
  virtual ~CheckPixel() { }
  virtual bool operator() ( int p, int i, int j, const vcl_vector<TruePixelType>& pixel ) const = 0;
};

template<class T>
class CheckRGB : public CheckPixel
{
 public:
  CheckRGB( const char* file )
  {
    vil2_image_view_base_sptr im = vil2_load( (image_base + file).c_str() );
    if ( !im )
      vcl_cout << "[ couldn't load " << file << "]\n";
    else
      img_ = im;
#ifdef DEBUG
    vcl_cout << '\n'; vil2_print_all(vcl_cout, img_);
#endif
  }

  bool operator() ( int p, int i, int j, const vcl_vector<TruePixelType>& pixel ) const
  {
    assert( p == 0 );
    return img_ && pixel.size() == 3 && pixel[0] == img_(i,j).r && pixel[1] == img_(i,j).g && pixel[2] == img_(i,j).b;
  }
 protected:
  vil2_image_view< vil_rgb<T> > img_;
};

template<class T>
class CheckRGBNear : public CheckRGB<T>
{
 public:
  CheckRGBNear( const char* file, TruePixelType tol):
    CheckRGB<T>(file), tol_sq_(tol*tol) {}

  bool operator() ( int p, int i, int j, const vcl_vector<TruePixelType>& pixel ) const
  {
    assert( p == 0 );
    if (!( img_ && pixel.size() == 3)) return false;
    // Find difference in two values whilst avoiding unsigned underflow
    // Find difference in two values whilst avoiding unsigned underflow
    const TruePixelType diff_A = pixel[0]*pixel[0] +
      (TruePixelType)img_(i,j).r * (TruePixelType)img_(i,j).r + pixel[1]*pixel[1] +
      (TruePixelType)img_(i,j).g * (TruePixelType)img_(i,j).g + pixel[2]*pixel[2] +
      (TruePixelType)img_(i,j).b * (TruePixelType)img_(i,j).b;

    const TruePixelType diff_B =
      2 * pixel[0] * (TruePixelType)img_(i,j).r + (TruePixelType)tol_sq_ +
      2 * pixel[1] * (TruePixelType)img_(i,j).g + (TruePixelType)tol_sq_ +
      2 * pixel[2] * (TruePixelType)img_(i,j).b + (TruePixelType)tol_sq_ ;
    return diff_A < diff_B;
  }
 protected:
  TruePixelType tol_sq_;
};

template<class T>
class CheckColourPlanes : public CheckPixel
{
 public:
  CheckColourPlanes( const char* file )
  {
    vil2_image_view_base_sptr im = vil2_load( (image_base + file).c_str() );
    if ( !im )
      vcl_cout << "[ couldn't load " << file << "]\n";
    else
    {
      img_ = im;
#ifdef DEBUG
      vcl_cout << '\n'; vil2_print_all(vcl_cout, img_);
#endif
    }
  }

  bool operator() ( int p, int i, int j, const vcl_vector<TruePixelType>& pixel ) const
  {
    return img_ && pixel.size() == 1 && pixel[0] == img_(i,j,p);
  }
 protected:
  vil2_image_view< T > img_;
};

template<class T>
class CheckGrey : public CheckPixel
{
 public:
  CheckGrey( const char* file )
  {
    vil2_image_view_base_sptr im = vil2_load( (image_base + file).c_str() );
    if ( !im )
      vcl_cout << "[ couldn't load " << file << "]\n";
    else
    {
      img_ = im;
#ifdef DEBUG
      vcl_cout << '\n'; vil2_print_all(vcl_cout, img_);
#endif
    }
  };

  bool operator() ( int p, int i, int j, const vcl_vector<TruePixelType>& pixel ) const
  {
    assert( p == 0 );
    return img_
      && pixel.size() == 1 &&
      pixel[0] == (TruePixelType)img_(i,j);
  }
 protected:
  vil2_image_view< T > img_;
};

template<class T>
class CheckGreyNear : public CheckGrey<T>
{
 public:
  CheckGreyNear( const char* file, TruePixelType Tol)
    : CheckGrey<T>(file), tol_sq_(Tol*Tol) {}

  bool operator() ( int p, int i, int j, const vcl_vector<TruePixelType>& pixel ) const
  {
    assert( p == 0 );
    if (!( img_
      && pixel.size() == 1)) return false;
    // Find difference in two values whilst avoiding unsigned underflow
    const TruePixelType diff_A = pixel[0]*pixel[0] +
      (TruePixelType)img_(i,j) * (TruePixelType)img_(i,j);
    const TruePixelType diff_B = 2 * pixel[0] * (TruePixelType)img_(i,j) + (TruePixelType)tol_sq_ ;
    return diff_A <= diff_B;
  }
 protected:
  TruePixelType tol_sq_;
};

template class CheckRGB< vxl_byte >;
template class CheckRGB< vxl_uint_16 >;
template class CheckColourPlanes< vxl_byte >;
template class CheckGrey< vxl_uint_16 >;
template class CheckGrey< vxl_byte >;
template class CheckGrey< bool >;


bool
test( const char* true_data_file, const CheckPixel& check )
{
  // The true data is a ASCII file consisting of a sequence of numbers. The first set of numbers are:
  //    number of planes (P)
  //    number of components (C)
  //    width (in pixels, not components)
  //    height (in pixels, not components)
  //    planes*width*height*components of data, in the following order:
  //        plane1pix1comp1 plane1pix1comp2 ... plane1pixNcompC ... planePpix1comp1 ... planePpixNcompC
  //      where N = width*height

  int num_planes;
  int num_comp;
  int width;
  int height;

  vcl_ifstream fin( (image_base+true_data_file).c_str() );
  if ( !( fin >> num_planes >> num_comp >> width >> height ) ) {
    vcl_cout << "[couldn't read header from " << true_data_file << "]";
    return false;
  }

  vcl_vector<TruePixelType> pixel( num_comp );

  for ( int p=0; p < num_planes; ++p ) {
    for ( int j=0; j < height; ++j ) {
      for ( int i=0; i < width; ++i ) {
        for ( int c=0; c < num_comp; ++c ) {
          if ( !( fin >> pixel[c] ) ) {
            vcl_cout << "[couldn't read value at " << p << "," << i << "," << j << "," << c
                     << " from " << true_data_file << "]";
            return false;
          }
        }
        if ( !check( p, i, j, pixel ) )
          return false;
      }
    }
  }

  return true;
}

int
test_file_format_read_main( int argc, char* argv[] )
{
  if ( argc >= 2 ) {
    image_base = argv[1];
#ifdef VCL_WIN32
    image_base += "\\";
#else
    image_base += "/";
#endif
  }

  testlib_test_start(" file format read");

  vcl_cout << "Portable aNy Map [pnm]: pbm, pgm, ppm)\n";
  testlib_test_begin( "  1-bit pbm ascii" );
  testlib_test_perform( test( "ff_grey1bit_true.txt", CheckGrey<bool>( "ff_grey1bit_ascii.pbm" ) ) );
  testlib_test_begin( "  1-bit pbm raw" );
  testlib_test_perform( test( "ff_grey1bit_true.txt", CheckGrey<bool>( "ff_grey1bit_raw.pbm" ) ) );
  testlib_test_begin( "  8-bit pgm ascii" );
  testlib_test_perform( test( "ff_grey8bit_true.txt", CheckGrey<vxl_byte>( "ff_grey8bit_ascii.pgm" ) ) );
  testlib_test_begin( "  8-bit pgm raw" );
  testlib_test_perform( test( "ff_grey8bit_true.txt", CheckGrey<vxl_byte>( "ff_grey8bit_raw.pgm" ) ) );
  testlib_test_begin( " 16-bit pgm ascii" );
  testlib_test_perform( test( "ff_grey16bit_true.txt", CheckGrey<vxl_uint_16>( "ff_grey16bit_ascii.pgm" ) ) );
  testlib_test_begin( " 16-bit pgm raw" );
  testlib_test_perform( test( "ff_grey16bit_true.txt", CheckGrey<vxl_uint_16>( "ff_grey16bit_raw.pgm" ) ) );
  testlib_test_begin( "  8-bit ppm ascii" );
  testlib_test_perform( test( "ff_rgb8bit_true.txt", CheckRGB<vxl_byte>( "ff_rgb8bit_ascii.ppm" ) ) );
  testlib_test_begin( "  8-bit ppm ascii as planar" );
  testlib_test_perform( test( "ff_planar8bit_true.txt", CheckColourPlanes<vxl_byte>( "ff_rgb8bit_ascii.ppm" ) ) );
  testlib_test_begin( "  8-bit ppm raw" );
  testlib_test_perform( test( "ff_rgb8bit_true.txt", CheckRGB<vxl_byte>( "ff_rgb8bit_raw.ppm" ) ) );
  testlib_test_begin( "  8-bit ppm raw as planar" );
  testlib_test_perform( test( "ff_planar8bit_true.txt", CheckColourPlanes<vxl_byte>( "ff_rgb8bit_raw.ppm" ) ) );
  testlib_test_begin( " 16-bit ppm ascii" );
  testlib_test_perform( test( "ff_rgb16bit_true.txt", CheckRGB<vxl_uint_16>( "ff_rgb16bit_ascii.ppm" ) ) );
  testlib_test_begin( " 16-bit ppm raw" );
  testlib_test_perform( test( "ff_rgb16bit_true.txt", CheckRGB<vxl_uint_16>( "ff_rgb16bit_raw.ppm" ) ) );

  vcl_cout << "JPEG [jpg]\n";
  testlib_test_begin( "  8-bit grey, normal image to 4 quanta" );
  testlib_test_perform( test( "ff_grey8bit_true.txt",
    CheckGreyNear<vxl_byte>( "ff_grey8bit_compressed.jpg", 4 ) ) );
  testlib_test_begin( "  8-bit RGB, easy image accurate to 3 quanta" );
  testlib_test_perform( test( "ff_rgb8biteasy_true.txt",
    CheckRGBNear<vxl_byte>( "ff_rgb8biteasy_compressed.jpg", 3 ) ) );

  vcl_cout << "Windows bitmap [bmp]\n";
  testlib_test_begin( "  8-bit greyscale (xv created)" );
  testlib_test_perform( test( "ff_grey8bit_true.txt", CheckGrey<vxl_byte>( "ff_grey8bit.bmp" ) ) );
  testlib_test_begin( "  8-bit RGB (xv created)" );
  testlib_test_perform( test( "ff_planar8bit_true.txt", CheckColourPlanes<vxl_byte>( "ff_rgb8bit_xv.bmp" ) ) );

  vcl_cout << "Portable Network Graphics [png]\n";
  testlib_test_begin( "  8-bit RGB uncompressed" );
  testlib_test_perform( test( "ff_rgb8bit_true.txt", CheckRGB<vxl_byte>( "ff_rgb8bit_uncompressed.png" ) ) );
  testlib_test_begin( "  8-bit RGB compressed" );
  testlib_test_perform( test( "ff_rgb8bit_true.txt", CheckRGB<vxl_byte>( "ff_rgb8bit_compressed.png" ) ) );

#if 0
  testlib_test_begin( "  8-bit indexed RGB" );
  testlib_test_perform( test( "ff_rgb8bit_true.txt", CheckRGB<vxl_byte>( "ff_rgb8bit_indexed.ras" ) ) );


  vcl_cout << "Sun raster [ras]\n";
  testlib_test_begin( "  8-bit grey, no colourmap" );
  testlib_test_perform( test( "ff_grey8bit_true.txt", CheckGrey<vxl_byte>( "ff_grey8bit_nocol.ras" ) ) );
  testlib_test_begin( "  8-bit RGB, no colourmap" );
  testlib_test_perform( test( "ff_rgb8bit_true.txt", CheckRGB<vxl_byte>( "ff_rgb8bit_raw.ras" ) ) );
  testlib_test_begin( "  8-bit indexed RGB" );
  testlib_test_perform( test( "ff_rgb8bit_true.txt", CheckRGB<vxl_byte>( "ff_rgb8bit_indexed.ras" ) ) );

  vcl_cout << "TIFF [tiff]\n";
  testlib_test_begin( "  8-bit RGB uncompressed" );
  testlib_test_perform( test( "ff_rgb8bit_true.txt", CheckRGB<vxl_byte>( "ff_rgb8bit_uncompressed.tif" ) ) );
  testlib_test_begin( "  8-bit RGB packbits" );
  testlib_test_perform( test( "ff_rgb8bit_true.txt", CheckRGB<vxl_byte>( "ff_rgb8bit_packbits.tif" ) ) );

   vcl_cout << "SGI IRIS [iris]\n";
   testlib_test_begin( "  8-bit RGB rle" );
   testlib_test_perform( test( "ff_planar8bit_true.txt", CheckColourPlanes<vxl_byte>( "ff_rgb8bit.iris" ) ) );

  vcl_cout << "Portable Network Graphics [png]\n";
  testlib_test_begin( "  8-bit RGB uncompressed" );
  testlib_test_perform( test( "ff_rgb8bit_true.txt", CheckRGB<vxl_byte>( "ff_rgb8bit_uncompressed.png" ) ) );
  testlib_test_begin( "  8-bit RGB compressed" );
  testlib_test_perform( test( "ff_rgb8bit_true.txt", CheckRGB<vxl_byte>( "ff_rgb8bit_compressed.png" ) ) );
#endif
  return testlib_test_summary();
}
