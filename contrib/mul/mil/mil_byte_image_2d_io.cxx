#ifdef __GNUC__
#pragma implementation
#endif

//: \file
//  \brief Load and save mil_image_2d_of<vil_byte> from named files.
//  \author Tim Cootes

#include <vcl_cstdlib.h>
#include <vcl_string.h>
#include <vsl/vsl_indent.h>
#include <mil/mil_byte_image_2d_io.h>
#include <vcl_cassert.h>

#include <vil/vil_load.h>
#include <vil/vil_save.h>
#include <vil/vil_memory_image_of.h>
#include <vil/vil_rgb_byte.h>

//=======================================================================

mil_byte_image_2d_io::mil_byte_image_2d_io()
{
}

//=======================================================================

mil_byte_image_2d_io::~mil_byte_image_2d_io()
{
}

//: Define whether to load images as colour or grey-scale
//  Options are '' (ie rely on image), 'Grey' or 'RGB'
void mil_byte_image_2d_io::setColour(const vcl_string& c)
{
  colour_ = c;
}

//: Whether to load images as RGB, Grey-scale or leave to image format
const vcl_string& mil_byte_image_2d_io::colour() const
{
  return colour_;
}

//: Current image
//  (The one generated by last call to b_read())
const mil_image& mil_byte_image_2d_io::image() const
{
  return image_;
}

//: Attempt to load image from named file
// \param filetype  String hinting at what image format is
// \return true if successful
bool mil_byte_image_2d_io::loadImage(const vcl_string& path,
              const vcl_string& filetype)
{
  return loadTheImage(image_,path,filetype);
}

//: Attempt to save image to named file
// \param filetype  String defining what format to save in
// \return true if successful
bool mil_byte_image_2d_io::saveImage(const vcl_string& path,
              const vcl_string& filetype) const
{
  return saveTheImage(image_,path,filetype);
}


vcl_string mil_byte_image_2d_io::guessFileType(const vcl_string& path) const
{
  int n = path.size();
  if (n<=4) return "Unknown";

  const char* ext = path.c_str()+n-4;
  vcl_string s_ext= ext;

  if (s_ext == ".BMP" || s_ext==".bmp")
    return "bmp";
  else
  if (s_ext == ".ras" || s_ext==".RAS")
    return "sunras";
  else
  if (s_ext == ".rad")
    return "radial";
  else
  if (s_ext == ".gif" || s_ext==".GIF")
    return "gif";
  else
  if (s_ext == ".jpg" || s_ext==".JPG")
    return "jpeg";

  return "Unknown";
}

void mil_byte_image_2d_io::copyAsGrey(vil_image& img)
{
  int nx = img.width();
  int ny = img.height();
  vil_memory_image_of<vil_byte> buf(img);
  image_.set_n_planes(1);
  image_.resize(nx,ny);
  // Inefficient copy
  for (int y=0;y<ny;++y)
    for (int x=0;x<nx;++x)
    image_(x,y)=buf(x,y);
}

void mil_byte_image_2d_io::copyAsRGB(vil_image& img)
{
   int nx = img.width();
   int ny = img.height();
   vil_memory_image_of<vil_rgb_byte> buf(img);
    image_.set_n_planes(3);
  image_.resize(nx,ny);
  // Inefficient copy
  for (int y=0;y<ny;++y)
    for (int x=0;x<nx;++x)
    {
      image_(x,y,0)=buf(x,y).R();
      image_(x,y,1)=buf(x,y).G();
      image_(x,y,2)=buf(x,y).B();
    }
}

void mil_byte_image_2d_io::copyGreyToRGB(vil_image& img)
{
  int nx = img.width();
  int ny = img.height();
  // Assume it is a colour image - copy to greyscale image
  vil_memory_image_of<vil_rgb_byte> buf(img);
  image_.set_n_planes(1);
  image_.resize(nx,ny);
    // Inefficient copy, using a `weighting' on RGB to get the grey-scale
  for (int y=0;y<ny;++y)
    for (int x=0;x<nx;++x)
      image_(x,y)=buf(x,y).grey();
}

//: Attempt to load image from named file
// \param filetype  String hinting at what image format is
//  Return true if successful
bool mil_byte_image_2d_io::loadTheImage(mil_image_2d_of<vil_byte>& image,
              const vcl_string& path,
              const vcl_string& f_type)

{
  vil_image img = vil_load(path.c_str());  // ie f_type is ignored here !!
  int nx = img.width();
  int ny = img.height();
  if (nx==0 || ny==0) return false;

  bool img_is_grey = (img.get_size_bytes()==(nx*ny));

  if (colour_=="Grey")   // ie make a grey image whatever image is loaded
  {
    if (img_is_grey)
      copyAsGrey(img);
    else
      copyGreyToRGB(img);
  }
  else if (colour_=="RGB")
    copyAsRGB(img);
  else    // ie colour_="" => rely on image
    if (img_is_grey)
      copyAsGrey(img);  //copy grey image as grey
    else
      copyAsRGB(img);   //ie copy colour image as colour

  image=image_;
  return true;
}


//: Attempt to save image to named file
// \param filetype  String defining what format to save in
bool mil_byte_image_2d_io::saveTheImage(const mil_image_2d_of<vil_byte>& image,
            const vcl_string& path,
            const vcl_string& f_type) const
{
  vcl_string filetype = f_type;
  if (f_type=="")
    filetype=guessFileType(path);

  if (filetype=="Unknown")
  {
    vcl_cerr<<"File type for "<<path<<" unknown"<<vcl_endl;
    return false;
  }

  //convert mil_image to vil_image
  //if colour=>vil_memory_image_of<vil_rgb_byte>
  //if grey=>vil_memory_image_of<vil_byte>
  //then use vil_save with appropriate "filetype"

  bool image_is_grey=false,image_is_colour=false;
  if (image.n_planes()==1) image_is_grey=true;
  else if (image.n_planes()==3) image_is_colour=true;
  else
  {
    vcl_cerr<<"Failed to save: number of planes = "<<image.n_planes()<<" ??, require 1 or 3"<<vcl_endl;
    return false;
  }

  int nx=image.nx();
  int ny=image.ny();

  if (image_is_grey)
  {
    vil_memory_image_of<vil_byte> buf(nx,ny);
    // Inefficient copy
    for (int y=0;y<ny;++y)
      for (int x=0;x<nx;++x)
        buf(x,y)= image(x,y);

    vil_save(buf,path.c_str(),filetype.c_str());
    //vil_save(buf,path.c_str());
    return true;
  }

  else if (image_is_colour)
  {
    vil_memory_image_of<vil_rgb_byte> buf(nx,ny);
    vil_byte red,green,blue;
    // Inefficient copy
    for (int y=0;y<ny;++y)
      for (int x=0;x<nx;++x)
      {
        red=image(x,y,0);
        green=image(x,y,1);
        blue=image(x,y,2);
        buf(x,y)=vil_rgb_byte(red,green,blue);
      }
    vil_save(buf,path.c_str(),filetype.c_str());
    //vil_save(buf,path.c_str());
    return true;
  }
  else
  return false;
}


//=======================================================================
// Method: is_a
//=======================================================================

vcl_string mil_byte_image_2d_io::is_a() const
{
  return vcl_string("mil_byte_image_2d_io");
}

//=======================================================================
// Method: is_class
//=======================================================================

bool mil_byte_image_2d_io::is_class(vcl_string const& s) const
{
  static const vcl_string s_ = "mil_byte_image_2d_io";
  return s==s_ || mil_image_io::is_class(s);
}

//=======================================================================
// Method: version_no
//=======================================================================

short mil_byte_image_2d_io::version_no() const
{
  return 1;
}

//=======================================================================
// Method: clone
//=======================================================================

mil_image_io* mil_byte_image_2d_io::clone() const
{
  return new mil_byte_image_2d_io(*this);
}

//=======================================================================
// Method: print
//=======================================================================

void mil_byte_image_2d_io::print_summary(vcl_ostream&) const
{
}

//=======================================================================
// Method: save
//=======================================================================

void mil_byte_image_2d_io::b_write(vsl_b_ostream& bfs) const
{
  vsl_b_write(bfs,version_no());
}

//=======================================================================
// Method: load
//=======================================================================

void mil_byte_image_2d_io::b_read(vsl_b_istream& bfs)
{
  short version;
  vsl_b_read(bfs,version);
  switch (version)
  {
    case (1):
      break;
    default:
      vcl_cerr << "mil_byte_image_2d_io::b_read() ";
      vcl_cerr << "Unexpected version number " << version << vcl_endl;
      vcl_abort();
  }
}
