#ifdef __GNUC__
#pragma implementation
#endif

#include "vil_viff.h"

char const* vil_viff_format_tag = "viff";

#include <vcl/vcl_iostream.h>
#include <vcl/vcl_cstring.h>

#include <vil/vil_stream.h>
#include <vil/vil_image_impl.h>
#include <vil/vil_image.h>

extern "C" vil_viff_xvimage *vil_viff_createimage
(long col_size, long row_size, long data_storage_type, long num_of_images,
 long num_data_bands, char * comment, long map_row_size, long map_col_size,
 long map_scheme, long map_storage_type, long location_type, 
 long location_dim);
extern "C" void vil_viff_freeimage (vil_viff_xvimage *);

static inline void swap(void* p,int length)
{
  char* t = (char*)p;
  for (int j=0;j<length/2;++j)
  {
    char c = t[j];
    t[j] = t[length-j-1];
    t[length-j-1] = c;
  }
}

vil_image_impl* vil_viff_file_format::make_input_image(vil_stream* is)
{
  // Attempt to read header
  char magic[2];
  is->read(magic, 2);
  bool ok = (magic[0] == (char)XV_FILE_MAGIC_NUM &&
	     magic[1] == (char)XV_FILE_TYPE_XVIFF );
  if (!ok) return 0;
  return new vil_viff_generic_image(is);
}

vil_image_impl* vil_viff_file_format::make_output_image(vil_stream* is, int planes,
					       int width,
					       int height,
					       int components,
					       int bits_per_component,
					       vil_component_format format)
{
  return new vil_viff_generic_image(is, planes, width, height, components, bits_per_component, format);
}

char const* vil_viff_file_format::tag() const
{
  return vil_viff_format_tag;
}

/////////////////////////////////////////////////////////////////////////////

vil_viff_generic_image::vil_viff_generic_image(vil_stream* is)
  : is_(is)
{
  is_->ref();
  read_header();
}

char const* vil_viff_generic_image::file_format() const
{
  return vil_viff_format_tag;
}

vil_viff_generic_image::vil_viff_generic_image(vil_stream* is, int planes,
					       int width,
					       int height,
					       int components,
					       int bits_per_component,
					       vil_component_format format)
  : is_(is), width_(width), height_(height),
    maxval_(255), planes_(planes), start_of_data_(0),
    bits_per_component_(bits_per_component),
    format_(format), endian_consistent_(true)
{
  is_->ref();
  write_header();
}

vil_viff_generic_image::~vil_viff_generic_image()
{
  is_->unref();
}

bool vil_viff_generic_image::read_header()
{
  // Go to start
  is_->seek(0);
  start_of_data_ = VIFF_HEADERSIZE;

  // Read header
  if (VIFF_HEADERSIZE != is_->read((void*)(&header_),VIFF_HEADERSIZE))
    return false;

  if (header_.identifier != (char)XV_FILE_MAGIC_NUM ||
      header_.file_type != (char)XV_FILE_TYPE_XVIFF)
    return false;
  
  check_endian();
  
  //Copy width and hight from header 
  maxval_ = 0;
  
  unsigned long rs = header_.row_size;
  unsigned long cs = header_.col_size;
  unsigned long dst = header_.data_storage_type;
  unsigned long ndb = header_.num_data_bands;

  if(!endian_consistent_)
  {
    swap(&rs,sizeof(rs));
    swap(&cs,sizeof(cs));
    swap(&dst,sizeof(dst));
    swap(&ndb,sizeof(ndb));
  }
  
  width_ = rs;
  height_ = cs;
  
  // decide on data storage type
  switch (dst)
  {
    case VFF_TYP_BIT:
      format_ = VIL_COMPONENT_FORMAT_UNSIGNED_INT;
      bits_per_component_ = 1;
      break;
    case VFF_TYP_1_BYTE:
      format_ = VIL_COMPONENT_FORMAT_UNSIGNED_INT;
      bits_per_component_ = 8;
      break;
    case VFF_TYP_2_BYTE:
      format_ = VIL_COMPONENT_FORMAT_UNSIGNED_INT;
      bits_per_component_ = 16;
      break;
    case VFF_TYP_4_BYTE:
      format_ = VIL_COMPONENT_FORMAT_UNSIGNED_INT;
      bits_per_component_ = 32;
      break;
    case VFF_TYP_FLOAT:
      format_ = VIL_COMPONENT_FORMAT_IEEE_FLOAT;
      bits_per_component_ = 32;
      break;
    case VFF_TYP_DOUBLE:
      format_ = VIL_COMPONENT_FORMAT_IEEE_FLOAT;
      bits_per_component_ = 64;
      break;
    case VFF_TYP_COMPLEX:
      format_ = VIL_COMPONENT_FORMAT_COMPLEX;
      bits_per_component_ = 64;
      break;
    case VFF_TYP_DCOMPLEX:
      format_ = VIL_COMPONENT_FORMAT_COMPLEX;
      bits_per_component_ = 128;
      break;
    default:
      vcl_cout << "vil_viff: non supported data type: VFF_TYP " 
	   <<   header_.data_storage_type << vcl_endl;      
      format_ = VIL_COMPONENT_FORMAT_UNKNOWN;
      return false;
  }

  // number of colour bands
  planes_ = ndb;

  return true;
}

bool vil_viff_generic_image::write_header()
{
  is_->seek(0);
  int type = 0;
  if (bits_per_component_ > 1) 
    bits_per_component_ = 8*((bits_per_component_+7)/8); // round to next 8-tuple

  if (component_format()==VIL_COMPONENT_FORMAT_SIGNED_INT ||
      component_format()==VIL_COMPONENT_FORMAT_UNSIGNED_INT)
  {
    switch (bits_per_component_) 
    {
      case 1: type=VFF_TYP_BIT; break;
      case 8: type=VFF_TYP_1_BYTE; break;
      case 16: type=VFF_TYP_2_BYTE; break;
      case 32: type=VFF_TYP_4_BYTE; break;
      default:
        vcl_cerr << "vil_viff: non supported data type: " << bits_per_component_ << " bit pixels\n";
        return false;
    }
  }
  else if (component_format()==VIL_COMPONENT_FORMAT_IEEE_FLOAT)
  {
    switch (bits_per_component_) 
    {
      case 32: type=VFF_TYP_FLOAT; break;
      case 64: type=VFF_TYP_DOUBLE; break;
      default:
        vcl_cerr << "vil_viff: non supported data type: " << bits_per_component_ << " bit float pixels\n";
        return false;
    }
  }
  else if (component_format()==VIL_COMPONENT_FORMAT_COMPLEX)
  {
    switch (bits_per_component_) 
    {
      case 64: type=VFF_TYP_COMPLEX; break;
      case 128: type=VFF_TYP_DCOMPLEX; break;
      default:
        vcl_cerr << "vil_viff: non supported data type: " << bits_per_component_ << " bit complex pixels\n";
        return false;
    }
  }
  else
  {
    vcl_cout << "vil_viff: non supported data type: " << component_format() << vcl_endl;
    return false;
  }

  //create header
  vil_viff_xvimage *imagep = vil_viff_createimage ( height_, width_, 
	type,1,planes_, "TargetJr VIFF image writer output",0,0,
	VFF_MS_NONE,VFF_MAPTYP_NONE,VFF_LOC_IMPLICIT,0);

  //make local copy of header
  memcpy(&header_, imagep, sizeof(header_));
  start_of_data_ = sizeof(header_);

  // release xv header from createimage
  vil_viff_freeimage(imagep);
  
  is_->write((void*)(&header_), start_of_data_);
  start_of_data_ = is_->tell();
  return true;
}

bool vil_viff_generic_image::get_section(void* buf, int x0, int y0, int xs, int ys) const
{
  unsigned char* ib = (unsigned char*) buf;
  unsigned long rowsize = bits_per_component_*xs/8;
  unsigned long tbytes = rowsize*ys*planes_;
  for(int p = 0; p<planes_; ++p) {
    for (int y = y0; y < y0+ys; ++y) {
      is_->seek(start_of_data_ + p*width_*height_*bits_per_component_/8 + bits_per_component_*(y*width_+x0)/8);
      is_->read(ib, rowsize);
      ib += rowsize;
    }
  }
  if(!endian_consistent_)
    for (unsigned int i=0;i<tbytes;i+=bits_per_component_/8)
      swap(ib+i,bits_per_component_/8);

  return true;
}

bool vil_viff_generic_image::put_section(void const* buf, int x0, int y0, int xs, int ys)
{
  unsigned char const* ob = (unsigned char const*) buf;
  unsigned long rowsize = bits_per_component_*xs/8;
  for(int p = 0; p<planes_; ++p) {
    for (int y = y0; y < y0+ys; ++y) {
      is_->seek(start_of_data_ + p*width_*height_*bits_per_component_/8 + bits_per_component_*(y*width_+x0)/8);
      is_->write(ob, rowsize);
      ob += rowsize;
    }
  }
  return true;
}

bool vil_viff_generic_image::check_endian()
{
  // check if format is consistent
  // Check the data_storage_type in the header
  // If it is between 1 and 255, the "Endian" is consistent with the system
  // if not, we swap and check again

  long dst = header_.data_storage_type;

  endian_consistent_ = ((dst & 0xff) != 0);
  if (endian_consistent_)
    vcl_cout << "Endian is Consistent" << vcl_endl;
  else
    vcl_cout << "Endian is NOT Consistent" << vcl_endl;
  return endian_consistent_;
}

vil_image vil_viff_generic_image::get_plane(int plane) const 
{
  vcl_cerr << "FIXME: this should be an adapter that shifts to the plane asked for\n";
  return const_cast<vil_viff_generic_image*>(this);
}

bool vil_viff_generic_image::get_section_rgb_byte(void* buf, int x0, int y0, int width, int height) const
{
  vcl_cerr << "FIXME: vil_viff_generic_image::get_section_rgb_byte() not yet implemented\n";
  return false;
}

bool vil_viff_generic_image::get_section_float(void* buf, int x0, int y0, int width, int height) const
{
  if (component_format()!=VIL_COMPONENT_FORMAT_IEEE_FLOAT || bits_per_component_ != 32) 
    return false;
  return get_section(buf,x0,y0,width,height);
}

bool vil_viff_generic_image::get_section_byte(void* buf, int x0, int y0, int width, int height) const
{
  if (component_format()!=VIL_COMPONENT_FORMAT_UNSIGNED_INT || bits_per_component_ != 8) 
    return false;
  return get_section(buf,x0,y0,width,height);
}

