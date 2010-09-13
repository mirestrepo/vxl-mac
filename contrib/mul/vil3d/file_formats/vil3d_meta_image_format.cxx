// This is mul/vil3d/file_formats/vil3d_meta_image_format.cxx
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma implementation
#endif

//:
// \file
// \brief Reader/Writer for meta image format images.
// \author Chris Wolstenholme - Imorphics Ltd
// This file contains classes for reading and writing meta image format images
// Three key components are
//   vil3d_meta_image_header: Structure to contain header information
//   vil3d_meta_image: Resource object which interfaces to the file,
//                        allowing reading and writing via the get_copy_image()
//                        and put_image() functions
//   vil3d_meta_image_format: Object to create an appropriate vil3d_meta_image
//
//   The main work of loading and saving happens in vil3d_meta_image

#include "vil3d_meta_image_format.h"
#include <vcl_cstring.h>
#include <vcl_iostream.h>
#include <vul/vul_file.h>
#include <vil/vil_stream_fstream.h>
#include <vil3d/vil3d_image_view.h>
#include <vil3d/vil3d_property.h>
#include <vil3d/vil3d_image_resource.h>
#include <vil3d/vil3d_new.h>
#include <vil3d/vil3d_copy.h>

/*
 * Helper functions
 */
inline void vil3d_meta_image_swap16(char *a, unsigned n)
{
  for (unsigned i = 0; i < n * 2; i += 2)
  {
    char c = a[i]; a[i] = a[i+1]; a[i+1] = c;
  }
}

/*
 * Header stuff
 */

//===================================================================
// Header constructor
//===================================================================
vil3d_meta_image_header::vil3d_meta_image_header(void) :
header_valid_(false),
byte_order_msb_(false),
offset_i_(0.0), offset_j_(0.0), offset_k_(0.0),
vox_size_i_(1.0), vox_size_j_(1.0), vox_size_k_(1.0),
dim_size_i_(0), dim_size_j_(0), dim_size_k_(0), nplanes_(1),
need_swap_(false)
{
  // No contruction code
}

//===================================================================
// Header destructor
//===================================================================
vil3d_meta_image_header::~vil3d_meta_image_header(void)
{
  // No destructor code
}

//===================================================================
// Set/get byte order
//===================================================================
void vil3d_meta_image_header::set_byte_order_msb(const bool is_msb)
{
  byte_order_msb_ = is_msb;
}

bool vil3d_meta_image_header::byte_order_is_msb(void) const
{
  return byte_order_msb_;
}

//===================================================================
// Set/get offset
//===================================================================
void vil3d_meta_image_header::set_offset(const double off_i,
                                         const double off_j,
                                         const double off_k)
{
  offset_i_ = off_i;
  offset_j_ = off_j;
  offset_k_ = off_k;
}

double vil3d_meta_image_header::offset_i(void) const
{
  return offset_i_;
}

double vil3d_meta_image_header::offset_j(void) const
{
  return offset_j_;
}

double vil3d_meta_image_header::offset_k(void) const
{
  return offset_k_;
}

//===================================================================
// Set/get voxel sizes
//===================================================================
void vil3d_meta_image_header::set_vox_size(const double vox_i,
                                           const double vox_j,
                                           const double vox_k)
{
  vox_size_i_ = vox_i;
  vox_size_j_ = vox_j;
  vox_size_k_ = vox_k;
}

double vil3d_meta_image_header::vox_size_i(void) const
{
  return vox_size_i_;
}

double vil3d_meta_image_header::vox_size_j(void) const
{
  return vox_size_j_;
}

double vil3d_meta_image_header::vox_size_k(void) const
{
  return vox_size_k_;
}

//===================================================================
// Set/get image dimensions
//===================================================================
void vil3d_meta_image_header::set_dim_size(const unsigned int ni,
                                           const unsigned int nj,
                                           const unsigned int nk,
                                           const unsigned int np)
{
  dim_size_i_ = ni;
  dim_size_j_ = nj;
  dim_size_k_ = nk;
  nplanes_ = np;
}

unsigned int vil3d_meta_image_header::ni(void) const
{
  return dim_size_i_;
}

unsigned int vil3d_meta_image_header::nj(void) const
{
  return dim_size_j_;
}

unsigned int vil3d_meta_image_header::nk(void) const
{
  return dim_size_k_;
}

unsigned int vil3d_meta_image_header::nplanes(void) const
{
  return nplanes_;
}

//===================================================================
// Set/get element type
//===================================================================
void vil3d_meta_image_header::set_element_type(const vcl_string &elem_type)
{
  elem_type_ = elem_type;
}

const vcl_string &vil3d_meta_image_header::element_type(void) const
{
  return elem_type_;
}

//===================================================================
// Set/get image file name
//===================================================================
void vil3d_meta_image_header::set_image_fname(const vcl_string &image_fname)
{
  im_file_ = image_fname;
}

const vcl_string &vil3d_meta_image_header::image_fname(void) const
{
  return im_file_;
}

//===================================================================
// Set/get the pixel format
//===================================================================
void vil3d_meta_image_header::set_pixel_format(const vil_pixel_format format)
{
  pformat_ = format;
}

const vil_pixel_format vil3d_meta_image_header::pixel_format(void) const
{
  return pformat_;
}

//===================================================================
// Set the header back to defaults
//===================================================================
void vil3d_meta_image_header::clear(void)
{
  header_valid_ = false;
  byte_order_msb_ = false;
  offset_i_ = offset_j_ = offset_k_ = 0.0;
  vox_size_i_ = vox_size_j_ = vox_size_k_ = 1.0;
  dim_size_i_ = dim_size_j_ = dim_size_k_ = 0;
  elem_type_ = im_file_ = "";
  need_swap_ = false;
}

//===================================================================
// Read the header
//===================================================================
bool vil3d_meta_image_header::read_header(const vcl_string &header_fname)
{
  vcl_ifstream hfs(header_fname.c_str());

  if (!hfs)
    return false;

  vcl_string nxt_line;
  vcl_getline(hfs,nxt_line);
  while (hfs.good() && !hfs.eof())
  {
    if (!check_next_header_line(nxt_line))
    {
      hfs.close();
      return false;
    }
    vcl_getline(hfs,nxt_line);
  }
  hfs.close();
  if (header_valid_)
  {
    vcl_string pth = vul_file::dirname(header_fname);
    pth += "/" + im_file_;
    im_file_ = pth;
    return true;
  }
  else
    return false;
}

//===================================================================
// Write the header
//===================================================================
bool vil3d_meta_image_header::write_header(const vcl_string &header_fname) const
{
  vcl_ofstream ofs(header_fname.c_str());
  if (!ofs)
    return false;

  ofs << "ObjectType = Image\n";
  ofs << "NDims = 3\n";
  ofs << "BinaryData = True\n";
  ofs << "BinaryDataByteOrderMSB = " << ((byte_order_msb_) ? "True" : "False") << "\n";
  ofs << "CompressedData = False\n";
  ofs << "TransformMatrix = 1 0 0 0 1 0 0 0 1\n";
  ofs << "Offset = " << offset_i_ << " " << offset_j_ << " " << offset_k_ << "\n";
  ofs << "CenterOfRotation = 0 0 0\n";
  ofs << "AnatomicalOrientation = RAI\n";
  ofs << "ElementSpacing = " << vox_size_i_ << " " << vox_size_j_ << " " << vox_size_k_ << "\n";
  ofs << "DimSize = " << dim_size_i_ << " " << dim_size_j_ << " " << dim_size_k_ << "\n";
  ofs << "ElementType = " << elem_type_ << "\n";
  ofs << "ElementDataFile = " << vul_file::strip_directory(im_file_) << "\n";

  ofs.close();
  return true;
}

//===================================================================
// Display header elements
//===================================================================
void vil3d_meta_image_header::print_header(vcl_ostream &os) const
{
  os << "\n============= Meta Image Header Summary Begin =============\n";
  os << "vil3d_meta_image_header - byte order is msb: " << ((byte_order_msb_) ? "true" : "false") << "\n";
  os << "vil3d_meta_image_header - offset " << offset_i_ << ", " << offset_j_ << ", " << offset_k_ << "\n";
  os << "vil3d_meta_image_header - voxel size: " << vox_size_i_ << ", " << vox_size_j_ << ", " << vox_size_k_ << "\n";
  os << "vil3d_meta_image_header - dimension size: " << dim_size_i_ << ", " << dim_size_j_ << ", " << dim_size_k_ << "\n";
  os << "vil3d_meta_image_header - nplanes: " << nplanes_ << "\n";
  os << "vil3d_meta_image_header - element type: " << elem_type_ << "\n";
  os << "vil3d_meta_image_header - image file: " << im_file_ << "\n";
  os << "============= Meta Image Header Summary End =============\n\n";
}

//===================================================================
// Check if swapping is needed
//===================================================================
void vil3d_meta_image_header::check_need_swap(void)
{
  union short_char
  {
    short short_val;
    char char_val[2];
  } test_swap;

  test_swap.short_val = 1;

  if (test_swap.char_val[0]==1 && byte_order_msb_) // System little endian, file big endian
    need_swap_ = true;
  else if (test_swap.char_val[1]==1 && (!byte_order_msb_)) // System big endian, file little endian
    need_swap_ = true;
  else
    need_swap_ = false;
}

//===================================================================
// Return whether swap is needed
//===================================================================
bool vil3d_meta_image_header::need_swap(void) const
{
  return need_swap_;
}

//===================================================================
// Check the next line in the header
//===================================================================
bool vil3d_meta_image_header::check_next_header_line(const vcl_string &nxt_line)
{
  // Look for each element we're interested in
  vcl_string val = get_header_value(nxt_line);
  if (val=="")
    return false;

  if (nxt_line.find("ObjectType")!= vcl_string::npos)
  {
    if (val != "Image")
    {
      vcl_cerr << "Loader only handles Image Types." << vcl_endl;
      return false;
    }
  }
  else if (nxt_line.find("NDims")!= vcl_string::npos)
  {
    unsigned int nd = atoi(val.c_str());
    if (nd != 3)
    {
      vcl_cerr << "Loader only handles 3D Images." << vcl_endl;
      return false;
    }
  }
  else if (nxt_line.find("BinaryDataByteOrderMSB")!= vcl_string::npos)
  {
    byte_order_msb_ = (val=="True") ? true : false;
    header_valid_ = true;
    check_need_swap();
  }
  else if (nxt_line.find("CompressedData")!= vcl_string::npos)
  {
    if (val=="True")
    {
      vcl_cerr << "Loader does not handle compressed data" << vcl_endl;
      return false;
    }
  }
  else if (nxt_line.find("TransformMatrix")!= vcl_string::npos)
  {
    if (val != "1 0 0 0 1 0 0 0 1")
    {
      vcl_cout << "Loader only handles identity in TransformMatrix." << vcl_endl;
      vcl_cout << "Transformation ignored." << vcl_endl;
    }
  }
  else if (nxt_line.find("Offset")!= vcl_string::npos) // If there is another field at some point with Offset in the name check them before this one!
  {
    return set_header_offset(val);
  }
  else if (nxt_line.find("DimSize")!= vcl_string::npos)
  {
    return set_header_dim_size(val);
  }
  else if (nxt_line.find("ElementSpacing")!= vcl_string::npos)
  {
    return set_header_voxel_size(val);
  }
  else if (nxt_line.find("ElementSize")!= vcl_string::npos)
  {
    return set_header_voxel_size(val);
  }
  else if (nxt_line.find("ElementType")!= vcl_string::npos)
  {
    elem_type_ = val;
    if (elem_type_ == "MET_SHORT")
      pformat_ = VIL_PIXEL_FORMAT_INT_16;
    else if (elem_type_ == "MET_UCHAR")
      pformat_ = VIL_PIXEL_FORMAT_BYTE;
    else
    {
      vcl_cerr << "Unsupported element type specified" << vcl_endl;
      return false;
    }
    header_valid_ = true;
  }
  else if (nxt_line.find("ElementDataFile")!= vcl_string::npos)
  {
    im_file_ = val;
    header_valid_ = true;
  }
  return true;
}

//===================================================================
// Get the value associated with a header element
//===================================================================
vcl_string vil3d_meta_image_header::get_header_value(const vcl_string &nxt_line)
{
  vcl_string::size_type pos, epos;
  pos = nxt_line.find("=");
  if (pos == vcl_string::npos)
  {
    return "";
  }

  pos = nxt_line.find_first_not_of(" ", pos+1);
  epos = nxt_line.find_last_not_of(" ");
  return nxt_line.substr(pos, (epos-pos)+1);
}

//===================================================================
// Set the header offset
//===================================================================
bool vil3d_meta_image_header::set_header_offset(const vcl_string &offs)
{
  vcl_string::size_type pos,epos;
  epos=offs.find_first_of(" ");
  if (epos==vcl_string::npos)
  {
    vcl_cerr << "Offset does not contain three values." << vcl_endl;
    return false;
  }

  offset_i_=atof(offs.substr(0,epos).c_str());
  pos=offs.find_first_not_of(" ",epos);
  epos=offs.find_first_of(" ",pos);
  if (pos==vcl_string::npos || epos==vcl_string::npos)
  {
    vcl_cerr << "Offset does not contain three values." << vcl_endl;
    return false;
  }

  offset_j_=atof(offs.substr(pos,epos).c_str());
  pos=offs.find_first_not_of(" ",epos);
  if (pos==vcl_string::npos)
  {
    vcl_cerr << "Offset does not contain three values." << vcl_endl;
    return false;
  }
  offset_k_=atof(offs.substr(pos).c_str());
  epos = offs.find_first_of(" ",pos);
  pos=offs.find_first_not_of(" ",epos);
  if (pos != vcl_string::npos)
  {
     vcl_cerr << "Offset contains more than three values." << vcl_endl;
     return false;
  }
  header_valid_ = true;
  return true;
}

//===================================================================
// Set the dimensions from the header
//===================================================================
bool vil3d_meta_image_header::set_header_dim_size(const vcl_string &dims)
{
  vcl_string::size_type pos,epos;
  epos=dims.find_first_of(" ");
  if (epos==vcl_string::npos)
  {
    vcl_cerr << "Dim Size does not contain three values." << vcl_endl;
    return false;
  }
  dim_size_i_=atoi(dims.substr(0,epos).c_str());
  pos=dims.find_first_not_of(" ",epos);
  epos=dims.find_first_of(" ",pos);
  if (pos==vcl_string::npos || epos==vcl_string::npos)
  {
    vcl_cerr << "Dim Size does not contain three values." << vcl_endl;
    return false;
  }
  dim_size_j_=atoi(dims.substr(pos,epos).c_str());
  pos=dims.find_first_not_of(" ",epos);
  if (pos==vcl_string::npos)
  {
    vcl_cerr << "Dim Size does not contain three values." << vcl_endl;
    return false;
  }
  dim_size_k_=atoi(dims.substr(pos).c_str());
  epos = dims.find_first_of(" ",pos);
  pos=dims.find_first_not_of(" ",epos);
  if (pos != vcl_string::npos)
  {
     vcl_cerr << "Dim Size contains more than three values." << vcl_endl;
     return false;
  }
  // For now only deal with 1 plane
  nplanes_=1;
  header_valid_=true;
  return true;
}

//===================================================================
// Set the header voxel size
//===================================================================
bool vil3d_meta_image_header::set_header_voxel_size(const vcl_string &vsize)
{
  vcl_string::size_type pos,epos;
  epos=vsize.find_first_of(" ");
  if (epos==vcl_string::npos)
  {
    vcl_cerr << "Element Spacing/Size does not contain three values." << vcl_endl;
    return false;
  }
  vox_size_i_=atof(vsize.substr(0,epos).c_str());
  pos=vsize.find_first_not_of(" ",epos);
  epos=vsize.find_first_of(" ",pos);
  if (pos==vcl_string::npos || epos==vcl_string::npos)
  {
    vcl_cerr << "Element Spacing/Size does not contain three values." << vcl_endl;
    return false;
  }
  vox_size_j_=atof(vsize.substr(pos,epos).c_str());
  pos=vsize.find_first_not_of(" ",epos);
  if (pos==vcl_string::npos)
  {
    vcl_cerr << "Element Spacing/Size does not contain three values." << vcl_endl;
    return false;
  }
  vox_size_k_=atof(vsize.substr(pos).c_str());
  epos = vsize.find_first_of(" ",pos);
  pos=vsize.find_first_not_of(" ",epos);
  if (pos != vcl_string::npos)
  {
     vcl_cerr << "Element Spacing/Size contains more than three values." << vcl_endl;
     return false;
  }
  header_valid_ = true;
  return true;
}

//===================================================================
// Display the header
//===================================================================
vcl_ostream& operator<<(vcl_ostream& os, const vil3d_meta_image_header& header)
{
  header.print_header(os);
  return os;
}

/*
 * Format stuff
 */

//===================================================================
// Construct image format
//===================================================================
vil3d_meta_image_format::vil3d_meta_image_format()
{
  // Nothing to be done on construction
}

//===================================================================
// Destruct image format
//===================================================================
vil3d_meta_image_format::~vil3d_meta_image_format()
{
  // Nothing to be done on destruction
}

//===================================================================
// Create an input image
//===================================================================
vil3d_image_resource_sptr vil3d_meta_image_format::make_input_image(const char *fname) const
{
  vil3d_meta_image_header header;
  vcl_string filename(fname);
  
  if (!header.read_header(fname)) return 0;
  vcl_cout<<"vil3d_meta_image_format::make_input_image() Header: "<<header<<vcl_endl;

  return new vil3d_meta_image(header,filename);
}

//===================================================================
// Create an output image
//===================================================================
vil3d_image_resource_sptr vil3d_meta_image_format::make_output_image(const char *filename,
                                                                     unsigned int ni, 
                                                                     unsigned int nj, 
                                                                     unsigned int nk, 
                                                                     unsigned int nplanes, 
                                                                     vil_pixel_format format) const
{
  if (format != VIL_PIXEL_FORMAT_BYTE   &&
      format != VIL_PIXEL_FORMAT_INT_16 )
  {
    vcl_cerr << "vil3d_meta_image_format::make_output_image() WARNING\n"
             << "  Unable to deal with pixel format : " << format << vcl_endl;
    return 0;
  }

  vil3d_meta_image_header header;
  header.clear();
  header.set_dim_size(ni,nj,nk,nplanes);
  header.set_pixel_format(format);

  switch (format)
  {
  case VIL_PIXEL_FORMAT_BYTE: header.set_element_type("MET_UCHAR");
                              break;
  case VIL_PIXEL_FORMAT_INT_16: header.set_element_type("MET_SHORT");
                              break;
  default:
      vcl_cerr << "vil3d_meta_image_format::make_output_image() WARNING\n"
             << "  Unable to deal with pixel format : " << format << vcl_endl;
      return 0;
  }

  vcl_string str_fname(filename);
  vcl_string base_filename;
  int n=str_fname.size();
  if (n>=4 && (str_fname.substr(n-4,4)==".mhd" || str_fname.substr(n-4,4)==".raw"))
    base_filename = str_fname.substr(0,n-4);
  else
    base_filename = str_fname;
  vcl_string im_file = vul_file::strip_directory(base_filename);
  header.set_image_fname(im_file + ".raw");
  if (!header.write_header(base_filename+".mhd")) return 0;
  return new vil3d_meta_image(header,base_filename);
}

/*
 * Image stuff
 */

//===================================================================
// Contruct an image
//===================================================================
vil3d_meta_image::vil3d_meta_image(const vil3d_meta_image_header &header,
                                   const vcl_string &fname) :
header_(header), 
fpath_(fname)
{
  // No code necessary
}

//===================================================================
// Destruct
//===================================================================
vil3d_meta_image::~vil3d_meta_image(void)
{
  // No code necessary
}

//===================================================================
// Get the image dimension details
//===================================================================
unsigned int vil3d_meta_image::nplanes(void) const
{
  return header_.nplanes();
}

unsigned int vil3d_meta_image::ni(void) const
{
  return header_.ni();
}

unsigned int vil3d_meta_image::nj(void) const
{
  return header_.nj();
}

unsigned int vil3d_meta_image::nk(void) const
{
  return header_.nk();
}

//===================================================================
// Get the current header
//===================================================================
const vil3d_meta_image_header &vil3d_meta_image::header(void) const
{
  return header_;
}

//===================================================================
// Get the pixel format
//===================================================================
vil_pixel_format vil3d_meta_image::pixel_format(void) const
{
  return header_.pixel_format();
}

//===================================================================
// Set the voxel size
//===================================================================
bool vil3d_meta_image::set_voxel_size(float vi, float vj, float vk)
{
  header_.set_vox_size(vi,vj,vk);
  if (!header_.write_header(fpath_+".mhd")) return false;
  return true;
}

//===================================================================
// Set the offet
//===================================================================
void vil3d_meta_image::set_offset(const double i, const double j, const double k,
                                  const double vx_i, const double vx_j, const double vx_k)
{
  double os_i, os_j, os_k;
  os_i = (-(i*vx_i));
  os_j = (-(j*vx_j));
  os_k = (-(k*vx_k));
  header_.set_offset(os_i,os_j,os_k);
  header_.set_vox_size(vx_i,vx_j,vx_k);
  header_.write_header(fpath_+".mhd");
}

//===================================================================
// Create a read/write view of a copy of this data
//===================================================================
vil3d_image_view_base_sptr vil3d_meta_image::get_copy_view(unsigned int i0, unsigned int ni,
                                                           unsigned int j0, unsigned int nj,
                                                           unsigned int k0, unsigned int nk) const
{
  // Can only cope with loading whole image at present.
  if (i0!=0 || int(ni)!=header_.ni() ||
      j0!=0 || int(nj)!=header_.nj() ||
      k0!=0 || int(nk)!=header_.nk()   ) return 0;

  vcl_string image_data_path=header_.image_fname();
  vil_smart_ptr<vil_stream> is = new vil_stream_fstream(image_data_path.c_str(),"r");
  if (!is->ok()) return 0;

// NOTE: See GIPL loader for more general data reading
#define read_data_of_type(type) \
  vil3d_image_view< type > im = \
         vil3d_new_image_view_plane_k_j_i(ni, nj, nk, nplanes(), type()); \
  is->read(&im(0,0,0,0), ni * nj * nk * nplanes() * sizeof(type));

  switch (pixel_format())
  {
   case VIL_PIXEL_FORMAT_BYTE:
   {
    read_data_of_type(vxl_byte);
    return new vil3d_image_view<vxl_byte>(im);
   }
   case VIL_PIXEL_FORMAT_INT_16:
   {
    read_data_of_type(vxl_int_16);
    if (header_.need_swap())
      vil3d_meta_image_swap16((char *)(im.origin_ptr()), ni*nj*nk);
    return new vil3d_image_view<vxl_int_16>(im);
   }
   default:
    vcl_cout<<"ERROR: vil3d_meta_image_format::get_copy_view()\n"
            <<"Can't deal with pixel type " << pixel_format() << vcl_endl;
    return 0;
  }
}

//===================================================================
// Put view
//===================================================================
bool vil3d_meta_image::put_view(const vil3d_image_view_base &im, 
                                unsigned int i0, 
                                unsigned int j0, 
                                unsigned int k0)
{
  if (!view_fits(im, i0, j0, k0))
  {
    vcl_cerr << "ERROR: " << __FILE__ << ":\n view does not fit\n";
    return false;
  }
  if (im.ni()!=ni() || im.nj()!=nj() || im.nk()!=nk())
  {
    vcl_cerr<<"Can only write whole image at once.\n";
    return false;
  }

  vcl_string image_data_path=fpath_+".raw";
  vil_smart_ptr<vil_stream> os = new vil_stream_fstream(image_data_path.c_str(),"w");
  if (!os->ok()) return 0;

  switch (pixel_format())
  {
   case VIL_PIXEL_FORMAT_BYTE:
   {
    vil3d_image_view<vxl_byte> view_copy(ni(),nj(),nk(),nplanes());
    vil3d_copy_reformat(static_cast<const vil3d_image_view<vxl_byte>&>(im),view_copy);
    os->write(view_copy.origin_ptr(),ni()*nj()*nk()*nplanes());
    // Should check that write was successful
    return true;
   }
   case VIL_PIXEL_FORMAT_INT_16:
   {
     header_.check_need_swap();
    vil3d_image_view<vxl_int_16> view_copy(ni(),nj(),nk(),nplanes());
    vil3d_copy_reformat(static_cast<const vil3d_image_view<vxl_int_16>&>(im),view_copy);
    if (header_.need_swap())
      vil3d_meta_image_swap16((char *)(view_copy.origin_ptr()), ni()*nj()*nk()*nplanes());
    os->write(view_copy.origin_ptr(),ni()*nj()*nk()*nplanes()*sizeof(vxl_int_16));
    // Should check that write was successful
    return true;
   }
   default:
    vcl_cout<<"ERROR: vil3d_analyze_format::put_view()\n"
            <<"Can't deal with pixel type " << pixel_format() << vcl_endl;
  }

  return false;
}

//===================================================================
// Get an image property
//===================================================================
bool vil3d_meta_image::get_property(const char *label, void *property_value) const
{
  if (vcl_strcmp(vil3d_property_voxel_size, label)==0)
  {
    float* array = static_cast<float*>(property_value);
    // meta image stores data in mm
    array[0] = static_cast<float>(header_.vox_size_i() / 1000.0);
    array[1] = static_cast<float>(header_.vox_size_j() / 1000.0);
    array[2] = static_cast<float>(header_.vox_size_k() / 1000.0);
    return true;
  }

  if (vcl_strcmp(vil3d_property_origin_offset, label)==0)
  {
    float* array = static_cast<float*>(property_value);
    array[0] = static_cast<float>((-header_.offset_i())/header_.vox_size_i());
    array[1] = static_cast<float>((-header_.offset_j())/header_.vox_size_j());
    array[2] = static_cast<float>((-header_.offset_k())/header_.vox_size_k());
    return true;
  }

  return false;
}
