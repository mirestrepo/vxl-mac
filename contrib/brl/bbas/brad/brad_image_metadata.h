//-----------------------------------------------------------------------------
//:
// \file
// \brief Collection of image metadata values needed for radiometric calculations
// \author D. E. Crispell
// \date January 27, 2012
//
//----------------------------------------------------------------------------
#ifndef brad_image_metadata_h_
#define brad_image_metadata_h_

#include <vbl/vbl_ref_count.h>
#include <vbl/vbl_smart_ptr.h>

#include <vcl_string.h>
#include <vcl_iostream.h>

struct image_time {
  int year, month, day, hour, min;
};

class brad_image_metadata : public vbl_ref_count
{
 public:
  //: Constructor extracts information from NITF header and vendor-specific metadata file
  brad_image_metadata(vcl_string const& nitf_filename, vcl_string const& meta_folder = "");
  //: Default constructor
  brad_image_metadata() : gain_(1.0), offset_(0.0) {}

  // position of sun relative to imaged location
  double sun_elevation_; // degrees above horizon
  double sun_azimuth_;   // degrees east of north

  // position of image sensor relative to imaged location
  double view_elevation_; // degrees above horizon
  double view_azimuth_;   // degrees east of north

  // gain and offset required to convert digital number (DN) to band-averaged radiance value
  double gain_;           // units W m^-2 sr^-1 um^-1 DN^-1
  double offset_;         // units W m^-2 sr^-1 um^-1

  // band-averaged sun irradiance (includes term accounting for Earth-Sun distance)
  double sun_irradiance_; // units W m^-2

  image_time t_;
  unsigned number_of_bits_;
  //: parse header in nitf image, assumes that metadata files are in the same folder with the image
  //  If meta_folder is not empty, they are searched in that folder as well
  bool parse(vcl_string const& nitf_filename, vcl_string const& meta_folder = "");
 protected:
  //: Parse Quickbird IMD file
  bool parse_from_imd(vcl_string const& filename);
  //: Parse GeoEye PVL file
  bool parse_from_pvl(vcl_string const& filename);
};

typedef vbl_smart_ptr<brad_image_metadata> brad_image_metadata_sptr;

//: Write brad_image_metadata to stream
vcl_ostream&  operator<<(vcl_ostream& s, brad_image_metadata const& md);

//: Read brad_image_metadata from stream
vcl_istream&  operator>>(vcl_istream& s, brad_image_metadata& md);

#endif
