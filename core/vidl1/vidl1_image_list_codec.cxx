// This is core/vidl1/vidl1_image_list_codec.cxx
#include "vidl1_image_list_codec.h"
//:
// \file

#include <vcl_cassert.h>
#include <vcl_iostream.h>

#include <vul/vul_sprintf.h>

#include <vidl1/vidl1_movie.h>
#include <vil/vil_image_view_base.h>
#include <vil/vil_load.h>
#include <vil/vil_save.h>

//=========================================================================
//  Methods for vidl1_image_list_codec.
//_________________________________________________________________________

vcl_string vidl1_image_list_codec::default_initialization_image_type_ = "tiff";

//------------------------------------------------------------------------
// CONSTRUCTOR(S) AND DESTRUCTOR


//: Constructor, from a list of images
vidl1_image_list_codec::vidl1_image_list_codec(vcl_list<vil_image_resource_sptr>& images)
{
  // Set the image type to the default value
  default_image_type_ = default_initialization_image_type_;

  for (vcl_list<vil_image_resource_sptr>::iterator i=images.begin(); i!= images.end(); ++i)
    images_.push_back(*i);

  if (!init())
    vcl_cerr << "Failed to initialize the ImageList Codec.\n";
}

//: Constructor, from a vector of images
vidl1_image_list_codec::vidl1_image_list_codec(vcl_vector<vil_image_resource_sptr>& images)
{
  // Set the image type to the default value
  default_image_type_ = default_initialization_image_type_;

  for (vcl_vector<vil_image_resource_sptr>::iterator i=images.begin(); i!= images.end(); ++i)
    images_.push_back(*i);

  if (!init())
    vcl_cerr << "Failed to initialize the ImageList Codec.\n";
}

//: Basic constructor. Should not be called unless we initialize the codec by some ways.
vidl1_image_list_codec::vidl1_image_list_codec()
{
  // Set the image type to the default value
  default_image_type_ = default_initialization_image_type_;

  // Nothing to do, here
  // Caution, a call to this constructor
  // creates an instance of this class in bad shape
}

//: Initialize
bool vidl1_image_list_codec::init()
{
  if (images_.empty())
    return false;

//   unfinished !!!!! TODO

     set_number_frames(images_.size());
     vil_image_resource_sptr first = images_[0];

// Come from TargetJr, don't know the vxl equivalent
//   set_format(first->get_format());
//   set_image_class(first->get_image_class());
     set_bits_pixel(vil_pixel_format_sizeof_components(first->pixel_format()) *
                    vil_pixel_format_num_components(first->pixel_format()) * first->nplanes() * 8);
     set_width(first->ni());
     set_height(first->nj());

  return true;
}


//: Return the resource to the image
vil_image_resource_sptr
vidl1_image_list_codec::get_resource(int position) const
{
  return images_[position];
}

//: Get a section of pixels in function of the frame number, position and size.
vil_image_view_base_sptr vidl1_image_list_codec::get_view(int position, int x0, int w, int y0, int h) const
{
  return images_[position]->get_view(x0, w, y0, h);
}

//: Put a section of pixels in function of the frame number, position and size.
bool vidl1_image_list_codec::put_view(int /*position*/, const vil_image_view_base & /*im*/, int /*x0*/, int /*y0*/)
{
  vcl_cerr << "vidl1_image_list_codec::put_section not implemented\n";
  return false;
}

//: Load from a file name.
vidl1_codec_sptr vidl1_image_list_codec::load(vcl_string const& fname, char mode)
{
  // will try and load as many images as possible starting with
  //   index 0 and stopping when we run out of images
  assert(mode == 'r');

  for (int i=0; true; ++i)
  {
    vcl_string name = vul_sprintf(fname.c_str(), i);
    vil_image_resource_sptr img= vil_load_image_resource(name.c_str());

    if (img)
      images_.push_back(img);
    else
      break;
  }

  if (!init())
  {
    vcl_cerr << "Failed to initialize the ImageList Codec.\n";
    return NULL;
  }

  return this;
}

//: Load a 'movie' from a list of filenames, return a codec.
vidl1_codec_sptr vidl1_image_list_codec::load(const vcl_list<vcl_string> &fnames, char mode)
{
#if 0 // commented out (why?)
  // Makes sure image loaders are registered
  register_image_loaders();
#endif
  assert(mode == 'r');

  for (vcl_list<vcl_string>::const_iterator i = fnames.begin(); i!=fnames.end(); ++i)
  {
    vil_image_resource_sptr img =  vil_load_image_resource((*i).c_str());
    if (img)
    {
      images_.push_back(img);
    }
  }

  // Initialize the codec
  if (!init())
  {
    vcl_cerr << "Failed to initialize the ImageList Codec.\n";
    return NULL;
  }

  // Every thing was all right,
  // return myself
  return this;
}

//: Load a 'movie' from a vector of filenames, return a codec.
vidl1_codec_sptr vidl1_image_list_codec::load(const vcl_vector<vcl_string> &fnames, char mode)
{
#if 0 // commented out (why?)
  // Make sure image loaders are registered
  register_image_loaders();
#endif
  assert(mode == 'r');

  for (vcl_vector<vcl_string>::const_iterator i = fnames.begin(); i!=fnames.end(); ++i)
  {
    vil_image_resource_sptr img =  vil_load_image_resource((*i).c_str());

    if (img)
      images_.push_back(img);
  }

  // Initialize the codec
  if (!init())
  {
    vcl_cerr << "Failed to initialize the ImageList Codec.\n";
    return NULL;
  }

  // Every thing was all right,
  // return myself
  return this;
}

//: Supposed to check the validity of this codec for a special filename.
// Not so well implemented for this codec.
// This could check if the filename is a valid image type
// by probing all the image types.
bool vidl1_image_list_codec::probe(vcl_string const& /*fname*/)
{
  return false;
}

//: Save the given video as a set of images of the default set type.
bool vidl1_image_list_codec::save(vidl1_movie* movie, vcl_string const& fname)
{
  if (default_image_type_ == "")
  {
    vcl_cerr << "No default image type defined to save the video as a list of images.\n";
    return false;
  }

  return save(movie, fname, default_image_type_);
}

//: Save the given video as a set of images of the type given.
bool vidl1_image_list_codec::save(vidl1_movie* movie,
                                  vcl_string const& fname,
                                  vcl_string const& type)
{
  // The value to be returned
  bool ret = true;

  // Create the extension for filenames
  vcl_string extension = type.substr(0, type.size()-5); // To get rid of "Image" string

  for (vidl1_movie::frame_iterator pframe = movie->begin();
       pframe <= movie->last();
       ++pframe)
  {
    // Get the image from the frame
    vil_image_view_base_sptr image = pframe->get_view();

    // Create a name for the current image to be saved
    vcl_string currentname = vul_sprintf("%s%05d.%s", fname.c_str(),
                                         pframe.current_frame_number(),
                                         extension.c_str());

    bool saved_image = vil_save(*image, currentname.c_str(), type.c_str());

    if (!saved_image)
      ret = false;
  }

  return ret;
}
