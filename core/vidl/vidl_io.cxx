//:
// \file

#include "vidl_io.h"
#include "vidl_codec_callbacks.h"
#include <vidl/vidl_movie.h>
#include <vidl/vidl_clip.h>
#include <vidl/vidl_image_list_codec.h>
#ifdef HAS_MPEG
# include <vidl/vidl_mpegcodec.h>
#endif

#ifdef _MSC_VER // Microsoft compiler
# include <vidl/vidl_avicodec.h>
#endif // _MSC_VER

#if 0 // TODO
#include <Basics/stat.h>
#include <Basics/dir.h>
#include <Basics/stringarray.h>
#include <Basics/IUPath.h>
#include <Basics/IUFilename.h>
#endif

#include <vul/vul_file.h>
#include <vul/vul_sequence_filename_map.h>

#include <vcl_iostream.h>
#include <vcl_list.h>
#include <vcl_vector.h>

vcl_list<vidl_codec_sptr> vidl_io::supported_types_;

#if 0
// Constructor does nothing.
vidl_io::vidl_io()
{
}

// Destructor does nothing.
vidl_io::~vidl_io()
{
}
#endif

//-----------------------------------------------------------------------------

//: Load a movie, takes a file name and return the created movie.
// A starting frame, ending frame and increment number are optionals
vidl_movie_sptr vidl_io::load_movie(
        const char* fname,
        int start,
        int end,
        int increment,
        char mode
        )
{
  vidl_clip_sptr clip = load_clip(fname, start, end, increment, mode);

  // Stop here if the clip was not created
  if (!clip)
    return 0;

  vidl_movie_sptr movie = new vidl_movie(clip);

  return movie;
}

//: Loads and creates movie from a list of image file names.
// A starting frame, ending frame and increment number are optionals
vidl_movie_sptr  vidl_io::load_movie(
        const vcl_list<vcl_string> &fnames,
        int start,
        int end,
        int increment,
        char mode
        )
{
  vidl_clip_sptr clip = load_clip(fnames, start, end, increment, mode);

  // Stop here if the clip was not created
  if (!clip)
    return 0;

  vidl_movie_sptr movie = new vidl_movie(clip);

  return movie;
}

//: Loads and creates movie from a vector of image file names.
// A starting frame, ending frame and increment number are optionals
vidl_movie_sptr  vidl_io::load_movie(
        const vcl_vector<vcl_string> &fnames,
        int start,
        int end,
        int increment,
        char mode
        )
{
  vidl_clip_sptr clip = load_clip(fnames, start, end, increment, mode);

  // Stop here if the clip was not created
  if (!clip)
    return 0;

  vidl_movie_sptr movie = new vidl_movie(clip);

  return movie;
}

static bool looks_like_a_file_list(char const* fname);
static vidl_clip_sptr load_from_file_list(char const* fname);

//: Load a clip, takes a file name and return the created clip.
// A starting frame, ending frame and increment number are optionals
vidl_clip_sptr  vidl_io::load_clip(
        const char* fname,
        int start,
        int end,
        int increment,
        char mode
        )
{
  // make sure that fname is sane
  if (!fname ||
      (vcl_strlen(fname) == 0))
    {
      return 0;
    }

#if 0
  // Check if the filename is a directory
  IUE_stat fs(fname);  // check that file size is non zero
    if (!fs) {
      return 0;
    }

  cl_stat_t fs;
  f (!vcl_stat(fname, &fs))
     return 0;
#endif

  // test if fname is a directory
  if (vul_file::is_directory(fname))
      {
          // fname is a directory.
          // So, we will process all the files in this directory
          // as images making the video.
          return 0;
#if 0 // avoid warnings about unreachable code - fsm@robots.ox.ac.uk
          Dir dir(fname);
          if(!dir.IsOpen())
             return 0;
#endif
      }
  else if (looks_like_a_file_list(fname))
    return load_from_file_list(fname);

  // The file is not a directory,
  // Let us try all the known video formats,
  // hoping to find the good one
  vcl_list<vidl_codec_sptr>::iterator i = supported_types_.begin();
  if (i == supported_types_.end())
    vcl_cerr << "vidl_io: warning: no codecs installed\n";

  while (i != supported_types_.end())
  {
    if ((*i)->probe(fname))
      {
        vidl_codec_sptr codec = (*i)->load(fname, mode);
        if (!codec)
          return 0;

#ifdef HAS_MPEG
	//this calls the dialog box necessary for initialization
	//of the mpeg codec.
	vidl_mpegcodec * vmp = (*i)->castto_vidl_mpegcodec();
	if (vmp)
	  load_mpegcodec_callback(vmp);
#endif

        vidl_clip_sptr clip = new vidl_clip(codec, start, end, increment);
	vcl_cout << "vidl_io::load_move. just got a new clip." << vcl_endl;
        return clip;
      }

    ++i;
  }

  // We did not find a codec corresponding
  // to this file name
  // Return error
  return 0;
}

//: Loads and creates clip from a list of image file names.
// A starting frame, ending frame and increment number are optionals
vidl_clip_sptr vidl_io::load_clip(
        const vcl_list<vcl_string> &fnames,
        int start,
        int end,
        int increment,
        char mode
        )
{
  // Check if the input is correct
  if (fnames.empty())
    return 0;

  // If we have only one filename, we run the special function written for that
  if (fnames.size()==1)
    return load_clip((*fnames.begin()).c_str(), start, end, increment, mode);

  // We have severall files, so we suppose that the video is a set of images.
  return load_images(fnames, start, end, increment, mode);
}

//: Loads and creates clip from a list of image file names.
// A starting frame, ending frame and increment number are optionals
vidl_clip_sptr  vidl_io::load_clip(
        const vcl_vector<vcl_string> &fnames,
        int start,
        int end,
        int increment,
        char mode
        )
{
  // Check if the input is correct
  if (fnames.empty())
    return 0;

  // If we have only one filename, we run the special function written for that
  if (fnames.size()==1)
    return load_clip((*fnames.begin()).c_str(), start, end, increment, mode);

  // We have severall files, so we suppose that the video is a set of images.
  return load_images(fnames, start, end, increment, mode);
}

//: load a list of images as a clip.
// This function should not be called unless
// you are sure you are dealing with images
vidl_clip_sptr  vidl_io::load_images(
        const vcl_list<vcl_string> &fnames,
        int start,
        int end,
        int increment,
        char mode
        )
{
  // This should be a video represented by a set of images
  vidl_image_list_codec_sptr ImCodec = new vidl_image_list_codec();

  vidl_codec_sptr codec = ImCodec->load(fnames, mode);
  if (!codec)
    return 0;
  vidl_clip_sptr clip = new vidl_clip(codec, start, end, increment);
  return clip;
}

//: load a vector of images as a clip.
// This function should not be called unless
// you are sure you are dealing with images
vidl_clip_sptr  vidl_io::load_images(
        const vcl_vector<vcl_string> &fnames,
        int start,
        int end,
        int increment,
        char mode
        )
{
  // This should be a video represented by a set of images
  vidl_image_list_codec_sptr ImCodec = new vidl_image_list_codec();

  vidl_codec_sptr codec = ImCodec->load(fnames, mode);
  if (!codec)
    return 0;
  vidl_clip_sptr clip = new vidl_clip(codec, start, end, increment);
  return clip;
}

//: Save a video into a file "fname" as type "type"
bool vidl_io::save(vidl_movie* movie, const char* fname, const char* type)
{
  // Go along the vcl_list of supported videoCODECs,
  // find the one of the type asked if it does exist.
  vcl_list<vidl_codec_sptr>::iterator i = supported_types_.begin();

  while ((i != supported_types_.end()) && (vcl_strcmp((*i)->type(), type)))
    {
      // const char* debug = (*i)->type();
      // vcl_cout << "debug : " << debug << " type : " << type << vcl_endl;
      ++i;
    }

  // Check if the type asked really exists in the context
  // If it does not, Try the vcl_list of images mode.
  if (i==supported_types_.end())
    return save_images(movie, fname, type);//return false;


  // Try to save it
  if ((*i)->save(movie, fname))
    return true;
  else
    return false;
}

// This function should be removed and integrated in save()
bool vidl_io::save_images(vidl_movie* movie, const char* fname,  const char* type)
{
  vidl_image_list_codec codec;

  // Try to save and return success or failure
  return codec.save(movie, fname, type);
}

//: Return the list of the supported video coder/decoder types
vcl_list<vcl_string> vidl_io::supported_types()
{
  // Create the vcl_list with type() for all the codecs
  vcl_list<vcl_string> ret;
  for (vcl_list<vidl_codec_sptr>::iterator i=supported_types_.begin(); i!=supported_types_.end(); ++i)
    ret.push_back((*i)->type());

  // Return the vcl_list of type supported codecs
  return ret;
}

//: register a new coder
void vidl_io::register_codec(vidl_codec* codec)
{
  supported_types_.push_back(codec);
}

//: Destroy codecs.
// Must call this before the MPEG library is deleted, i.e. on exit.
void vidl_io::close()
{
  vcl_list<vcl_string> ret;
  for (vcl_list<vidl_codec_sptr>::iterator i=supported_types_.begin(); i!=supported_types_.end(); ++i)
    (*i)->close();
  supported_types_.erase(supported_types_.begin(), supported_types_.end());
}

static bool looks_like_a_file_list(char const* fname)
{
  while (*fname) {
    if (*fname == '%' || *fname == '#')
      return true;
    ++fname;
  }
  return false;
}

static vidl_clip_sptr load_from_file_list(char const* fname)
{
  // Declare the vcl_list of image filenames
  vcl_list<vcl_string> filenames;

  vul_sequence_filename_map map(fname);

  for(int i = 0;i < map.get_nviews(); ++i) {
    vcl_string fullpath = map.image_name(i);
    // check to see if file is a directory.
    if (vul_file::is_directory(fullpath))
      continue;
    filenames.push_back(fullpath);
  }

  // Call load_images and return the result
  return vidl_io::load_images(filenames, 'r');
}
