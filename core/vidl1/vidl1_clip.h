// This is core/vidl1/vidl1_clip.h
#ifndef vidl1_clip_h
#define vidl1_clip_h
//:
// \file
// \author Nicolas Dano, september 1999
//
// \verbatim
//  Modifications
//   June 2000 Julien ESTEVE          Ported from TargetJr
//   10/4/2001 Ian Scott (Manchester) Converted perceps header to doxygen
//   10/7/2003 Matt Leotta (Brown)    Converted vil1 to vil
//   10/9/2004 Peter Vanroose  Inlined all 1-line methods in class decl
// \endverbatim

#include <vbl/vbl_ref_count.h>
#include <vidl1/vidl1_clip_sptr.h>
#include <vidl1/vidl1_frame_sptr.h>
#include <vidl1/vidl1_codec_sptr.h>
#include <vil/vil_image_resource.h>

#include <vcl_vector.h>
#include <vcl_list.h>

//: Sequence of frames, element of a movie
//  A clip is a set of frames, it is part of a Movie.
//  It has the notion of starting frame, ending frame
//  and increment, so that when we iterate through the
//  sequence, it will iterate only through the frames
//  of interest.
//
//  See also vidl1_frame and vidl1_movie.

class vidl1_clip : public vbl_ref_count
{
  // PUBLIC INTERFACE
 public:

  // Constructors
  vidl1_clip(vidl1_codec_sptr codec,
             int start = 0,
             int end = 0,
             int increment = 1);

  vidl1_clip(vcl_list<vil_image_resource_sptr> &images,
             int start = 0,
             int end = 0,
             int increment = 1);

  vidl1_clip(vcl_vector<vil_image_resource_sptr> &images,
             int start = 0,
             int end = 0,
             int increment = 1);

  // Copy constructor.
  vidl1_clip(vidl1_clip const& x)
    : vbl_ref_count(), frames_(x.frames_),
      startframe_(x.startframe_), endframe_(x.endframe_),
      increment_(x.increment_), frame_rate_(x.frame_rate_), coder_(x.coder_) {}

  // Destructor
  ~vidl1_clip() {}

  // Data Access
  vidl1_frame_sptr get_frame(int n);

  //: Return the number of frames
  int length() const { return (endframe_-startframe_)/increment_ + 1; }

  //: Return the horizontal size of the frames in the clip
  int width() const;
  //: Return the vertical size of the frames in the clip
  int height() const;

  //: Return the codec.
  vidl1_codec_sptr get_codec() { return coder_; }

 protected:
  void init(int start, int end, int increment);

  // Data Members
  vcl_vector<vidl1_frame_sptr> frames_; //!< Where are the frames stored
  int startframe_;                     //!< The clip begins at startframe_
  int endframe_;                       //!< The clip ends at startframe_
  int increment_;                      //!< The clip uses 1 frame every "increment_"
  double frame_rate_;                  //!< 1 frame every ?? secs
  vidl1_codec_sptr coder_;              //!< video codec used for storage
};

#endif // vidl1_clip_h
