#ifndef vidl_mpegcodec_h
#define vidl_mpegcodec_h

//:
// \file
//
// \author l.e.galup
//  this class sets up and executes the vidl_mpegcodec_helper, which
//  is a wrapped port of mpeg2dec. it accepts the vo open function ptr
//  in the constructor. this was done this way to make it more extensible,
//  in case other video outputs would be added later.
//
//  To use:
//  1) load the file
//  2) set the attributes. format, demux, ...
//  3) then use get_section. the get section call will actualize
//     initialize the helper class. its a one shot deal. once
//     initialized, you are stuck with that roi. i could change
//     this later, if it proves necessary.
//
//  this class works on both mpeg1 and mpeg2.
// \date July 2002
// \verbatim
// Modifications
// \endverbatim

#include "vidl_mpegcodec_helper.h"
#include <vidl/vidl_codec.h>
#include <vidl/vidl_frame_sptr.h>
#include <vidl/vidl_codec_sptr.h>

class vil_image;

//: Allows user to load MPEG files as vxl video.
class vidl_mpegcodec : public vidl_codec
{
public:
  vidl_mpegcodec();
  ~vidl_mpegcodec();

  //Casting methods overridden here...
  vidl_mpegcodec* castto_vidl_mpegcodec(){return this;}

  //-----------------------------------------------------
  //pure virtual methods
  //------------------------------------------

  bool   get_section(int frame_position,
                     void* ib,
                     int x0,
                     int y0,
                     int xs,
                     int ys) const;
  int    put_section(int frame_position,
                     void* ib,
                     int x0,
                     int y0,
                     int xs,
                     int ys){return 0;}

  virtual bool probe(const char* fname);
  virtual vidl_codec_sptr load(const char* fname, char mode = 'r' );
  bool save(vidl_movie* movie, const char* fname){return true;};
  virtual const char* type() {return "MPEG";}

  //-----------------------------------------------
  //initialization methods.
  //these must be set by the user before init is called,
  //but after load. init must be called, before get_section is
  //called.
  void set_grey_scale(bool grey);
  void set_demux_video();
  void set_pid(vcl_string pid);
  bool init();

  //-------------------------------------------------

private:

  //--- these are set by load/save
  vidl_mpegcodec_helper * decoder_;
  frame_buffer * buffers_;
  bool inited;
};

#endif // vidl_mpegcodec_h
