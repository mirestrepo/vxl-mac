// This is core/vidl1/vidl1_ffmpeg_codec.h
#ifndef vidl1_ffmpeg_codec_h
#define vidl1_ffmpeg_codec_h
//:
// \file
// \author Matt Leotta
// \date Mar 22, 2005
//
// \verbatim
//  Modifications
//   Matt Leotta  Mar 22, 2005  Adapted from codec by Amitha Perera
// \endverbatim

extern "C" {
#ifdef  HAS_FFMPEG_SEVERAL
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#else
#include <ffmpeg/avcodec.h>
#include <ffmpeg/avformat.h>
#endif
}

#include <vidl1/vidl1_codec.h>
#include <vil/vil_image_view.h>

//: Allows user to load a video with ffmpeg
// See also vidl1_codec
class vidl1_ffmpeg_codec : public vidl1_codec
{
 public:
  //: Constructor
  vidl1_ffmpeg_codec();
  //: Destructor
 ~vidl1_ffmpeg_codec();

  //-----------------------------------------------------

  virtual vil_image_view_base_sptr  get_view(int position,
                                             int x0, int xs,
                                             int y0, int ys ) const;
  virtual bool put_view(int position,
                        const vil_image_view_base &im,
                        int x0, int y0 );

  //-----------------------------------------------------
  virtual bool probe(vcl_string const& fname);
  virtual vidl1_codec_sptr load(vcl_string const& fname, char mode = 'r' );
  virtual bool save(vidl1_movie* movie, vcl_string const& fname) { return false; }
  virtual vcl_string type() const { return "FFMPEG"; }
  virtual vidl1_ffmpeg_codec* castto_vidl1_ffmpeg_codec() { return this; }

  //: Advance to the next frame
  bool advance() const;

  //: Seek to a given frame number.
  bool seek( unsigned frame ) const;

  //: Enable or disable deinterlacing of the frames.
  void set_deinterlace( bool deinterlace ) {deinterlace_ = deinterlace;}

  //: Current frame number
  int cur_frame_num() const;


 protected:

  //: Read the current frame.
  vil_image_view<vxl_byte> cur_frame() const;

  bool open(vcl_string const& fname, char mode);
  void close();

  //: initialize ffmpeg
  static void initialize();

 private:

  //: count the number of frames in the video
  int count_frames() const;
 
  AVFormatContext* fmt_cxt_;
  int vid_index_;
  AVStream* vid_str_;

  //: Decode time of last frame.
  mutable int64_t last_dts;

  //: The last successfully read frame.
  //
  //  If frame_->data[0] is not NULL, then the frame corresponds to
  //  the codec state, so that codec.width and so on apply to the
  //  frame data.
  mutable AVFrame* frame_;

  //: The last successfully decoded frame.
  //
  //  If not null, it corresponds to the data in frame_. It may be
  //  null if we advance() but don't call cur_frame().
  mutable vil_image_view<vxl_byte> cur_img_;

  //: Apply deinterlacing on the frames?
  bool deinterlace_;

  //: Some codec/file format combinations need a frame number offset.
  //  These codecs have a delay between reading packets and generating frames.
  mutable unsigned frame_number_offset_;
};

#endif // vidl1_ffmpeg_codec_h
