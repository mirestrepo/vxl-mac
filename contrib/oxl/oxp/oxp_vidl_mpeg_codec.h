#ifndef oxp_vidl_mpeg_codec_h
#define oxp_vidl_mpeg_codec_h
//:
// \file
// \author awf
// \date Dec 2001
//
// \verbatim
//  Modifications
//   10 Sep. 2004 Peter Vanroose  Inlined all 1-line methods in class decl
// \endverbatim

#include <vidl_vil1/vidl_vil1_codec.h>
#include <oxp/oxp_mpeg_codec.h>

//: Allows user to load MPEG files as vxl video.
// use set_demux if mpeg is a VOB
class oxp_vidl_mpeg_codec : public vidl_vil1_codec
{
 public:
  //-----------------------------------------------------

  virtual bool   get_section(int position, void* ib, int x0, int y0, int xs, int ys) const {
    return p.get_section(position, ib, x0, y0, xs, ys);
  }
  virtual int    put_section(int position, void* ib, int x0, int y0, int xs, int ys) {
    return p.put_section(position, ib, x0, y0, xs, ys);
  }

  //-----------------------------------------------------
  virtual bool probe(vcl_string const& fname) { return p.probe(fname); }
  virtual vidl_vil1_codec_sptr load(vcl_string const& fname, char mode = 'r' );
  virtual bool save(vidl_vil1_movie* movie, vcl_string const& fname);
  virtual vcl_string type() const { return "MPEG"; }

  // Call before destruction to a void segv on exit
  void close() { p.close(); }

 private:
  oxp_mpeg_codec p;
};

#endif // oxp_vidl_mpeg_codec_h
