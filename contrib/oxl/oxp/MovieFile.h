#ifndef MovieFile_h_
#define MovieFile_h_
#ifdef __GNUC__
#pragma interface
#endif
//:
// \file
// \brief Read various movie formats
//    MovieFile is an interface to movie files and image sequences.
//    It is currently read-only.
// \author
//     Andrew W. Fitzgibbon, Oxford RRG, 26 Aug 98
//
//-----------------------------------------------------------------------------

#include <vcl_fstream.h>
#include <vcl_string.h>

#include <vil/vil_memory_image_of.h>
#include <vil/vil_byte.h>
#include <vil/vil_rgb.h>

struct MovieFileInterface;

//: an interface to movie files and image sequences.  It is currently read-only
class MovieFile {
public:
  // Constructors/Destructors--------------------------------------------------

  MovieFile(char const * filename, int start = 0, int step = 1, int end = -1);
  ~MovieFile();

  int GetLength();
  int GetSizeX(int frame_index = 0);
  int GetSizeY(int frame_index = 0);
  int GetBitsPixel();

  int GetRealFrameIndex(int frame) { return start_ + frame * step_; }
  int GetNumFrames();

  vil_image GetImage(int frame_index);

  bool HasFrame(int frame_index);
  void GetFrame(int frame_index, vil_rgb<unsigned char> * frame);
  void GetFrame(int frame_index, vil_byte* frame);
  void GetFrame(int frame_index, vil_memory_image_of<vil_rgb<unsigned char> >& frame);
  void GetFrame(int frame_index, vil_memory_image_of<vil_byte>& frame);
  void GetFrameRGB(int frame_index, vil_byte* frame);

  bool ok() { return qt != 0; }

protected:
  vcl_string filename_;
  int start_, step_, end_;
  MovieFileInterface* qt;
  vil_memory_image_of<vil_rgb<unsigned char> >* tmp_buf_;

  int index(int);
};

#endif // MovieFile_h_
