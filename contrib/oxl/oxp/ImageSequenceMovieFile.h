#ifndef ImageSequenceMovieFile_h_
#define ImageSequenceMovieFile_h_
#ifdef __GNUC__
#pragma interface
#endif
//
// .NAME	ImageSequenceMovieFile - MovieFile subclass
// .LIBRARY	POX
// .HEADER	Oxford Package
// .INCLUDE	oxp/ImageSequenceMovieFile.h
// .FILE	ImageSequenceMovieFile.cxx
//
// .SECTION Description
//    ImageSequenceMovieFile is a subclass of MovieFileInterface that
//    reads from a sequence of images.
//
// .SECTION Author
//     Andrew W. Fitzgibbon, Oxford RRG, 31 Dec 98
//
//-----------------------------------------------------------------------------

#include <oxp/MovieFileInterface.h>

struct ImageSequenceMovieFilePrivates;

struct ImageSequenceMovieFile : public MovieFileInterface {

  ImageSequenceMovieFile(char const* filepattern, int frame_index_to_search_for_extension);
  ~ImageSequenceMovieFile();

  int GetLength();
  int GetSizeX(int);
  int GetSizeY(int);
  int GetBitsPixel();
  bool IsInterlaced();
  bool HasFrame(int);
  vil_image GetImage(int);

  bool GetFrame(int frame_index, void* buffer);
  bool GetField(int field_index, void* buffer);

private:
  ImageSequenceMovieFilePrivates* p;
};

#endif // ImageSequenceMovieFile_h_
