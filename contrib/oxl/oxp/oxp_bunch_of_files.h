//-*- c++ -*-------------------------------------------------------------------
#ifndef oxp_bunch_of_files_h_
#define oxp_bunch_of_files_h_
#ifdef __GNUC__
#pragma interface
#endif
// Author: awf@robots.ox.ac.uk
// Created: 18 Dec 01

#include <vcl_cstdio.h>
#include <vcl_vector.h>
#include <vcl_string.h>

struct oxp_bunch_of_files {
  typedef __int64 offset_t;

  oxp_bunch_of_files();
  oxp_bunch_of_files(char const* fmt);
  ~oxp_bunch_of_files();

  bool open(char const* fmt);
  void close();

  void seek(offset_t to);
  int tell() const;
  int read(void*, unsigned int);
  bool ok() { return current_file_index != -1; }

private:
  int current_file_index;
  vcl_vector<vcl_string> filenames;
  vcl_vector<FILE*> fps;
  vcl_vector<int> filesizes;
  vcl_vector<offset_t> start_byte;
};

#endif   // DO NOT ADD CODE AFTER THIS LINE! END OF DEFINITION FOR CLASS oxp_oxp_bunch_of_files.

