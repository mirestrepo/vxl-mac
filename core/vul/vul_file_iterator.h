// This is vxl/vul/vul_file_iterator.h
#ifndef vul_file_iterator_h_
#define vul_file_iterator_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \brief class to iterate through directories and/or "glob" patterns (*.*)
// \author awf@robots.ox.ac.uk
// \date 27 Nov 00
//
// \verbatim
// Modifications
// PDA (Manchester) 21/03/2001: Tidied up the documentation
// Peter Vanroose   27/05/2001: Corrected the documentation
// \endverbatim

#include <vcl_string.h>

struct vul_file_iterator_data;

//: Iterate through directories and/or "glob" patterns (*.*)
// It is efficient to use
// \code
//   for (vul_file_iterator fn="/dir/*"; fn; ++fn) {
//     ... use fn() as filename
//   }
// \endcode
// simply to list the contents of a directory.  If you really
// want just the *.ext files, it is efficient to use
// \code
//   for (vul_file_iterator fn="/dir/*.ext"; fn; ++fn) {
//     ... use fn() as filename
//   }
// \endcode
// rather than opendir/glob/etc.
// Valid glob patterns are unix-like.

class vul_file_iterator
{
 public:

  vul_file_iterator();

  //: Initialize, and scan to get first file from "glob"
  vul_file_iterator(char const* glob);

  //: Initialize, and scan to get first file from "glob"
  vul_file_iterator(vcl_string const& glob);

  ~vul_file_iterator();

  //: Ask if done.
  // Won't spin the disk
  operator bool();

  //: Return the currently pointed-to pathname.
  // Won't spin the disk
  char const* operator()();

  //: Return the non-directory part of the current pathname.
  char const* filename();

  //: Return the match for the i'th glob wildcard character (* or ?).
  // Uses the most recent glob result.
  char const* match(int i);

  //: Increment to the next file
  // Will spin the disk
  vul_file_iterator& operator++();

  //: Run a new match
  void reset(char const* glob);

 protected:
  vul_file_iterator_data* p;

 private:
  // postfix++ privatized.
  vul_file_iterator operator++(int) { return vul_file_iterator(); }
};

#endif // vul_file_iterator_h_
