//-*- c++ -*-------------------------------------------------------------------
#ifdef __GNUC__
#pragma implementation
#endif
//
// Class: vbl_awk
// Author: Andrew W. Fitzgibbon, Oxford RRG
// Created: 17 May 97
//
//-----------------------------------------------------------------------------

#include "vbl_awk.h"

#include <vcl/vcl_cctype.h>
#include <vcl/vcl_iostream.h>

// -- Construct from input stream
vbl_awk::vbl_awk(istream& s, ModeFlags mode):
  fd_(s),
  mode_(mode)
{
  done_ = false;
  line_number_ = 0;
  split_line_ = 0;
  
  next();
}

vbl_awk::~vbl_awk() 
{
  delete [] split_line_;
}

void vbl_awk::next()
{
  // Read line -- should be quite fast after the first one.
  line_.erase();
  int l = 0;
  //bool do_backslash_continuations = (int(mode_) & int(backslash_continuations)) != 0;
  //  bool do_strip_comments = (int(mode_) & int(strip_comments)) != 0;
  
  while (1) {
    int c = fd_.get();
    if (c == EOF) {
      done_ = true;
      break;
    }

    if (c == '\n')
      break;

    line_ += c;
    ++l;
  }

  char const* linep = line_.c_str();
  
  // copy string
  delete [] split_line_;
  split_line_ = new char[line_.size() + 1];
  strcpy(split_line_, linep);

  // Chop line up into fields
  fields_.clear();
  char* cp = split_line_;

  for(;;) {
    // Eat white
    while (*cp && isspace(*cp))
      ++cp;
    if (!*cp) break;
  
    // Push
    fields_.push_back(cp);

    // Find nonwhite
    while (*cp && !isspace(*cp))
      ++cp;
    if (!*cp) break;

    // Zap space
    *cp++ = '\0';
  }

  // Increment line number
  ++line_number_;
}

char const* vbl_awk::line_from(int field_number) const
{
  char const *p = line_.c_str();
  if (field_number >= NF())
    field_number = NF() - 1;
  if (field_number < 0) {
    vcl_cerr << "vbl_awk::line_from("<< field_number <<") -- ZOIKS\n";
    return line();
  }
  
  return p + (fields_[field_number] - split_line_);
}

void testvbl_awk()
{
  vcl_cout << "Start\n";
  for(vbl_awk awk(vcl_cin); awk; ++awk) {
    vcl_cout << awk.NF() << ":" << awk[2] << vcl_endl;
  }
}
