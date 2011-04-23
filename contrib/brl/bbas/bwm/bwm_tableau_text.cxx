#include "bwm_tableau_text.h"

#include <vcl_iostream.h>
#include <vcl_fstream.h>

void bwm_tableau_text::set_text(vcl_string filename)
{
  if (filename.empty()) {
      vcl_cout << "Bad filename\n";
      return;
  }

  vcl_ifstream is(filename.data());
  if(!is.is_open())  {
    vcl_cout << "Can't open file\n";
    return;
  }

  vcl_string str;
  unsigned length = ydim_ - yinc_;
  float ypos = ymarg_;
  char* s = new char(length);
  this->clear();
  this->set_size(1);
  while (!is.eof()) {
    is.getline(s, length);
    vcl_cout << s << vcl_endl;
    if (s[0] == '-')
      this->set_colour(1, 0, 0);
    else 
      this->set_colour(1, 1, 1);
    add(xmarg_, ypos, s);
    ypos += 15;
  }
}
void bwm_tableau_text::set_string(vcl_string & str)
{
  unsigned width = xdim_ - 2*xmarg_;
  this->set_colour(1, 1, 1);
  float ypos = ymarg_;
  unsigned int spos=0, fpos;
  while(ypos<(ydim_-ymarg_)){
    fpos= str.find('\n',spos);
    if(fpos >= str.size())
      break;
    unsigned nc = fpos-spos;
    vcl_string s = str.substr(spos, nc);
    add(xmarg_, ypos, s.c_str());
    ypos += 15;
    spos = fpos+1;
	if(spos >= str.size())
      break;
  }
}
