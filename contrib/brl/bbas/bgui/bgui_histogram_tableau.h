// This is brl/bbas/bgui/bgui_histogram_tableau.h
#ifndef bgui_histogram_tableau_h_
#define bgui_histogram_tableau_h_
//:
// \file
// \author  Matt Leotta
// \brief   A tableau to a histogram for an image
//
//  The histogram is plotted on a easy2D tableau

#include <vil1/vil1_memory_image_of.h>
#include <vil1/vil1_rgb.h>
#include <vgui/vgui_tableau.h>
#include <vgui/vgui_event.h>
#include <vgui/vgui_soview2D.h>
#include <vgui/vgui_easy2D_tableau_sptr.h>

class vgui_tableau;

#include <bgui/bgui_histogram_tableau_sptr.h>


class bgui_histogram_tableau : public vgui_tableau
{
public:

  //: Constructor takes all the labels used on the graph.
  bgui_histogram_tableau();
  //: Destructor.
  ~bgui_histogram_tableau();
  //: Update the histogram
  void update(vil1_memory_image_of< vil1_rgb<unsigned char> >& img);
  //: Return the name of this tableau.
  vcl_string type_name() const { return "bgui_histogram_tableau";}
  //: Handles all events for this tableau.
  bool handle(const vgui_event&);
  //: Clear the data
  void clear();

protected:

private:
  int left_offset_;
  int top_offset_;
  int graph_width_;
  int graph_height_;
  //: List of points.
  vcl_vector<float> xpoints_, ypoints_;
  vgui_easy2D_tableau_sptr easy_;
  vgui_soview2D_linestrip* plot_;
  vcl_vector<double> data_;
};


// <vgui_make_sptr>
struct bgui_histogram_tableau_new : public bgui_histogram_tableau_sptr
{
  typedef bgui_histogram_tableau_sptr base;
  bgui_histogram_tableau_new()
    : base(new bgui_histogram_tableau()) { }
};
// <vgui_make_sptr>


#endif //bgui_histogram_tableau.
