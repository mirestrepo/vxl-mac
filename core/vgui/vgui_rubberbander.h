// This is oxl/vgui/vgui_rubberbander.h
#ifndef vgui_rubberbander_h_
#define vgui_rubberbander_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \author  K.Y.McGaul
// \date    31-MAR-2000
// \brief   Tableau to rubberband circles, lines etc.  
//
//  Contains classes: vgui_rubberbander  vgui_rubberbander_new
//                    vgui_rubberbander_client  vgui_rubberbander_easy2D_client
//
// \verbatim
//  Modifications
//   K.Y.McGaul     31-MAR-2000    Initial version.
//   F.S. de M.     31-MAR-2000    Minor cosmetic changes.
//   Marko Bacic    07-JUL-2000    Added support for linestrip
//   Marko Bacic    19-JUL-2000    Now supports vgui_rubberbander_client
//   FSM            14-AUG-2000    Fixed so that it works with Windows
// \endverbatim


#include <vgui/vgui_tableau.h>
#include <vgui/vgui_easy2D.h>
#include <vgui/vgui_event_condition.h>

//: Receives the parameters captured by vgui_rubberbander.
class vgui_rubberbander_client
{
 public:
  virtual ~vgui_rubberbander_client() { }
  virtual void add_point(float, float);
  virtual void add_line(float,float,float,float);
  virtual void add_infinite_line(float,float,float);
  virtual void add_circle(float,float,float);
  virtual void add_linestrip(int n,float const *,float const *);
  virtual void add_polygon(int n,float const*,float const*);
  virtual void add_box(float,float,float,float);
};

//: Rubberbanding onto a vgui_easy2D.
//
//  Special case of rubberbander_client for cases where we just want to draw
//  rubberbanded objects straight onto an easy2D.
class vgui_rubberbander_easy2D_client : public vgui_rubberbander_client
{
 public:
  vgui_easy2D_sptr easy;
  vgui_rubberbander_easy2D_client(vgui_easy2D_sptr const& e): easy(e) { }

  void add_point(float x, float y){easy->add_point(x,y);}
  void add_line(float x0, float y0, float x1, float y1){easy->add_line(x0, y0, x1, y1);}
  void add_infinite_line(float a, float b, float c){easy->add_infinite_line(a, b, c);}
  void add_circle(float x, float y, float r){easy->add_circle(x, y, r);}
  void add_linestrip(int n, float const* x, float const* y){easy->add_linestrip(n, x, y);}
  void add_polygon(int n, float const* x, float const* y){easy->add_polygon(n, x, y);}
  void add_box(float, float, float, float){}
};

//: Tableau to rubberband circles, lines etc.  
//
//  The values captured (eg. two points defining a line) are passed to the 
//  appropriate function (eg. add_line) in the client passed in to the 
//  constructor.  This client is derived from vgui_rubberbander_client.
//
//  In more detail:
//
//  The user draws the object in the rendering area using the defined
//  'gestures'.  For example, to draw a line, the first gesture is a mouse
//  down event defining the start point.  A line is shown on the display
//  between the start point and the mouse pointer until the second gesture
//  is performed (releasing the mouse button) and this defines the end point.
//  
//  These values are passed to add_line in the client.  These values could
//  be used to draw the line (like vgui_rubberbander_easy2D_client) or
//  you can create your own client derived from vgui_rubberbander_client to
//  do something different.
//
//  See xcv to see a rubberbander in action (used to add geometric objects).
//
//  If you want to get values from a user and you want your code to wait
//  until the values have been collected then rubberbander won't do this.
//  You need to write something like ./oxl/xcv/xcv_picker_tableau that
//  grabs the event loop.
class vgui_rubberbander : public vgui_tableau
{
 public:
  bool use_overlays;  // capes@robots - default is true

  void init (vgui_rubberbander_client* client);

  //: Constructor - don't use this, use vgui_rubberbander_new.
  vgui_rubberbander(vgui_rubberbander_client* client);

  //vgui_rubberbander(vgui_easy2D_sptr const&);

  //: Return the type of this tableau ('vgui_rubberbander').
  vcl_string type_name() const { return "vgui_rubberbander"; }

  // these describe what the user has to do
  // to use the rubberbanding gesture.
  vgui_event_condition gesture0;
  vgui_event_condition gesture1;
  vgui_event_condition gesture2;

  void rubberband_point();
  void rubberband_line();
  void rubberband_infinite_line();
  void rubberband_circle();
  void rubberband_polygon();
  void rubberband_linestrip(); // u97mb
  void rubberband_box();
  void rubberband_none();

  vgui_rubberbander_client* get_client(){return client_;}
  void set_client(vgui_rubberbander_client *);

  void draw_point(float x0, float y0);
  void draw_line(float x0, float y0, float x1, float y1);
  void draw_infinite_line(float a, float b, float c); // ax + by + c = 0
  void draw_circle(float x0, float y0, float r);
  void draw_linestrip(float x,float y); // u97mb
  void draw_polygon(float x, float y);
  void draw_box(float x0,float y0, float x1,float y1);

 protected:
  ~vgui_rubberbander() { }
  bool handle_point(vgui_event const&, float, float);
  bool handle_line(vgui_event const&, float, float);
  bool handle_linestrip(vgui_event const&,float , float ); // u97mb
  bool handle_infinite_line(vgui_event const&, float, float);
  bool handle_circle(vgui_event const&, float, float);
  bool handle_polygon(vgui_event const&, float, float);
  bool handle_box(vgui_event const&, float, float);
  bool handle(vgui_event const&);

 private:
  vgui_rubberbander_client *client_;
  enum object_type {none_enum, point_enum, line_enum, infinite_line_enum, circle_enum, polygon_enum, linestrip_enum,box_enum};
  bool active;
  static object_type obj_type;
  float lastx, lasty;   // position where mouse was last seen.
  vcl_vector<float>x_coords, y_coords;
};

typedef vgui_tableau_sptr_t<vgui_rubberbander> vgui_rubberbander_sptr;

//: Creates a smart-pointer to a vgui_rubberbander tableau.
struct vgui_rubberbander_new : public vgui_rubberbander_sptr
{
  vgui_rubberbander_new(vgui_rubberbander_client* client)
    : vgui_rubberbander_sptr(new vgui_rubberbander(client)) { }
  //vgui_rubberbander_new(vgui_easy2D_sptr const&e)   : vgui_rubberbander_sptr(new vgui_rubberbander(e)) { }
};

#endif // vgui_rubberbander_h_
