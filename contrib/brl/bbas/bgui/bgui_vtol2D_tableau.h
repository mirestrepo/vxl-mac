// This is brl/bbas/bgui/bgui_vtol2D_tableau.h
#ifndef bgui_vtol2D_tableau_h_
#define bgui_vtol2D_tableau_h_
//-----------------------------------------------------------------------------
//:
// \file
// \brief A child tableau of bgui_vsol2D_tableau that knows how to display vtol objects.
// \author
//   J.L. Mundy
//
//   Default styles are defined for each geometry and topology object soview.
//   Users can change the default style by using the set_*_style commands,
//   e.g. set_edge_style(0.0, 0.5, 0.5, 3) will define the style for each
//   new edge added to the display.  This default is the style that the
//   edge appearance will resume after being highlighted.
//
// \verbatim
//  Modifications:
//   J.L. Mundy November 28, 2002    Initial version.
//   J.L. Mundy December 16, 2002    Added map between soviews and vtol objects
//   J.L. Mundy March 22, 2003       Added set style commands
//   Amir Tamrakar April 22, 2002    Removed the functions to display vsol objects
//                                   to bgui_vsol2D_tableau and subclasses this
//                                   from it instead
//   Mark Johnson June 13, 2003      Stopped using interior class functions to
//                                   highlight objects. Added support for
//                                   specifying colors of individual objects.
// \endverbatim
//-----------------------------------------------------------------------------
#include <vcl_vector.h>
#include <vcl_map.h>
#include <vcl_string.h>
#include <vgui/vgui_style.h>
#include <vtol/vtol_vertex_2d.h>
#include <vtol/vtol_edge_2d.h>
#include <vtol/vtol_vertex_2d_sptr.h>
#include <vtol/vtol_edge_2d_sptr.h>
#include <vtol/vtol_face_2d_sptr.h>
#include <vtol/vtol_topology_object_sptr.h>
#include <vgui/vgui_tableau_sptr.h>
#include <vgui/vgui_image_tableau_sptr.h>
#include <vgui/vgui_rubberband_tableau.h>
#include <bgui/bgui_easy2D_tableau.h>
#include <bgui/bgui_vsol2D_tableau.h>
#include <bgui/bgui_vsol2D_tableau_sptr.h>
#include <bgui/bgui_vtol2D_tableau_sptr.h>

class bgui_vtol_soview2D_vertex;
class bgui_vtol_soview2D_edge;
class bgui_vtol_soview2D_edge_group;
class bgui_vtol_soview2D_face;

class bgui_vtol2D_tableau : public bgui_vsol2D_tableau
{
 public:
  bgui_vtol2D_tableau(const char* n="unnamed");

  bgui_vtol2D_tableau(vgui_image_tableau_sptr const& it,
                      const char* n="unnamed");

  bgui_vtol2D_tableau(vgui_tableau_sptr const& t,
                      const char* n="unnamed");

  ~bgui_vtol2D_tableau();

  virtual vcl_string type_name() const;

  //:virtual handle method for events
  virtual bool handle(vgui_event const &);

  //: the vtol display methods for individual topology classes
  bgui_vtol_soview2D_vertex* add_vertex(vtol_vertex_2d_sptr const& v);

  bgui_vtol_soview2D_vertex* add_vertex(vtol_vertex_2d_sptr const& v,
                                        const float r,
                                        const float g,
                                        const float b,
                                        const float point_radius );

  bgui_vtol_soview2D_edge* add_edge(vtol_edge_2d_sptr const& e);

  bgui_vtol_soview2D_edge* add_edge(vtol_edge_2d_sptr const& e,
                                    const float r,
                                    const float g,
                                    const float b,
                                    const float line_width );

  bgui_vtol_soview2D_edge_group* add_edge_group(vcl_vector<vtol_edge_2d_sptr>&
                                                edges);

  bgui_vtol_soview2D_edge_group* add_edge_group(vcl_vector<vtol_edge_2d_sptr>&
                                                edges,
                                                const float r,
                                                const float g,
                                                const float b,
                                                const float line_width);

  bgui_vtol_soview2D_face* add_face(vtol_face_2d_sptr const& f);

  bgui_vtol_soview2D_face* add_face(vtol_face_2d_sptr const& f,
                                    const float r,
                                    const float g,
                                    const float b,
                                    const float line_width);

  //: display methods for vectors of topology classes (not grouped)
  void add_topology_object(vtol_topology_object_sptr const& tos);

  void add_topology_object(vtol_topology_object_sptr const& tos,
                           const float r,
                           const float g,
                           const float b,
                           const float line_width,
                           const float point_radius);

  void add_topology_objects(vcl_vector<vtol_topology_object_sptr> const& tos);

  void add_topology_objects(vcl_vector<vtol_topology_object_sptr> const& tos,
                            const float r,
                            const float g,
                            const float b,
                            const float line_width,
                            const float point_radius);

  void add_edges(vcl_vector<vtol_edge_2d_sptr> const & edges,
                 bool verts=false);

  void add_edges(vcl_vector<vtol_edge_2d_sptr> const & edges,
                 bool verts,
                 const float r,
                 const float g,
                 const float b,
                 const float line_width,
                 const float point_radius);

  void add_faces(vcl_vector<vtol_face_2d_sptr> const & faces, bool verts=false);

  void add_faces(vcl_vector<vtol_face_2d_sptr> const & faces, bool verts,
                 const float r,
                 const float g,
                 const float b,
                 const float line_width,
                 const float point_radius);

  //: clear the tableau including the highlight map
  void clear_all();

  //: Methods for getting mapped objects
  //void enable_highlight(){highlight_ = true;}
  //void disable_highlight(){highlight_ = false;}
  vtol_edge_2d_sptr get_mapped_edge(const int id);

  //: Methods for changing the default style of displayable objects
  void set_vtol_topology_object_style(vtol_topology_object_sptr tos,
                                      const float r, const float g, const float b,
                                      const float line_width, const float point_radius);
  void set_vertex_style(const float r, const float g, const float b,
                        const float point_radius);
  void set_edge_style(const float r, const float g, const float b,
                      const float line_width);
  void set_edge_group_style(const float r, const float g, const float b,
                            const float line_width);
  void set_face_style(const float r, const float g, const float b,
                      const float line_width);

  //: Access to temporary cached object, useful for new rubberbanded objects
  void set_temp(vtol_topology_object_sptr const& to){temp_=to;}
  vtol_topology_object_sptr get_temp(){return temp_;}
 protected:
  DefaultStyle vertex_style_;
  DefaultStyle edge_style_;
  DefaultStyle edge_group_style_;
  DefaultStyle face_style_;
  void init();
  vtol_topology_object_sptr temp_; //temporary storage for a topology object
  vcl_map<int, vtol_topology_object_sptr> obj_map_;
};

//this stuff is needed to establish inheritance between tableau  smart pointers
//cloned from xcv_image_tableau
struct bgui_vtol2D_tableau_new : public bgui_vtol2D_tableau_sptr
{
  typedef bgui_vtol2D_tableau_sptr base;

  bgui_vtol2D_tableau_new(const char* n="unnamed") :
    base(new bgui_vtol2D_tableau(n)) { }
  bgui_vtol2D_tableau_new(vgui_image_tableau_sptr const& it,
                          const char* n="unnamed") :
    base(new bgui_vtol2D_tableau(it,n)) { }

  bgui_vtol2D_tableau_new(vgui_tableau_sptr const& t, const char* n="unnamed")
    :  base(new bgui_vtol2D_tableau(t, n)) { }

  operator bgui_vsol2D_tableau_sptr () const { bgui_vsol2D_tableau_sptr tt; tt.vertical_cast(*this); return tt; }
};

//A client for rubberbanding stuff
class bgui_vtol2D_rubberband_client : public vgui_rubberband_client
{
 public:

  bgui_vtol2D_tableau_sptr vtol2D_;

  //:constructor - takes a pointer to a vtol2D tableau
  bgui_vtol2D_rubberband_client(bgui_vtol2D_tableau_sptr const& vtol2D);

    //: Called by vgui_rubberband_tableau when the user has selected a point.
  virtual void add_point(float, float);

  //: Called by vgui_rubberband_tableau when the user has selected a finite line.
  virtual void add_line(float,float,float,float);

  //: Called by vgui_rubberband_tableau when user has selected an infinite line.
  virtual void add_infinite_line(float,float,float);

  //: Called by vgui_rubberband_tableau when the user has selected a circle.
  virtual void add_circle(float,float,float);

  //: Called by vgui_rubberband_tableau when the user has selectd a linestrip.
  virtual void add_linestrip(int n,float const *,float const *);

  //: Called by vgui_rubberband_tableau when the user has selected a polygon.
  virtual void add_polygon(int n,float const*,float const*);

  //: Called by vgui_rubberband_tableau when user has selected a rectangular box
  virtual void add_box(float,float,float,float);

  //: Called by vgui_rubberband_tableau whenever mouse motion is captured.
  //  This turns off the highlighting of geometry objects to eliminate
  //  flickering highlights while drawing temporary objects.
  void clear_highlight();

};

#endif // bgui_vtol2D_tableau_h_
