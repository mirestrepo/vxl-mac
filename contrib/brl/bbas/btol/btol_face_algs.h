#ifndef btol_face_algs_h_
#define btol_face_algs_h_
//-----------------------------------------------------------------------------
//:
// \file
// \author J.L. Mundy
// \brief topology algorithms involving faces or face-centric routines
// \verbatim
//  Modifications
//   Initial version October 30, 2003
// \endverbatim
//
//-----------------------------------------------------------------------------
#include <vgl/vgl_polygon.h>
#include <vsol/vsol_point_2d_sptr.h>
#include <vtol/vtol_edge_2d_sptr.h>
#include <vtol/vtol_face_2d_sptr.h>
#include <vcl_vector.h>

class btol_face_algs
{
 public:
  ~btol_face_algs();
  static bool vtol_to_vgl(vtol_face_2d_sptr const & face, vgl_polygon<float>& poly);
  static bool edge_intersects(vtol_face_2d_sptr const & face,
                              vtol_edge_2d_sptr const & edge);

  static bool intersecting_edges(vtol_face_2d_sptr const & face,
                                 vcl_vector<vtol_edge_2d_sptr> const & edges,
                                 vcl_vector<vtol_edge_2d_sptr>& inter_edges);

  //:only valid for face with straight line edges
  static vsol_point_2d_sptr centroid(vtol_face_2d_sptr const & face);

  //:create an axis-aligned box face
  static vtol_face_2d_sptr box(const double x0, const double y0,
                               const double width, const double height);
 private:
  btol_face_algs();
};

#endif
