// This is gel/gst/gst_polygon_2d.h
#ifndef gst_polygon_2d_h_
#define gst_polygon_2d_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \author crossge@crd.ge.com

#include <vcl_iostream.h>
#include <vcl_vector.h>
#include <vbl/vbl_ref_count.h>
#include "gst_edge_2d_sptr.h"

class gst_polygon_2d : public vbl_ref_count
{
 public:

  // constructors and destructors
  gst_polygon_2d() {}
  ~gst_polygon_2d() {}

  // getters and setters
  int size() const { return edges_.size(); }
  gst_edge_2d_sptr operator[]( const int i) const { return edges_[i]; }
  void add( const gst_edge_2d_sptr edge) { edges_.push_back( edge); }

  //: check closure of edges
  //   returns false if the edges are not closed or
  //                 if there is more than one possible cycle
  bool check_validity() const;

  // useful computational accessors
  bool inside( const double x, const double y) const;
  bool inside( const gst_vertex_2d_sptr v) const;

  double get_centroid_x() const;
  double get_centroid_y() const;

  double area() const;

  // output
  friend vcl_ostream &operator<<( vcl_ostream &os, gst_polygon_2d &p);

 protected:

  //: unordered, but oriented, list of edges
  vcl_vector<gst_edge_2d_sptr> edges_;
};

vcl_ostream &operator<<( vcl_ostream &os, gst_polygon_2d &p);

#endif // gst_polygon_2d_h_
