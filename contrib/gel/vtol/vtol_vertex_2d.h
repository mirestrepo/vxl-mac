#ifndef VTOL_VERTEX_2D_H
#define VTOL_VERTEX_2D_H
// .NAME vtol_vertex_2d - Topological container for a spatial point
// .LIBRARY vtol
// .HEADER gel Package
// .INCLUDE vtol/vtol_vertex_2d.h
// .FILE vtol_vertex_2d.cxx
//
// .SECTION Description
//  The vtol_vertex_2d class is used to represent either a 2D or 2D point on
//  a topological structure.  A vtol_vertex_2d maintains a pointer to the IUPoint
//  which is the actual spatial point.
//
// .SECTION Modifications:
//     JLM December 1995, Added timeStamp (touch) to
//                        operations which affect bounds.
//
//     JLM October 1996,  Added the method EuclideanDistance(vtol_vertex_2d &)
//     to permit Charlie Rothwell's Polyhedra code to be more
//     generic.  Note this is distance, NOT squared distance.
//     LEG May 2000. ported to vxl
//
/// .EXAMPLE vtol_vertex_2d.example
#include <vtol/vtol_vertex_2d_sptr.h>

#include <vnl/vnl_double_2.h>
#include <vsol/vsol_point_2d_sptr.h>
#include <vtol/vtol_vertex.h>

class vtol_vertex_2d
  : public vtol_vertex
{
public:
  //***************************************************************************
  // Initialization
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Default constructor
  //---------------------------------------------------------------------------
  explicit vtol_vertex_2d(void);

  //---------------------------------------------------------------------------
  //: Constructor from a point (the point is not copied)
  //: REQUIRE: new_point!=0
  //---------------------------------------------------------------------------
  explicit vtol_vertex_2d(vsol_point_2d &new_point);

  //---------------------------------------------------------------------------
  //: Constructor from a vector
  //---------------------------------------------------------------------------
  explicit vtol_vertex_2d(const vnl_double_2 &v);

  //---------------------------------------------------------------------------
  //: Constructor from abscissa `new_x' and ordinate `new_y' of the point
  //---------------------------------------------------------------------------
  explicit vtol_vertex_2d(const double new_x,
                          const double new_y);

  //---------------------------------------------------------------------------
  //: Copy constructor. Copy the point but not the links
  //---------------------------------------------------------------------------
  explicit vtol_vertex_2d(const vtol_vertex_2d &other);

  //---------------------------------------------------------------------------
  //: Destructor
  //---------------------------------------------------------------------------
  virtual ~vtol_vertex_2d();

  //---------------------------------------------------------------------------
  //: Clone `this': creation of a new object and initialization
  //: See Prototype pattern
  //---------------------------------------------------------------------------
  virtual vsol_spatial_object_3d_sptr clone(void) const;

  // Accessors

  //---------------------------------------------------------------------------
  //: Return the point
  //---------------------------------------------------------------------------
  virtual vsol_point_2d_sptr point(void) const;

  //---------------------------------------------------------------------------
  //: Set the point (the point is not copied)
  //: REQUIRE: new_point.ptr()!=0
  //---------------------------------------------------------------------------
  virtual void set_point(vsol_point_2d &new_point);



  // Methods called on Vertex
  // for vsol_point_2d.   These are here
  // during the transition period.
  // Looks like forever now - JLM

  //---------------------------------------------------------------------------
  //: Return the abscissa of the point
  //---------------------------------------------------------------------------
  virtual double x(void) const;

  //---------------------------------------------------------------------------
  //: Return the ordinate of the point
  //---------------------------------------------------------------------------
  virtual double y(void) const;

  //---------------------------------------------------------------------------
  //: Set the abscissa of the point with `new_x'
  //---------------------------------------------------------------------------
  virtual void set_x(const double new_x);

  //---------------------------------------------------------------------------
  //: Set the ordinate of the point with `new_y'
  //---------------------------------------------------------------------------
  virtual void set_y(const double new_y);

  //---------------------------------------------------------------------------
  //: Is `this' has the same coordinates for its point than `other' ?
  //---------------------------------------------------------------------------
  virtual bool operator==(const vtol_vertex_2d &other) const;
  bool operator==(const vsol_spatial_object_3d& obj) const; // virtual of vsol_spatial_object_2d

  //---------------------------------------------------------------------------
  //: Assignment of `this' with `other' (copy the point not the links)
  //---------------------------------------------------------------------------
  virtual vtol_vertex_2d& operator=(const vtol_vertex_2d &other);

  //***************************************************************************
  // Replaces dynamic_cast<T>
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Return `this' if `this' is a vertex, 0 otherwise
  //---------------------------------------------------------------------------
  virtual const vtol_vertex_2d *cast_to_vertex_2d(void) const;

  //---------------------------------------------------------------------------
  //: Return `this' if `this' is a vertex, 0 otherwise
  //---------------------------------------------------------------------------
  virtual vtol_vertex_2d *cast_to_vertex_2d(void);

  //---------------------------------------------------------------------------
  //: Create a line edge from `this' and `other' only if this edge does not
  //: exist. Otherwise it just returns the existing edge
  //: REQUIRE: other.ptr()!=0 and other.ptr()!=this
  //---------------------------------------------------------------------------

  virtual vtol_edge *new_edge(vtol_vertex &other);


  double distance_from(const vnl_double_2 &);

  double euclidean_distance(vtol_vertex_2d &v); //actual distance, not squared - JLM

  void print(vcl_ostream &strm=vcl_cout) const;
  void describe(vcl_ostream &strm=vcl_cout, int blanking=0) const;

protected:
  //---------------------------------------------------------------------------
  // Description: point associated to the vertex
  //---------------------------------------------------------------------------
  vsol_point_2d_sptr _point;

   //:  copy the geometry

  virtual void copy_geometry(const vtol_vertex &other);

  // : compare the geometry

  virtual bool compare_geometry(const vtol_vertex &other) const;


};

#endif
