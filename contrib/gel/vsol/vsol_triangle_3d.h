#ifndef vsol_triangle_3d_h
#define vsol_triangle_3d_h
//*****************************************************************************
//:
//  \file
// \brief Triangle of a 3D space.
//
// The vertices order gives the orientation of the triangle
//
// \author
// Fran�ois BERTEL
//
// \verbatim
// Modifications
// 2001/07/03 Peter Vanroose  Replaced vnl_double_3 by vgl_vector_3d
// 2000/06/17 Peter Vanroose  Implemented all operator==()s and type info
// 2000/05/04 Fran�ois BERTEL Creation
// \endverbatim
//*****************************************************************************

class vsol_triangle_3d;

//*****************************************************************************
// External declarations for values
//*****************************************************************************
#include <vsol/vsol_triangle_3d_sptr.h>
#include <vsol/vsol_polygon_3d.h>

class vsol_triangle_3d
  :public vsol_polygon_3d
{
  //***************************************************************************
  // Initialization
  //***************************************************************************
public:
  //---------------------------------------------------------------------------
  //: Constructor from its 3 vertices
  //---------------------------------------------------------------------------
  explicit vsol_triangle_3d(const vsol_point_3d_sptr &new_p0,
                            const vsol_point_3d_sptr &new_p1,
                            const vsol_point_3d_sptr &new_p2);

  //---------------------------------------------------------------------------
  //: Copy constructor
  //---------------------------------------------------------------------------
  vsol_triangle_3d(const vsol_triangle_3d &other);

  //---------------------------------------------------------------------------
  //: Destructor
  //---------------------------------------------------------------------------
  virtual ~vsol_triangle_3d();

  //---------------------------------------------------------------------------
  //: Clone `this': creation of a new object and initialization
  //  See Prototype pattern
  //---------------------------------------------------------------------------
  virtual vsol_spatial_object_3d_sptr clone(void) const;

  //***************************************************************************
  // Access
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Return the first vertex
  //---------------------------------------------------------------------------
  virtual vsol_point_3d_sptr p0(void) const;

  //---------------------------------------------------------------------------
  //: Return the second vertex
  //---------------------------------------------------------------------------
  virtual vsol_point_3d_sptr p1(void) const;

  //---------------------------------------------------------------------------
  //: Return the last vertex
  //---------------------------------------------------------------------------
  virtual vsol_point_3d_sptr p2(void) const;

  //***************************************************************************
  // Comparison
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Has `this' the same points than `other' in the same order ?
  //---------------------------------------------------------------------------
  virtual bool operator==(const vsol_triangle_3d &other) const;
  virtual bool operator==(const vsol_polygon_3d &other) const; // virtual of vsol_polygon_3d
  virtual bool operator==(const vsol_spatial_object_3d& obj) const; // virtual of vsol_spatial_object_3d

  //***************************************************************************
  // Status report
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Return the area of `this'
  //---------------------------------------------------------------------------
  virtual double area(void) const;

  //***************************************************************************
  // Element change
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Set the first vertex
  //---------------------------------------------------------------------------
  virtual void set_p0(const vsol_point_3d_sptr &new_p0);

  //---------------------------------------------------------------------------
  //: Set the second vertex
  //---------------------------------------------------------------------------
  virtual void set_p1(const vsol_point_3d_sptr &new_p1);

  //---------------------------------------------------------------------------
  //: Set the last vertex
  //---------------------------------------------------------------------------
  virtual void set_p2(const vsol_point_3d_sptr &new_p2);

  //***************************************************************************
  // Basic operations
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Is `p' in `this' ?
  //---------------------------------------------------------------------------
  virtual bool in(const vsol_point_3d_sptr &p) const;

  //---------------------------------------------------------------------------
  //: Return the unit normal vector at point `p'.
  //  Has to be deleted manually. Depends on the vertices order. If some
  //  vertices are aligned, the normal is the null vector.
  //  REQUIRE: in(p)
  //---------------------------------------------------------------------------
  virtual vgl_vector_3d<double>
  normal_at_point(const vsol_point_3d_sptr &p) const;
};

#endif // vsol_triangle_3d_h
