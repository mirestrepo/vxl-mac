#ifndef VSOL_RECTANGLE_2D_H
#define VSOL_RECTANGLE_2D_H
//*****************************************************************************
//
// .NAME vsol_rectangle_2d - Rectangle of a 2D space
// .LIBRARY vsol
// .INCLUDE vsol/vsol_rectangle_2d.h
// .FILE    vsol/vsol_rectangle_2d.cxx
//
// .SECTION Description
// The vertices are defined in the counterclockwise.
// .SECTION Author
// Fran�ois BERTEL
//
// .SECTION Modifications
// 2000/06/17 Peter Vanroose  Implemented all operator==()s and type info
// 2000/05/08 Fran�ois BERTEL Creation
//*****************************************************************************

class vsol_rectangle_2d;

//*****************************************************************************
// External declarations for values
//*****************************************************************************
#include <vsol/vsol_rectangle_2d_sptr.h>
#include <vsol/vsol_polygon_2d.h>

class vsol_rectangle_2d
  :public vsol_polygon_2d
{
  //***************************************************************************
  // Initialization
  //***************************************************************************
public:
  //---------------------------------------------------------------------------
  //: Constructor from 3 points.
  //: `new_p0' is the origin of the rectangle. `new_p1' defines the abscissa
  //: axis and the width. `new_p2' defines the ordinate axis and the height.
  //: REQUIRE: valid_vertices(new_p0,new_p1,new_p2)
  //---------------------------------------------------------------------------
  explicit vsol_rectangle_2d(const vsol_point_2d_sptr &new_p0,
                             const vsol_point_2d_sptr &new_p1,
                             const vsol_point_2d_sptr &new_p2);

  //---------------------------------------------------------------------------
  //: Copy constructor
  //---------------------------------------------------------------------------
  vsol_rectangle_2d(const vsol_rectangle_2d &other);

  //---------------------------------------------------------------------------
  //: Destructor
  //---------------------------------------------------------------------------
  virtual ~vsol_rectangle_2d();

  //---------------------------------------------------------------------------
  //: Clone `this': creation of a new object and initialization
  //: See Prototype pattern
  //---------------------------------------------------------------------------
  virtual vsol_spatial_object_2d_sptr clone(void) const;

  //***************************************************************************
  // Access
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Return the first vertex
  //---------------------------------------------------------------------------
  virtual vsol_point_2d_sptr p0(void) const;

  //---------------------------------------------------------------------------
  //: Return the second vertex
  //---------------------------------------------------------------------------
  virtual vsol_point_2d_sptr p1(void) const;

  //---------------------------------------------------------------------------
  //: Return the third vertex
  //---------------------------------------------------------------------------
  virtual vsol_point_2d_sptr p2(void) const;

  //---------------------------------------------------------------------------
  //: Return the last vertex
  //---------------------------------------------------------------------------
  virtual vsol_point_2d_sptr p3(void) const;

  //***************************************************************************
  // Comparison
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Has `this' the same points than `other' in the same order ?
  //---------------------------------------------------------------------------
  virtual bool operator==(const vsol_rectangle_2d &other) const;
  virtual bool operator==(const vsol_polygon_2d &other) const; // virtual of vsol_polygon_2d
  virtual bool operator==(const vsol_spatial_object_2d& obj) const; // virtual of vsol_spatial_object_2d

  //***************************************************************************
  // Status report
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Compute the bounding box of `this'
  //---------------------------------------------------------------------------
  virtual void compute_bounding_box(void);

  //---------------------------------------------------------------------------
  //: Return the width
  //---------------------------------------------------------------------------
  virtual double width(void) const;

  //---------------------------------------------------------------------------
  //: Return the height
  //---------------------------------------------------------------------------
  virtual double height(void) const;

  //---------------------------------------------------------------------------
  //: Return the area of `this'
  //---------------------------------------------------------------------------
  virtual double area(void) const;

  //---------------------------------------------------------------------------
  //: Are `new_vertices' valid to build a rectangle ?
  //---------------------------------------------------------------------------
  virtual bool valid_vertices(const vcl_vector<vsol_point_2d_sptr> new_vertices) const;
};

#endif // #ifndef VSOL_RECTANGLE_2D_H
