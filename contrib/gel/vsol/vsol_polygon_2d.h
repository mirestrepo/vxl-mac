#ifndef VSOL_POLYGON_2D_H
#define VSOL_POLYGON_2D_H
//*****************************************************************************
//
// .NAME vsol_polygon_2d - Polygon of a 2D space
// .LIBRARY vsol
// .INCLUDE vsol/vsol_polygon_2d.h
// .FILE    vsol/vsol_polygon_2d.cxx
//
// .SECTION Description
// The vertices are defined in the counterclockwise.
// .SECTION Author
// Fran�ois BERTEL
//
// .SECTION Modifications
// 2000/06/17 Peter Vanroose  Implemented all operator==()s and type info
// 2000/05/09 Fran�ois BERTEL Creation
//*****************************************************************************

class vsol_polygon_2d;

//*****************************************************************************
// External declarations for values
//*****************************************************************************
#include <vsol/vsol_polygon_2d_ref.h>
#include <vsol/vsol_region_2d.h>

#include <vsol/vsol_point_2d_ref.h>
#include <vcl_vector.h>

class vsol_polygon_2d
  :public vsol_region_2d
{
  //***************************************************************************
  // Initialization
  //***************************************************************************
public:
  //---------------------------------------------------------------------------
  //: Constructor from a vcl_vector (not a geometric vector but a list of
  //: points)
  //: REQUIRE: new_vertices.size()>=3
  //---------------------------------------------------------------------------
  explicit vsol_polygon_2d(const vcl_vector<vsol_point_2d_ref> &new_vertices);

  //---------------------------------------------------------------------------
  //: Copy constructor
  //---------------------------------------------------------------------------
  vsol_polygon_2d(const vsol_polygon_2d &other);

  //---------------------------------------------------------------------------
  //: Destructor
  //---------------------------------------------------------------------------
  virtual ~vsol_polygon_2d();

  //---------------------------------------------------------------------------
  //: Clone `this': creation of a new object and initialization
  //: See Prototype pattern
  //---------------------------------------------------------------------------
  virtual vsol_spatial_object_2d_ref clone(void) const;

  //***************************************************************************
  // Access
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Return vertex `i'
  //: REQUIRE: valid_index(i)
  //---------------------------------------------------------------------------
  virtual vsol_point_2d_ref vertex(const int i) const;
  
  //***************************************************************************
  // Comparison
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Has `this' the same points than `other' in the same order ?
  //---------------------------------------------------------------------------
  virtual bool operator==(const vsol_polygon_2d &other) const;
  virtual bool operator==(const vsol_spatial_object_2d& obj) const; // virtual of vsol_spatial_object_2d

  //---------------------------------------------------------------------------
  //: Has `this' not the same points than `other' in the same order ?
  //---------------------------------------------------------------------------
  virtual bool operator!=(const vsol_polygon_2d &other) const;

  //***************************************************************************
  // Status report
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Return the region type of a polygon.  Its spatial type is a REGION
  //---------------------------------------------------------------------------
  vsol_region_2d_type region_type(void) const { return vsol_region_2d::POLYGON; }

  //---------------------------------------------------------------------------
  //: Compute the bounding box of `this'
  //---------------------------------------------------------------------------
  virtual void compute_bounding_box(void);

  //---------------------------------------------------------------------------
  //: Return the number of vertices
  //---------------------------------------------------------------------------
  virtual int size(void) const;
  
  //---------------------------------------------------------------------------
  //: Return the area of `this'
  //---------------------------------------------------------------------------
  virtual double area(void) const;

  //---------------------------------------------------------------------------
  //: Is `this' convex ?
  //---------------------------------------------------------------------------
  virtual bool is_convex(void) const;
 
  //---------------------------------------------------------------------------
  //: Is `i' a valid index for the list of vertices ?
  //---------------------------------------------------------------------------
  virtual bool valid_index(const int i) const;

  //***************************************************************************
  // Implementation
  //***************************************************************************
protected:
  //---------------------------------------------------------------------------
  //: Default constructor. Do nothing. Just to enable inherance.
  //---------------------------------------------------------------------------
  vsol_polygon_2d(void);

  //---------------------------------------------------------------------------
  // Description: List of vertices
  //---------------------------------------------------------------------------
  vcl_vector<vsol_point_2d_ref> *storage_;
};

#endif // #ifndef VSOL_POLYGON_2D_H
