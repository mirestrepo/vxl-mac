#ifndef vsol_surface_3d_h
#define vsol_surface_3d_h
//*****************************************************************************
//
// .NAME vsol_surface_3d - Abstract surface in a 3D space
// .LIBRARY vsol
// .INCLUDE vsol/vsol_surface_3d.h
// .FILE    vsol/vsol_surface_3d.cxx
//
// .SECTION Author
// Fran�ois BERTEL
//
// .SECTION Modifications
// 2001/07/03 Peter Vanroose  Replaced vnl_double_3 by vgl_vector_3d
// 2000/05/04 Fran�ois BERTEL Creation
//*****************************************************************************

class vsol_surface_3d;

//*****************************************************************************
// External declarations for values
//*****************************************************************************
#include <vsol/vsol_surface_3d_sptr.h>
#include <vsol/vsol_spatial_object_3d.h>

#include <vsol/vsol_point_3d_sptr.h>
#include <vgl/vgl_vector_3d.h>

class vsol_surface_3d
  :public vsol_spatial_object_3d
{
  //***************************************************************************
  // Initialization
  //***************************************************************************
public:
  //---------------------------------------------------------------------------
  //: Destructor
  //---------------------------------------------------------------------------
  virtual ~vsol_surface_3d();

  //***************************************************************************
  // Basic operations
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Is `p' in `this' ?
  //---------------------------------------------------------------------------
  virtual bool in(const vsol_point_3d_sptr &p) const=0;

  //---------------------------------------------------------------------------
  //: Return the unit normal vector at point `p'. Have to be deleted manually
  //  REQUIRE: in(p)
  //---------------------------------------------------------------------------
  virtual vgl_vector_3d<double>
  normal_at_point(const vsol_point_3d_sptr &p) const=0;
};

#endif // vsol_surface_3d_h
