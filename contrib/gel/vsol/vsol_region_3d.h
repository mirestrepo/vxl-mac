#ifndef vsol_region_3d_h_
#define vsol_region_3d_h_
//*****************************************************************************
//:
// \file
// \brief  Region of a 3D space
//
// \author
// Fran�ois BERTEL
//
// \verbatim
// Modifications
// 2000/05/04 Fran�ois BERTEL Creation
// 2000/06/17 Peter Vanroose  Implemented all operator==()s and type info
// 2003/01/08 Peter Vanroose  Added pure virtual is_convex()
// \endverbatim
//*****************************************************************************

//*****************************************************************************
// External declarations for values
//*****************************************************************************
#include <vsol/vsol_surface_3d.h>
class vsol_polygon_3d;

class vsol_region_3d : public vsol_surface_3d
{
 public:
  enum vsol_region_3d_type
  { REGION_NO_TYPE=0,
    POLYGON,
    NUM_REGION_TYPES
  };

  //---------------------------------------------------------------------------
  //: Return the spatial type
  //---------------------------------------------------------------------------
  vsol_spatial_object_3d_type spatial_type(void) const { return vsol_spatial_object_3d::REGION; }

  //---------------------------------------------------------------------------
  //: Return the region type
  //---------------------------------------------------------------------------
  virtual vsol_region_3d_type region_type(void) const { return vsol_region_3d::REGION_NO_TYPE; }

  //***************************************************************************
  // Initialization
  //***************************************************************************

  //---------------------------------------------------------------------------
  // Destructor
  //---------------------------------------------------------------------------
  virtual ~vsol_region_3d();

  //***************************************************************************
  // virtuals of vsol_spatial_object_3d
  //***************************************************************************

   virtual vsol_region_3d* cast_to_region(void) { return this; }
   virtual const vsol_region_3d * cast_to_region(void) const { return this; }

   virtual vsol_polygon_3d *cast_to_polygon(void) {return 0;}
   virtual const vsol_polygon_3d *cast_to_polygon(void) const {return 0;}

  //***************************************************************************
  // Status report
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Return the area of `this'
  //---------------------------------------------------------------------------
  virtual double area(void) const=0;

  //---------------------------------------------------------------------------
  //: Return true if this region is convex
  //---------------------------------------------------------------------------
  virtual bool is_convex(void) const=0;
};

#endif // #ifndef vsol_region_3d_h_
