#ifndef VSOL_REGION_3D_H
#define VSOL_REGION_3D_H
//*****************************************************************************
//:
//  \file
// \brief  Region of a 3D space
//
// \author
// Fran�ois BERTEL
//
// \verbatim
// Modifications
// 2000/06/17 Peter Vanroose  Implemented all operator==()s and type info
// 2000/05/04 Fran�ois BERTEL Creation
// \endverbatim
//*****************************************************************************

class vsol_region_3d;

//*****************************************************************************
// External declarations for values
//*****************************************************************************
#include <vsol/vsol_region_3d_sptr.h>
#include <vsol/vsol_surface_3d.h>

class vsol_region_3d
  :public vsol_surface_3d
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
public:
  //---------------------------------------------------------------------------
  //: Destructor
  //---------------------------------------------------------------------------
  virtual ~vsol_region_3d();

  //***************************************************************************
  // Status report
  //***************************************************************************

  //---------------------------------------------------------------------------
  //: Return the area of `this'
  //---------------------------------------------------------------------------
  virtual double area(void) const=0;
};

#endif // #ifndef VSOL_REGION_3D_H
