#ifndef VCSL_SPHERICAL_H
#define VCSL_SPHERICAL_H
//*****************************************************************************
//
// .NAME vcsl_spherical - 3D coordinate system specified by distance rho,
//                          angle theta and phi.
// .LIBRARY vcsl
// .HEADER  vxl Package
// .INCLUDE vcsl/vcsl_spherical.h
// .FILE    vcsl/vcsl_spherical.cxx
//
// .SECTION Author
// Fran�ois BERTEL
//
// .SECTION Modifications
// 2000/06/28 Fran�ois BERTEL Creation. Adapted from IUE
//*****************************************************************************

#include <vcsl/vcsl_spherical_sptr.h>

//*****************************************************************************
// External declarations for values
//*****************************************************************************
#include <vcsl/vcsl_spatial.h>

class vcsl_spherical
  : public vcsl_spatial
{
public:
  //***************************************************************************
  // Constructors/Destructor
  //***************************************************************************

  //: Default constructor.
  explicit vcsl_spherical(void);

  //: Destructor
  virtual ~vcsl_spherical();

  //***************************************************************************
  // Because VXL does not use dynamic_cast<> :-(
  //***************************************************************************

  virtual const vcsl_spherical *cast_to_spherical(void) const;
};

#endif // #ifndef VCSL_SPHERICAL_H
