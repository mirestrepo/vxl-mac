#ifndef VCSL_CARTESIAN_2D_H
#define VCSL_CARTESIAN_2D_H
//*****************************************************************************
//
// .NAME vcsl_cartesian_2d - 2D Cartesian coordinate system
// .LIBRARY vcsl
// .HEADER  vxl Package
// .INCLUDE vcsl/vcsl_cartesian_2d.h
// .FILE    vcsl/vcsl_cartesian_2d.cxx
//
// .SECTION Author
// Fran�ois BERTEL
//
// .SECTION Modifications
// 2000/06/28 Fran�ois BERTEL Creation. Adapted from IUE
//*****************************************************************************

#include <vcsl/vcsl_cartesian_2d_sptr.h>

//*****************************************************************************
// External declarations for values
//*****************************************************************************
#include <vcsl/vcsl_spatial.h>

class vcsl_cartesian_2d
  : public vcsl_spatial
{
public:
  //***************************************************************************
  // Constructors/Destructor
  //***************************************************************************

  //: Default constructor.
  explicit vcsl_cartesian_2d(void);

  //: Destructor
  virtual ~vcsl_cartesian_2d();

  //***************************************************************************
  // Because VXL does not use dynamic_cast<> :-(
  //***************************************************************************
  
  virtual const vcsl_cartesian_2d *cast_to_cartesian_2d(void) const;

  //***************************************************************************
  // Status report
  //***************************************************************************
  
  //: Are the axes of `this' right handed ?
  virtual bool is_right_handed(void) const;

  //***************************************************************************
  // Status setting
  //***************************************************************************

  //: Set whether the coordinate system is right handed or not
  virtual void set_right_handed(const bool new_right_handed);

protected:
  //***************************************************************************
  // Implementation
  //***************************************************************************

  //:  True if the axes of `this' are right handed
  bool _right_handed;
};

#endif // #ifndef VCSL_CARTESIAN_2D_H
