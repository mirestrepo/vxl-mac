// This is vxl/vcsl/vcsl_cylindrical.cxx
#include "vcsl_cylindrical.h"

#include <vcsl/vcsl_axis.h>
#include <vcsl/vcsl_angle.h>

//***************************************************************************
// Constructors/Destructor
//***************************************************************************

//---------------------------------------------------------------------------
// Default constructor.
//---------------------------------------------------------------------------
vcsl_cylindrical::vcsl_cylindrical(void)
{
  vcsl_axis_sptr a;
  a=new vcsl_axis;
  axes_.push_back(a);
  a=new vcsl_axis(*(vcsl_angle::instance().ptr()));
  axes_.push_back(a);
  a=new vcsl_axis;
  axes_.push_back(a);
}

//---------------------------------------------------------------------------
// Destructor
//---------------------------------------------------------------------------
vcsl_cylindrical::~vcsl_cylindrical()
{
}

//***************************************************************************
// Because VXL does not use dynamic_cast<> :-(
//***************************************************************************

const vcsl_cylindrical *vcsl_cylindrical::cast_to_cylindrical(void) const
{
  return this;
}
