#include <vcsl/vcsl_cartesian_3d.h>

#include <vcsl/vcsl_axis.h>

//***************************************************************************
// Constructors/Destructor
//***************************************************************************

//---------------------------------------------------------------------------
// Default constructor
//---------------------------------------------------------------------------
vcsl_cartesian_3d::vcsl_cartesian_3d(void)
{
  vcsl_axis_sptr a;
  a=new vcsl_axis;
  axes_.push_back(a);
  a=new vcsl_axis(*(a.ptr()));
  axes_.push_back(a);
  a=new vcsl_axis(*(a.ptr()));
  axes_.push_back(a);
  _right_handed=true;
}

//---------------------------------------------------------------------------
// Destructor
//---------------------------------------------------------------------------
vcsl_cartesian_3d::~vcsl_cartesian_3d()
{
}

//***************************************************************************
// Because VXL does not use dynamic_cast<> :-(
//***************************************************************************

const vcsl_cartesian_3d *vcsl_cartesian_3d::cast_to_cartesian_3d(void) const
{
  return this;
}

//***************************************************************************
// Status report
//***************************************************************************

//---------------------------------------------------------------------------
// Are the axes of `this' right handed ?
//---------------------------------------------------------------------------
bool vcsl_cartesian_3d::is_right_handed(void) const
{
  return _right_handed;
}

//***************************************************************************
// Status setting
//***************************************************************************

//---------------------------------------------------------------------------
// Set whether the coordinate system is right handed or not
//---------------------------------------------------------------------------
void vcsl_cartesian_3d::set_right_handed(const bool new_right_handed)
{
  _right_handed=new_right_handed;
}
