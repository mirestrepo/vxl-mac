// This is core/vcsl/vcsl_length.cxx
#include "vcsl_length.h"

#include <vcsl/vcsl_meter.h>

//***************************************************************************
// Constructors/Destructor
//***************************************************************************

//---------------------------------------------------------------------------
// Destructor
//---------------------------------------------------------------------------
vcsl_length::~vcsl_length()
{
}

//***************************************************************************
// Status report
//***************************************************************************

//---------------------------------------------------------------------------
// Is `new_unit' a compatible unit for the dimension ?
//---------------------------------------------------------------------------
bool vcsl_length::compatible_unit(const vcsl_unit &new_unit) const
{
  return new_unit.cast_to_length_unit()!=0;
}

//---------------------------------------------------------------------------
// Return the standard unit associated to the dimension
//---------------------------------------------------------------------------
vcsl_unit_sptr vcsl_length::standard_unit(void) const
{
  return vcsl_meter::instance().ptr();
}

//***************************************************************************
// Singleton pattern
//***************************************************************************

//---------------------------------------------------------------------------
// Return the reference to the unique vcsl_length object
//---------------------------------------------------------------------------
vcsl_length_sptr vcsl_length::instance(void)
{
  if (instance_.ptr()==0)
    instance_=new vcsl_length;
  return instance_;
}

//---------------------------------------------------------------------------
// Default constructor
//---------------------------------------------------------------------------
vcsl_length::vcsl_length(void)
{
}

//---------------------------------------------------------------------------
// Reference to the unique vcsl_length object
//---------------------------------------------------------------------------
vcsl_length_sptr vcsl_length::instance_=0;
