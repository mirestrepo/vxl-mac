#include <vcsl/vcsl_cylindrical_to_cartesian_3d.h>
#include <vcl_cmath.h> // for sqrt(), cos(), sin()

#include <vcsl/vcsl_spatial.h>

//***************************************************************************
// Constructors/Destructor
//***************************************************************************

//---------------------------------------------------------------------------
// Destructor
//---------------------------------------------------------------------------
vcsl_cylindrical_to_cartesian_3d::~vcsl_cylindrical_to_cartesian_3d()
{
}

//***************************************************************************
// Status report
//***************************************************************************

//---------------------------------------------------------------------------
// Is `this' invertible at time `time'?
// REQUIRE: valid_time(time)
//---------------------------------------------------------------------------
bool vcsl_cylindrical_to_cartesian_3d::is_invertible(const double time) const
{
  // require
  assert(valid_time(time));

  return true;
}

//---------------------------------------------------------------------------
// Is `this' correctly set ?
//---------------------------------------------------------------------------
bool vcsl_cylindrical_to_cartesian_3d::is_valid(void) const
{
  return true;
}

//***************************************************************************
// Basic operations
//***************************************************************************

//---------------------------------------------------------------------------
// Image of `v' by `this'
// REQUIRE: is_valid()
// REQUIRE: v.size()==3
//---------------------------------------------------------------------------
vnl_vector<double> *
vcsl_cylindrical_to_cartesian_3d::execute(const vnl_vector<double> &v,
                                          const double time) const
{
  // require
  assert(is_valid());
  assert(v.size()==3);

  vnl_vector<double> *result;

  double rho;
  double theta;
  double z;

  double x;
  double y;

  result=new vnl_vector<double>(3);

  rho=v.get(0);
  theta=v.get(1);
  z=v.get(2);

  x=rho*vcl_cos(theta);
  y=rho*vcl_sin(theta);
  
  result->put(0,x);
  result->put(1,y);
  result->put(2,z);

  return result;
}

//---------------------------------------------------------------------------
// Image of `v' by the inverse of `this'
// REQUIRE: is_valid()
// REQUIRE: is_invertible(time)
// REQUIRE: v.size()==3
//---------------------------------------------------------------------------
vnl_vector<double> *
vcsl_cylindrical_to_cartesian_3d::inverse(const vnl_vector<double> &v,
                                          const double time) const
{
  // require
  assert(is_valid());
  assert(is_invertible(time));
  assert(v.size()==3);

  vnl_vector<double> *result;
  double x;
  double y;
  double z;
  double rho;
  double theta;

  result=new vnl_vector<double>(3);

  x=v.get(0);
  y=v.get(1);
  z=v.get(2);

  rho=vcl_sqrt(x*x+y*y);
  theta=vcl_atan2(y,x);

  result->put(0,rho);
  result->put(1,theta);
  result->put(2,z);

  return result;
}

//***************************************************************************
// Singleton pattern
//***************************************************************************

//: Return the reference to the unique vcsl_length object
vcsl_cylindrical_to_cartesian_3d_sptr
vcsl_cylindrical_to_cartesian_3d::instance(void)
{
  if(instance_.ptr()==0)
    instance_=new vcsl_cylindrical_to_cartesian_3d;
  return instance_;
}

//---------------------------------------------------------------------------
// Default constructor
//---------------------------------------------------------------------------
vcsl_cylindrical_to_cartesian_3d::vcsl_cylindrical_to_cartesian_3d(void)
{
}

//---------------------------------------------------------------------------
// Reference to the unique vcsl_cylindrical_to_cartesian_3d object
//---------------------------------------------------------------------------
vcsl_cylindrical_to_cartesian_3d_sptr
vcsl_cylindrical_to_cartesian_3d::instance_=0;
