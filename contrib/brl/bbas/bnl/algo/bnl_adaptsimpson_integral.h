#ifndef BNL_ADAPTSIMPSON_INTEGRAL_H_
#define BNL_ADAPTSIMPSON_INTEGRAL_H_

// : 
// \file
// \author Kongbin Kang at Brown
// \date Jan. 17th, 2005
// 
#include <bnl/bnl_definite_integral.h>

class bnl_adaptsimpson_integral : public bnl_definite_integral {

  private:
    //: used to wrap the function class to a ordinary function.
    static double int_fnct_(float* x);


  public:

    bnl_adaptsimpson_integral() {}

    //: a nd b are integral limits respectively. 
    // n is the number of intervals used in intergral. 
   double integral(bnl_integrant_fnct *f, float a, float b, float accuracy);
};

#endif
