/*
  fsm@robots.ox.ac.uk
*/

#include <vcl/vcl_iostream.h>
#include <vnl/vnl_math.h>
#include <vnl/vnl_complex.h>

int main(int,char**) {
  vnl_double_complex z(1,2);
  
  cout << "z=" << z << endl;
  cout << "abs(z)=" << vnl_math_abs(z) << endl;
  
  return 0;
}
