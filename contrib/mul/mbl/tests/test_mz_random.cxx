#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_utility.h>
#include <vcl_cmath.h> // for vcl_sqrt()

#include <vnl/vnl_test.h>
#include <mbl/mbl_mz_random.h>

void test_mz_random()
{
  vcl_cout << "***********************" << vcl_endl;
  vcl_cout << " Testing mbl_mz_random " << vcl_endl;
  vcl_cout << "***********************" << vcl_endl;

  mbl_mz_random mz_random;
  mz_random.reseed(123456);


  TEST("lrand32",mz_random.lrand32()==3501493769,true);
  TEST("lrand32(0,10)",mz_random.lrand32(0,10)==9,true);
  double d1 = mz_random.drand32(0,1) - 0.615854;
  TEST("drand32(0,1)",d1*d1<0.000001,true);
  double d2 = mz_random.drand64(0,1) - 0.225741;
  TEST("drand64(0,1)",d2*d2<0.000001,true);

  double sum = 0.0;
  double sum_sq = 0.0;
  int n = 1000;
  for (int i=0;i<n;++i)
  {
  	double r = mz_random.normal();
	sum += r;
	sum_sq += r*r;
  }

  double mean = sum/n;
  double var  = vcl_sqrt(sum_sq/n-mean*mean);
  TEST("normal() mean near zero",mean*mean<1e5,true);
  double dv = var-1.0;
  TEST("normal() var near one",dv*dv<1e4,true);
  vcl_cout<<"Mean: "<<mean<<" var: "<<var<<vcl_endl;

  sum = 0.0;
  sum_sq = 0.0;
  for (int i=0;i<n;++i)
  {
  	double r = mz_random.normal64();
	sum += r;
	sum_sq += r*r;
  }

  mean = sum/n;
  var  = vcl_sqrt(sum_sq/n-mean*mean);
  TEST("normal64() mean near zero",mean*mean<1e5,true);
  dv = var-1.0;
  TEST("normal64() var near one",dv*dv<1e4,true);
  vcl_cout<<"Mean: "<<mean<<" var: "<<var<<vcl_endl;

}

TESTMAIN(test_mz_random);
