#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_utility.h>
#include <vcl_cmath.h>

#include <mbl/mbl_sum_1d.h>

void test_sum_1d()
{
  vcl_cout << "******************************" << vcl_endl;
  vcl_cout << " Testing mbl_sum_1d " << vcl_endl;
  vcl_cout << "*****************************" << vcl_endl;

  mbl_sum_1d stats,stats2;

  TEST("Empty N.obs",stats.nObs()==0,true);
  TEST("Empty mean",stats.mean()==0.0,true);

  for (int i=0;i<5;++i)
  {
    stats.obs(i);
    stats2.obs(i+5);
  }

  TEST("nObs()",stats.nObs()==5,true);
  TEST("mean()",vcl_fabs(stats.mean()-2)<1e-6,true);

  vcl_cout<<stats<<vcl_endl;

  mbl_sum_1d stats3 = stats;
  TEST("Equality operator",stats==stats3,true);

  mbl_sum_1d stats4 = stats+stats2;
  TEST("Addition",stats4.nObs()==10 && vcl_fabs(stats4.mean()-4.5)<1e-6,true);
}

TESTLIB_DEFINE_MAIN(test_sum_1d);
