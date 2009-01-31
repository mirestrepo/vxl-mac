#include <testlib/testlib_test.h>
#include <bbgm/bbgm_image_of.h>
#include <bbgm/bbgm_image_sptr.h>
#include <bsta/bsta_attributes.h>
#include <bsta/bsta_mixture.h>
#include <bsta/bsta_mixture_fixed.h>
#include <bsta/bsta_gauss_if3.h>
#include <bsta/algo/bsta_adaptive_updater.h>

#include <bbgm/bbgm_update.h>
#include <bsta/bsta_gaussian_indep.h>
#include <vil/vil_image_view.h>
#include <vnl/vnl_random.h>
#include <bbgm/bbgm_loader.h>
#include <brdb/brdb_value.h>
#include <brdb/brdb_value_sptr.h>
#include <core/vil_pro/vil_load_image_view_process.h>
#include <bbgm/pro/bbgm_processes.h>

void init_random_image(vil_image_view<float>& img)
{
  vnl_random rand;
  for (unsigned int p=0; p<img.nplanes(); ++p)
    for (unsigned int j=0; j<img.nj(); ++j)
      for (unsigned int i=0; i<img.ni(); ++i)
        img(i,j,p) = static_cast<float>(rand.drand32());
}

MAIN( test_update_dist_image )
{
#if 0 //fix up later
  bbgm_loader::register_loaders();
  const unsigned int ni = 640, nj = 480;
  vil_image_view<float>* ip = new vil_image_view<float>(ni,nj,3);
  init_random_image(*ip);

  brdb_value_sptr image_view_value =
    new brdb_value_t<vil_image_view_base_sptr>(ip);

  bprb_process_sptr p1 = new bbgm_update_dist_image_process();
  p1->set_input(1, image_view_value);
  p1->set_input(2 , new brdb_value_t<int>(3));//number of mixture components
  p1->set_input(3 , new brdb_value_t<int>(300));//window size
  p1->set_input(4 , new brdb_value_t<float>(0.1f));//initial variance
  p1->set_input(5 , new brdb_value_t<float>(3.0f));//g_thresh
  p1->set_input(6 , new brdb_value_t<float>(0.02f));//min standard deviation
  bool good = p1->init();
  good = good && p1->execute();
  TEST("update distribution image process", good, true);
  SUMMARY();
#endif
}

