#include "bvpl_local_max_functor.h"
//:
// \file
// \brief Template specializations

template <>
void bvpl_local_max_functor<bvxm_opinion>::init()
{
  max_=bvxm_opinion(1.0,0.0);
}

template <>
void bvpl_local_max_functor<bsta_num_obs<bsta_gauss_f1> >::init()
{
  max_= bsta_gauss_f1(0.0f, 1.0f);
}

template <>
bool bvpl_local_max_functor<bsta_num_obs<bsta_gauss_f1> >::greater_than(bsta_num_obs<bsta_gauss_f1>& g1, bsta_num_obs<bsta_gauss_f1>& g2)
{
  return vcl_abs(g1.mean()) > vcl_abs(g2.mean());
}

template <>
bsta_num_obs<bsta_gauss_f1> bvpl_local_max_functor<bsta_num_obs<bsta_gauss_f1> >::min_response()
{
  return bsta_gauss_f1(0.0f, 1.0f);
}

template <>
bvxm_opinion bvpl_local_max_functor<bvxm_opinion >::min_response()
{
  return bvxm_opinion(1.0f, 0.0f);
}

template <>
float bvpl_local_max_functor<bsta_num_obs<bsta_gauss_f1> >::filter_response(unsigned id, unsigned target_id, bsta_num_obs<bsta_gauss_f1> curr_val)
{
  if (id !=target_id)
    return 0.0f;

  return vcl_abs(curr_val.mean());
}
