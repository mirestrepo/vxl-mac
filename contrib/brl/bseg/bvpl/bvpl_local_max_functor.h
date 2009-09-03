// This is brl/bseg/bvpl/bvpl_local_max_functor.h
#ifndef bvpl_local_max_functor_h_
#define bvpl_local_max_functor_h_
//:
// \file
// \brief Functor to find the 2D edges with computing expected values
//
// \author Vishal Jain (vj@lems.brown.edu)
// \date June 29, 2009
//
// \verbatim
//  Modifications
//   <none yet>
// \endverbatim

#include "bvpl_kernel_iterator.h"
#include <bvxm/grid/bvxm_opinion.h>
#include <bsta/bsta_gauss_f1.h>
#include <bsta/bsta_attributes.h>

template <class T>
class bvpl_local_max_functor
{
 public:
  //: Default constructor
  bvpl_local_max_functor();

  //: Destructor
  ~bvpl_local_max_functor() {}

   //: Apply a given operation to value val, depending on the dispatch character
  void apply(T& val, bvpl_kernel_dispatch& d);

  //: Returns the final operation of this functor
  T result(T cur_val);

  // Some additional functionalities

  bool greater_than(T& val1, T& val2);

  static T min_response();

  float filter_response(unsigned id, unsigned target_id, T curr_val);

 private:
  //: max_value over neighborhood
  T max_;
  //: cur_value over neighborhood
  T cur_val_;
  //: Initializes all local class variables
  void init();
};


//Template specializations. They are forward declared here and implemented in the .cxx file to avoid redefinitions
template <>
void bvpl_local_max_functor<bvxm_opinion>::init();
template <>
void bvpl_local_max_functor<bsta_num_obs<bsta_gauss_f1> >::init();
//template <>
//void bvpl_local_max_functor<bsta_num_obs<bsta_gauss_f1> >::apply(bsta_num_obs<bsta_gauss_f1>& val, bvpl_kernel_dispatch& d);
//template <>
//bvxm_opinion bvpl_local_max_functor<bvxm_opinion>::result(bvxm_opinion cur_val);
//template <>
//bsta_num_obs<bsta_gauss_f1> bvpl_local_max_functor<bsta_num_obs<bsta_gauss_f1> >::result(bsta_num_obs<bsta_gauss_f1> cur_val);
template <>
bool bvpl_local_max_functor<bsta_num_obs<bsta_gauss_f1> >::greater_than(bsta_num_obs<bsta_gauss_f1>& g1, bsta_num_obs<bsta_gauss_f1>& g2);
template <>
bvxm_opinion bvpl_local_max_functor<bvxm_opinion >::min_response();
template <>
bsta_num_obs<bsta_gauss_f1> bvpl_local_max_functor<bsta_num_obs<bsta_gauss_f1> >::min_response();
template <>
float bvpl_local_max_functor<bsta_num_obs<bsta_gauss_f1> >::filter_response(unsigned id, unsigned target_id, bsta_num_obs<bsta_gauss_f1> curr_val);


//: Default constructor
template <class T>
bvpl_local_max_functor<T>::bvpl_local_max_functor()
{
  this->init();
}

//: Init class variables
template <class T>
void bvpl_local_max_functor<T>::init()
{
  max_=T(0);
}


//: Apply functor
template <class T>
void bvpl_local_max_functor<T>::apply(T& val, bvpl_kernel_dispatch& d)
{
  if (greater_than(val,max_))
    max_=val;
}

//: Retrieve result
template <class T>
T bvpl_local_max_functor<T>::result( T cur_val)
{
  T result = greater_than(cur_val, max_)?cur_val:min_response();

  //reset all variables
  init();

  return result;
}

//: Comparison function
template <class T>
bool bvpl_local_max_functor<T>::greater_than(T& val1, T& val2)
{
  return val1 > val2;
}

//: Min value
template <class T>
T bvpl_local_max_functor<T>::min_response()
{
  return T(0);
}

//Filter used by non-max suppression
template <class T>
float bvpl_local_max_functor<T>::filter_response(unsigned id, unsigned target_id, T curr_val)
{
  if (id !=target_id)
    return 0.0f;

  return (float)curr_val;
}

#endif // bvpl_local_max_functor_h_
