// This is brl/bbas/bsta/bsta_joint_histogram.h
#ifndef bsta_joint_histogram_h_
#define bsta_joint_histogram_h_
//:
// \file
// \brief A simple joint_histogram class
// \author Joseph L. Mundy
// \date 5/19/04
//
// A templated joint_histogram class.  Supports entropy calculations
// 
//
// \verbatim
//  Modifications
// \endverbatim
#include <vbl/vbl_array_2d.h>
#include <vcl_iostream.h>
#include <vbl/vbl_ref_count.h>
#include <bsta/bsta_joint_histogram_base.h>
template <class T> class bsta_joint_histogram : public bsta_joint_histogram_base
{
 public:
  bsta_joint_histogram(const T range = 360, const unsigned int nbins = 8,
                       const T min_prob = 0.0);//0.005
 ~bsta_joint_histogram() {}

  unsigned int nbins() const { return nbins_; }

  T range() const {return range_;}

  T min_prob() const {return min_prob_;}

  vbl_array_2d<T> counts() const {return counts_;}

  void upcount(T a, T mag_a,
               T b, T mag_b);
  void parzen(const T sigma);

  T p(unsigned int a, unsigned int b) const;
  T volume() const;
  T entropy() const;
  T renyi_entropy() const;
  void print(vcl_ostream& os = vcl_cout) const;

  void set_count(unsigned r, unsigned c, T cnt)
    { if(r<static_cast<unsigned>(counts_.rows())&&
	c<static_cast<unsigned>(counts_.cols())) counts_[r][c]=cnt;}

 private:
  void compute_volume() const; // mutable const
  mutable bool volume_valid_;
  mutable T volume_;
  unsigned int nbins_;
  T range_;
  T delta_;
  T min_prob_;
  vbl_array_2d<T> counts_;
};

#define BSTA_JOINT_HISTOGRAM_INSTANTIATE(T) extern "Please #include <bsta/bsta_joint_histogram.txx>"

#endif // bsta_joint_histogram_h_
