#ifndef bsta_histogram_txx_
#define bsta_histogram_txx_
//:
// \file
#include "bsta_histogram.h"

#include <vcl_cmath.h> // for log()
#include <vcl_iostream.h>
#include <vcl_cassert.h>
#include "bsta_gauss.h"
#include <vnl/vnl_math.h> // for log2e == 1/vcl_log(2.0)

template <class T>
bsta_histogram<T>::bsta_histogram()
  : area_valid_(false), area_(0), nbins_(1), range_(0),
    delta_(0),min_prob_(0), min_(0), max_(0)
{
}

template <class T>
bsta_histogram<T>::bsta_histogram(const T range, const unsigned int nbins,
                                  const T min_prob)
  : area_valid_(false), area_(0), nbins_(nbins), range_(range),
    delta_(0),min_prob_(min_prob), min_(0), max_(range)
{
  if (nbins>0)
  {
    delta_ = range_/nbins;
    counts_.resize(nbins, T(0));
  }
}

template <class T>
bsta_histogram<T>::bsta_histogram(const T min, const T max,
                                  const unsigned int nbins,
                                  const T min_prob)
  : area_valid_(false), area_(0), nbins_(nbins), delta_(0),
    min_prob_(min_prob), min_ (min), max_(max)
{
  if (nbins>0)
  {
    range_ = max-min;
    delta_ = range_/nbins;
    counts_.resize(nbins, T(0));
  }
  else
  {
    range_ = 0;
    delta_ = 0;
  }
}

template <class T>
bsta_histogram<T>::bsta_histogram(const T min, const T max,
                                  vcl_vector<T> const& data, const T min_prob)
  : area_valid_(false), area_(0), delta_(0), min_prob_(min_prob),
    min_ (min), max_(max), counts_(data)
{
  nbins_ = data.size();
  range_ = max-min;
  if (nbins_>0)
    delta_ = range_/nbins_;
  else
    delta_ = 0;
}

template <class T>
void bsta_histogram<T>::upcount(T x, T mag)
{
  if (x<min_||x>max_)
    return;
  for (unsigned int i = 0; i<nbins_; i++)
    if ((i+1)*delta_>=(x-min_))
    {
      counts_[i] += mag;
      break;
    }
  area_valid_ = false;
}

template <class T>
void bsta_histogram<T>::compute_area() const
{
  area_ =0;
  for (unsigned int i = 0; i<nbins_; i++)
    area_ += counts_[i];
  area_valid_ = true;
}

template <class T>
T bsta_histogram<T>::p(unsigned int bin) const
{
  if (bin>=nbins_)
    return 0;
  if (!area_valid_)
    compute_area();
  if (area_ == T(0))
    return 0;
  else
    return counts_[bin]/area_;
}

template <class T>
T bsta_histogram<T>::p(const T val) const
{
  if (val<min_||val>max_)
    return 0;
  if (!area_valid_)
    compute_area();
  if (area_ == T(0))
    return 0;
  else
    for (unsigned int i = 0; i<nbins_; i++)
      if ((i+1)*delta_>=(val-min_))
        return counts_[i]/area_;
return 0;
}

template <class T>
T bsta_histogram<T>::area() const
{
  if (!area_valid_)
    compute_area();
  return area_;
}

//: Mean of distribution
template <class T>
T bsta_histogram<T>::mean() const
{
  return mean(0, nbins_-1);
}

  //: Mean of distribution between bin indices
template <class T>
T bsta_histogram<T>::mean(const unsigned int lowbin, const unsigned int highbin) const
{
  assert(lowbin<=highbin);
  assert(highbin<nbins_);
  T sum = 0;
  T sumx = 0;
  for (unsigned i = lowbin; i<=highbin; ++i)
  {
    sum += counts_[i];
    sumx += (i*delta_ + min_)*counts_[i];
  }
  if (sum==0)
    return 0;
  T result = sumx/sum;
  return result;
}

  //: Variance of distribution
template <class T>
T bsta_histogram<T>::variance() const
{
  return variance(0, nbins_-1);
}

  //: Variance of distribution between bin indices
template <class T>
T bsta_histogram<T>::
variance(const unsigned int lowbin, const unsigned int highbin) const
{
  assert(lowbin<=highbin);
  assert(highbin<nbins_);
  T mean = this->mean(lowbin, highbin);
  mean -= min_;
  T sum = 0;
  T sumx2 = 0;
  for (unsigned i = lowbin; i<=highbin; ++i)
  {
    sum += counts_[i];
    sumx2 += (i*delta_-mean)*(i*delta_-mean)*counts_[i];
  }
  if (sum==0)
    return 0;
  T result = sumx2/sum;
  return result;
}

template <class T>
T bsta_histogram<T>::entropy() const
{
  T ent = 0;
  for (unsigned int i = 0; i<nbins_; ++i)
  {
    T pi = this->p(i);
    if (pi>min_prob_)
      ent -= pi*T(vcl_log(pi));
  }
  ent *= (T)vnl_math::log2e;
  return ent;
}

template <class T>
T bsta_histogram<T>::renyi_entropy() const
{
  T sum = 0, ent = 0;
  for (unsigned int i = 0; i<nbins_; ++i)
  {
    T pi = this->p(i);
    sum += pi*pi;
  }
  if (sum>min_prob_)
    ent = - T(vcl_log(sum))*(T)vnl_math::log2e;
  return ent;
}

template <class T>
void bsta_histogram<T>::parzen(const T sigma)
{
  if (sigma<=0)
    return;
  double sd = (double)sigma;
  vcl_vector<double> in(nbins_), out(nbins_);
  for (unsigned int i=0; i<nbins_; i++)
    in[i]=counts_[i];
  bsta_gauss::bsta_1d_gaussian(sd, in, out);
  for (unsigned int i=0; i<nbins_; i++)
    counts_[i]=(T)out[i];
}

//The first non-zero bin starting at index = 0
template <class T>
unsigned bsta_histogram<T>::low_bin()
{
  unsigned lowbin=0;
  for (; lowbin<nbins_&&counts_[lowbin]==0; ++lowbin) /*nothing*/;
  return lowbin;
}

//The first non-zero bin starting at index = nbins-1
template <class T>
unsigned bsta_histogram<T>::high_bin()
{
  unsigned highbin=nbins_-1;
  for (; highbin>0&&counts_[highbin]==0; --highbin) /*nothing*/;
  return highbin;
}

// Fraction of area less than value
template <class T>
T bsta_histogram<T>::fraction_below(const T value) const
{
 if (value<min_||value>max_)
    return 0;
  if (!area_valid_)
    compute_area();
  if (area_ == T(0))
    return 0;
  T sum = 0, limit=(value-min_);
  for (unsigned int i = 0; i<nbins_; i++)
    if ((i+1)*delta_<limit)
      sum+=counts_[i];
    else
      return sum/area_;
 return 0;
}

// Fraction of area greater than value
template <class T>
T bsta_histogram<T>::fraction_above(const T value) const
{
 if (value<min_||value>max_)
    return 0;
  if (!area_valid_)
    compute_area();
  if (area_ == T(0))
    return 0;
  T sum = 0, limit=(value-min_);
  for (unsigned int i = 0; i<nbins_; i++)
    if ((i+1)*delta_>limit)
      sum+=counts_[i];
  return sum/area_;
}

// Value for area fraction below value
template <class T>
T bsta_histogram<T>::value_with_area_below(const T area_fraction) const
{
  if (area_fraction>T(1))
    return 0;
  if (!area_valid_)
    compute_area();
  if (area_ == T(0))
    return 0;
  T sum = 0;
  for (unsigned int i=0; i<nbins_; i++)
  {
    sum += counts_[i];
    if (sum>=area_fraction*area_)
      return (i+1)*delta_+min_;
  }
  return 0;
}

// Value for area fraction above value
template <class T>
T  bsta_histogram<T>::value_with_area_above(const T area_fraction) const
{
  if (area_fraction>T(1))
    return 0;
  if (!area_valid_)
    compute_area();
  if (area_ == T(0))
    return 0;
  T sum = 0;
  for (unsigned int i=nbins_-1; i!=0; i--)
  {
    sum += counts_[i];
    if (sum>area_fraction*area_)
      return (i+1)*delta_+min_;
  }
  return 0;
}

template <class T>
void bsta_histogram<T>::print(vcl_ostream& os) const
{
  for (unsigned int i=0; i<nbins_; i++)
    if (p(i) > 0)
      os << "p[" << i << "]=" << p(i) << '\n';
}

//: print as a matlab plot command
template <class T>
void bsta_histogram<T>::print_to_m(vcl_ostream& os) const
{
  os << "x = [" << min_;
  for (unsigned int i=1; i<nbins_; i++)
    os << ", " << min_ + i*delta_;
  os << "];\n";
  os << "y = [" << p((unsigned int)0);
  for (unsigned int i=1; i<nbins_; i++)
    os << ", " << p(i);
  os << "];\n";
  os << "bar(x,y,'r')\n";
}

template <class T>
vcl_ostream& bsta_histogram<T>::write(vcl_ostream& s) const
{
  s << area_valid_ << ' '
    << area_ << ' '
    << nbins_ << ' '
    << range_ << ' '
    << delta_ << ' '
    << min_prob_ << ' '
    << min_ << ' '
    << max_ << ' ';
  for (unsigned i = 0; i < counts_.size() ; i++)
    s << counts_[i] << ' ';

  return  s << '\n';
}

template <class T>
vcl_istream& bsta_histogram<T>::read(vcl_istream& s)
{
  s >> area_valid_
    >> area_
    >> nbins_
    >> range_
    >> delta_
    >> min_prob_
    >> min_
    >> max_;
  counts_.resize(nbins_);
  for (unsigned i = 0; i < counts_.size() ; i++)
    s >> counts_[i] ;
  return  s;
}

//: Write to stream
template <class T>
vcl_ostream& operator<<(vcl_ostream& s, bsta_histogram<T> const& h)
{
  return h.write(s);
}

//: Read from stream
template <class T>
vcl_istream& operator>>(vcl_istream& is, bsta_histogram<T>& h)
{
  return h.read(is);
}


#undef BSTA_HISTOGRAM_INSTANTIATE
#define BSTA_HISTOGRAM_INSTANTIATE(T) \
template class bsta_histogram<T >;\
template vcl_istream& operator>>(vcl_istream&, bsta_histogram<T >&);\
template vcl_ostream& operator<<(vcl_ostream&, bsta_histogram<T > const&)

#endif // bsta_histogram_txx_
