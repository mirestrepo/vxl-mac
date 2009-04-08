#ifndef boxm_sample_h_
#define boxm_sample_h_

#include "boxm_apm_traits.h"
#include <vcl_iostream.h>
#include <vsl/vsl_binary_io.h>
#include <bsta/bsta_attributes.h>

template <boxm_apm_type APM_MODEL>
class boxm_sample
{
public:
  static const boxm_apm_type apm_type = APM_MODEL;
  typedef typename boxm_apm_traits<APM_MODEL>::apm_datatype apm_datatype;
  typedef typename boxm_apm_traits<APM_MODEL>::obs_datatype obs_datatype;
  typedef typename boxm_apm_traits<APM_MODEL>::obs_mathtype obs_mathtype;

  //: default constructor
  boxm_sample() : alpha(0.001f), appearance() {}
  //: constructor
  boxm_sample(float alpha_val, apm_datatype apm_val): alpha(alpha_val), appearance(apm_val) {}

  short version_no() const{ return 1; }

  //: the occlusion density at the sample point
  float alpha;

  //: the appearance model at the sample point
  typename boxm_apm_traits<APM_MODEL>::apm_datatype appearance;

  void  print(vcl_ostream& os) const;

};

template <boxm_apm_type APM_MODEL>
void vsl_b_write(vsl_b_ostream & os, boxm_sample<APM_MODEL> const &sample);

template <boxm_apm_type APM_MODEL>
void vsl_b_write(vsl_b_ostream & os, boxm_sample<APM_MODEL> const * &sample);

template <boxm_apm_type APM_MODEL>
void vsl_b_read(vsl_b_istream & is, boxm_sample<APM_MODEL> &sample);

template <boxm_apm_type APM_MODEL>
void vsl_b_read(vsl_b_istream & is, boxm_sample<APM_MODEL> *&sample);

template <boxm_apm_type APM_MODEL>
vcl_ostream& operator << (vcl_ostream& os, const boxm_sample<APM_MODEL>& sample);

#endif

