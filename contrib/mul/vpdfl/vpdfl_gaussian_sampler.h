#ifndef vpdfl_gaussian_sampler_h
#define vpdfl_gaussian_sampler_h
#ifdef __GNUC__
#pragma interface
#endif

//:
// \file
// \author Ian Scott
// \date 19-Apr-2001
// \brief Sampler class for Multi-Variate Gaussian.

#include <vsl/vsl_binary_io.h>
#include <vcl_string.h>
#include <mbl/mbl_mz_random.h>
#include <vnl/vnl_fwd.h>
#include <vpdfl/vpdfl_sampler_base.h>


class vpdfl_gaussian;
//=======================================================================

//: Samples from a Gaussian PDF
class vpdfl_gaussian_sampler :public vpdfl_sampler_base {
protected:
    //: The random number generator
  mbl_mz_random rng_;
    //: workspace variable
  vnl_vector<double> b_;
public:

    //: Dflt ctor
  vpdfl_gaussian_sampler();

    //: Destructor
  virtual ~vpdfl_gaussian_sampler();

    //: Draw random sample from Gaussian distribution
  virtual void sample(vnl_vector<double>& x);

    //: Reseeds the internal random number generator
    // To achieve quasi-random initialisation use;
    // \verbatim
    // #include <vcl_ctime.h>
    // ..
    // sampler.reseed(vcl_time(0));
    // \endverbatim
  virtual void reseed(unsigned long);

  //========= methods which do not change state (const) ==========//

    //: Return a reference to the pdf model
    // This is properly cast.
  const vpdfl_gaussian& gaussian() const;

    //: Name of the class
  virtual vcl_string is_a() const;
    //: Name of the class
  virtual bool is_a(vcl_string const& s) const;

    //: Create a copy on the heap and return base class pointer
  virtual vpdfl_sampler_base* clone() const;
};

  //: Stream output operator for class reference
vcl_ostream& operator<<(vcl_ostream& os,const vpdfl_gaussian_sampler& b);

  //: Stream output operator for class pointer
vcl_ostream& operator<<(vcl_ostream& os,const vpdfl_gaussian_sampler* b);

#endif // vpdfl_pdf_sampler_base_h
