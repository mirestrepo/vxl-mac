#ifndef vpdfl_pdf_base_h
#define vpdfl_pdf_base_h
#ifdef __GNUC__
#pragma interface
#endif

//:
// \file
// \author Tim Cootes
// \date 12-Apr-2001
// \brief Base class for Multi-Variate Probability Density Function classes.

#include <vsl/vsl_binary_io.h>
#include <vnl/io/vnl_io_vector.h>
#include <vcl_string.h>

//=======================================================================

class vpdfl_sampler_base;

//: Base class for Multi-Variate Probability Density Function classes.
// Functions are available to test the plausibility of a vector or
// set of parameters, to modify a set of parameters so it is plausible
// and to choose a threshold of plausibility.  Also, for cases where
// the distributions of parameters are multi-modal, the number and
// centres of each peak can be recorded.
// This is particularly useful for non-linear and mixture model
// representations of the parameter distributions.
class vpdfl_pdf_base {
private:
  vnl_vector<double> mean_;
  vnl_vector<double> var_;
protected:
  void set_mean(const vnl_vector<double>& m) { mean_ = m; }
  void set_variance(const vnl_vector<double>& v) { var_ = v; }
public:

    //: Dflt ctor
  vpdfl_pdf_base();

    //: Destructor
  virtual ~vpdfl_pdf_base();

    //: Mean of distribution
  const vnl_vector<double>& mean() const { return mean_; }

    //: Variance of each dimension
  const vnl_vector<double>& variance() const { return var_; }

    //: Number of dimensions
  int n_dims() const { return mean_.size(); }

    //: Number of peaks of distribution
  virtual int n_peaks() const { return 1; }

    //: Position of the i'th peak
  virtual const vnl_vector<double>& peak(int) const { return mean_; }

    //: Log of probability density at x
  virtual double log_p(const vnl_vector<double>& x) const =0;

    //: Probability density at x
  virtual double operator()(const vnl_vector<double>& x) const;

    //: Gradient and value of PDF at x
    //  Computes gradient of PDF at x, and returns the prob at x in p
  virtual void gradient(vnl_vector<double>& g,
                        const vnl_vector<double>& x, double& p) const =0;

    //: Create a sampler object on the heap
    // Caller is responsible for deletion.
  virtual vpdfl_sampler_base* new_sampler()const=0 ;

    //: Compute threshold for PDF to pass a given proportion
  virtual double log_prob_thresh(double pass_proportion)const;

    //: Compute nearest point to x which has a density above a threshold
    //  If log_p(x)>log_p_min then x unchanged.  Otherwise x is moved
    //  (typically up the gradient) until log_p(x)>=log_p_min.
    // \param x This may be modified to the nearest plausible position.
  virtual void nearest_plausible(vnl_vector<double>& x, double log_p_min)const =0;

    //: Return true if the object represents a valid PDF.
    // This will return false, if n_dims() is 0, for example just ofter
    // default construction.
  virtual bool is_valid_pdf() const;



    //: Version number for I/O
  short version_no() const;

    //: Name of the class
  virtual vcl_string is_a() const = 0;
    //: Name of the class
  virtual bool is_a(vcl_string const& s) const;

    //: Create a copy on the heap and return base class pointer
  virtual vpdfl_pdf_base* clone() const = 0;

    //: Print class to os
  virtual void print_summary(vcl_ostream& os) const = 0;

    //: Save class to binary file stream
    //!in: bfs: Target binary file stream
  virtual void b_write(vsl_b_ostream& bfs) const = 0;

  //========== methods which change state (non-const) ============//

    //: Load class from binary file stream
    //!out: bfs: Target binary file stream
  virtual void b_read(vsl_b_istream& bfs) = 0;

protected:

private:

    //: To record name of class, returned by is_a() method
  static vcl_string class_name_;
};

  //: Allows derived class to be loaded by base-class pointer
  //  A loader object exists which is invoked by calls
  //  of the form "bfs>>base_ptr;".  This loads derived class
  //  objects from the disk, places them on the heap and
  //  returns a base class pointer.
  //  In order to work the loader object requires
  //  an instance of each derived class that might be
  //  found.  This function gives the model class to
  //  the appropriate loader.
void vsl_add_to_binary_loader(const vpdfl_pdf_base& b);

  //: Binary file stream output operator for class reference
void vsl_b_write(vsl_b_ostream& bfs, const vpdfl_pdf_base& b);

  //: Binary file stream output operator for pointer to class
void vsl_b_write(vsl_b_ostream& bfs, const vpdfl_pdf_base* b);

  //: Binary file stream input operator for class reference
void vsl_b_read(vsl_b_istream& bfs, vpdfl_pdf_base& b);

  //: Stream output operator for class reference
void vsl_print_summary(vcl_ostream& os,const vpdfl_pdf_base& b);

  //: Stream output operator for class pointer
void vsl_print_summary(vcl_ostream& os,const vpdfl_pdf_base* b);

  //: Stream output operator for class reference
vcl_ostream& operator<<(vcl_ostream& os,const vpdfl_pdf_base& b);

  //: Stream output operator for class pointer
vcl_ostream& operator<<(vcl_ostream& os,const vpdfl_pdf_base* b);

#endif // vpdfl_pdf_base_h
