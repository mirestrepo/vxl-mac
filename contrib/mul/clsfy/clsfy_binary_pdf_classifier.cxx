
//	Copyright: (C) 2000 British Telecommunications PLC



#include <clsfy/clsfy_binary_pdf_classifier.h>
#include <vcl_string.h>
#include <vcl_cmath.h>
#include <vcl_iostream.h>
#include <vcl_vector.h>
#include <vnl/io/vnl_io_vector.h>
#include <vsl/vsl_binary_loader.h>
#include <mbl/mbl_matxvec.h>
#include <mbl/mbl_matrix_products.h>
#include <vcl_cassert.h>


//=======================================================================

void clsfy_binary_pdf_classifier::deleteStuff()
{
  delete pdf_;
  pdf_ = 0;
}

//=======================================================================

//: Classify the input vector
// Returns either class1 (Inside PDF mode) or class 0 (Outside PDF mode).
unsigned clsfy_binary_pdf_classifier::classify(const vnl_vector<double> &input) const
{
  assert(pdf_);
  
  if (pdf_->log_p(input) >= log_prob_limit_)
    return 1;
  else
    return 0;
}

//=======================================================================

//: Return the probability the input being in each class p(class|data).
// output(i) i<n_classes, contains the probability that the input
// is in class i;
// 
void clsfy_binary_pdf_classifier::class_probabilities(
  vcl_vector<double> &outputs,
  const vnl_vector<double> &input)  const
{
		//: likelyhood = p(input|InClass) / prob_limit_
  double likelyhood= vcl_exp(log_l(input));
  outputs.resize(1);
  outputs[0] = likelyhood / (1 + likelyhood);
}

//=======================================================================

//: Log likelyhood  of being in class 0.vcl_log(p(data|class=0))
// This function is intended for use in binary classifiers only. It is
// related to the class 0 probability as follows,  
// P(class=0|data) = vcl_exp(logL) / (1+vcl_exp(logL).
// Don't forget that p(data|X) is a density and so the result can be
// greater than 1.0 or less than 0.0, (or indeed between 0.0 and 1.0).
double clsfy_binary_pdf_classifier::log_l(const vnl_vector<double> &input) const
{
  assert(pdf_);
  
		//: likelyhood = p(input|InClass) / prob_limit_
  return pdf_->log_p(input) - log_prob_limit_;
}

//=======================================================================

vcl_string clsfy_binary_pdf_classifier::is_a() const
{
  return vcl_string("clsfy_binary_pdf_classifier");
}

//=======================================================================

short clsfy_binary_pdf_classifier::version_no() const 
{ 
  return 1; 
}

//=======================================================================

// required if data is present in this class
void clsfy_binary_pdf_classifier::print_summary(vcl_ostream& os) const
{
  os << "log Probability limit, " << log_prob_limit_ << " ,PDF " << pdf_;
}

//=======================================================================

clsfy_classifier_base* clsfy_binary_pdf_classifier::clone() const
{
  return new clsfy_binary_pdf_classifier(*this);
}

//=======================================================================

clsfy_binary_pdf_classifier& clsfy_binary_pdf_classifier::operator=(const clsfy_binary_pdf_classifier& classifier)
{
  if (&classifier==this) return *this;
  
  clsfy_classifier_base::operator=(classifier);
  
  deleteStuff();
  
  if (classifier.pdf_)
    pdf_ = classifier.pdf_->clone();
  
  log_prob_limit_ = classifier.log_prob_limit_;
  
  return *this;
}


//=======================================================================

void clsfy_binary_pdf_classifier::b_write(vsl_b_ostream& bfs) const
{
  vsl_b_write(bfs,version_no());
  vsl_b_write(bfs,log_prob_limit_);
  vsl_b_write(bfs,pdf_);
}

//=======================================================================

void clsfy_binary_pdf_classifier::b_read(vsl_b_istream& bfs)
{
  if (!bfs) return;
  
  short version;
  vsl_b_read(bfs,version);
  switch (version)
  {
		case (1):
      vsl_b_read(bfs,log_prob_limit_);
      vsl_b_read(bfs,pdf_);
      break;
    default:
      vcl_cerr << "I/O ERROR: clsfy_binary_pdf_classifier::b_read(vsl_b_istream&) \n";
      vcl_cerr << "           Unknown version number "<< version << "\n";
      bfs.is().clear(vcl_ios::badbit); // Set an unrecoverable IO error on stream
      return;
  }
}

