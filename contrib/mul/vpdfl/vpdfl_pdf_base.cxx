#ifdef __GNUC__
#pragma implementation
#endif

//:
// \file
// \author Tim Cootes
// \date 12-Apr-2001
// \brief Base class for Multi-Variate Probability Density Function classes.

#include <vcl_cstdlib.h> // vcl_abort()
#include <vcl_cmath.h>
#include <vpdfl/vpdfl_pdf_base.h>
#include <vsl/vsl_indent.h>
#include <vsl/vsl_binary_loader.h>
#include <vcl_queue.h>
#include <vcl_ctime.h>
#include <vpdfl/vpdfl_sampler_base.h>

//=======================================================================
// Dflt ctor
//=======================================================================

vpdfl_pdf_base::vpdfl_pdf_base()
{
}

//=======================================================================
// Destructor
//=======================================================================

vpdfl_pdf_base::~vpdfl_pdf_base()
{
}

//=======================================================================
//: Probability density at x
//=======================================================================
double vpdfl_pdf_base::operator()(const vnl_vector<double>& x) const
{
  return vcl_exp(log_p(x));
}


double vpdfl_pdf_base::log_prob_thresh(double pass_proportion) const
{
  assert(pass_proportion >= 0.0);
  assert(pass_proportion < 1.0);

  // The number of samples on the less likely side of the boundary.
  // Increse the number for greater reliabililty
  const unsigned n_stat = 20;

  double /* above, */ below, lP;
  unsigned int nSamples, i;
  vnl_vector<double> x;

  vpdfl_sampler_base *sampler = new_sampler();
  if (pass_proportion > 0.5)
  {
    vcl_priority_queue<double, vcl_vector<double>, vcl_less<double> > pq;
    //We want at n_stat samples outside the cut-off.
    nSamples = (unsigned)(((double)n_stat / (1.0 - pass_proportion)) + 0.5);

    for (i = 0; i < n_stat+1; i++)
    {
      sampler->sample(x);
      pq.push(log_p(x));
    }

    for (; i < nSamples; i++)
    {
      sampler->sample(x);
      lP = log_p(x);
      // pq.top() should be the greatest value in the queue
      if (lP < pq.top())
      {
        pq.pop();
        pq.push(lP);
      }
    }
    // get two values either side of boundary;
#if 0
    above = pq.top();
#endif
    pq.pop();
    below = pq.top();
  }
  else
  {
    vcl_priority_queue<double, vcl_vector<double>, vcl_greater<double> > pq;
    //We want at n_stat samples inside the cut-off.
    nSamples = (unsigned)(((double)n_stat / pass_proportion) + 0.5);

    for (i = 0; i < n_stat+1; i++)
    {
      sampler->sample(x);
      pq.push(log_p(x));
    }

    for (; i < nSamples; i++)
    {
      sampler->sample(x);
      lP = log_p(x);
      if (lP > pq.top())  // pq.top() should be the smallest value in the queue.
      {
         pq.pop();
         pq.push(lP);
      }
    }
    // get two values either side of boundary;
#if 0
    above = pq.top();
#endif
    pq.pop();
    below = pq.top();
  }

  delete sampler;

  // Find geometric mean of probability densities to get boundary (arithmetic mean of logProbs.)
#if 0
  return (above + below)/2.0;
#else
  return below;
#endif
}

//=======================================================================

bool vpdfl_pdf_base::is_valid_pdf() const
{
  return mean_.size() == var_.size() && mean_.size() > 0;
}


//=======================================================================

short vpdfl_pdf_base::version_no() const
{
  return 1;
}

//=======================================================================
// Method: vxl_add_to_binary_loader
//=======================================================================

void vsl_add_to_binary_loader(const vpdfl_pdf_base& b)
{
  vsl_binary_loader<vpdfl_pdf_base>::instance().add(b);
}

//=======================================================================
// Method: is_a
//=======================================================================

vcl_string vpdfl_pdf_base::is_a() const
{
  static vcl_string class_name_ = "vpdfl_pdf_base";
  return class_name_;
}

//=======================================================================
// Method: is_class
//=======================================================================

bool vpdfl_pdf_base::is_class(vcl_string const& s) const
{
  static vcl_string class_name_ = "vpdfl_pdf_base";
  return s==class_name_;
}

//=======================================================================
// Method: print
//=======================================================================

  // required if data is present in this base class
void vpdfl_pdf_base::print_summary(vcl_ostream& os) const
{
  os << vsl_indent() << "N. Dims : "<< mean_.size();
}

//=======================================================================
// Method: save
//=======================================================================

  // required if data is present in this base class
void vpdfl_pdf_base::b_write(vsl_b_ostream& bfs) const
{
  vsl_b_write(bfs, version_no());
  vsl_b_write(bfs, mean_);
  vsl_b_write(bfs, var_);
}

//=======================================================================
// Method: load
//=======================================================================

  // required if data is present in this base class
void vpdfl_pdf_base::b_read(vsl_b_istream& bfs)
{
  short version;
  vsl_b_read(bfs,version);
  switch (version)
  {
    case (1):
      vsl_b_read(bfs,mean_);
      vsl_b_read(bfs,var_);
      break;
    default:
      vcl_cerr << "vpdfl_pdf_base::b_read() ";
      vcl_cerr << "Unexpected version number " << version << vcl_endl;
      vcl_abort();
  }
}

//=======================================================================
// Associated function: operator<<
//=======================================================================

void vsl_b_write(vsl_b_ostream& bfs, const vpdfl_pdf_base* b)
{
  if (b)
  {
    vsl_b_write(bfs, b->is_a());
    b->b_write(bfs);
  }
  else
    vsl_b_write(bfs, vcl_string("VSL_NULL_PTR"));
}

//=======================================================================
// Associated function: operator<<
//=======================================================================

void vsl_b_write(vsl_b_ostream& bfs, const vpdfl_pdf_base& b)
{
  b.b_write(bfs);
}

//=======================================================================
// Associated function: operator>>
//=======================================================================

void vsl_b_read(vsl_b_istream& bfs, vpdfl_pdf_base& b)
{
  b.b_read(bfs);
}


void vsl_print_summary(vcl_ostream& os,const vpdfl_pdf_base& b)
{
  os << b.is_a() << ": ";
  vsl_inc_indent(os);
  b.print_summary(os);
  vsl_dec_indent(os);
}


void vsl_print_summary(vcl_ostream& os,const vpdfl_pdf_base* b)
{
  if (b)
    vsl_print_summary(os, *b);
  else
    os << "No vpdfl_pdf_base defined.";
}

  //: Stream output operator for class reference
vcl_ostream& operator<<(vcl_ostream& os,const vpdfl_pdf_base& b)
{
  vsl_print_summary(os,b);
  return os;
}

  //: Stream output operator for class pointer
vcl_ostream& operator<<(vcl_ostream& os,const vpdfl_pdf_base* b)
{
  vsl_print_summary(os,b);
  return os;
}

