#ifdef __GNUC__
#pragma implementation
#endif

#include <vcl_cassert.h>
#include <vcl_string.h>
#include <vcl_cstdlib.h> // vcl_abort()

#include <vsl/vsl_indent.h>
#include <mbl/mbl_data_wrapper.h>
#include <vpdfl/vpdfl_axis_gaussian.h>
#include <vpdfl/vpdfl_axis_gaussian_builder.h>

//=======================================================================
// Dflt ctor
//=======================================================================

vpdfl_axis_gaussian_builder::vpdfl_axis_gaussian_builder()
    : min_var_(1.0e-6)
{
}

//=======================================================================
// Destructor
//=======================================================================

vpdfl_axis_gaussian_builder::~vpdfl_axis_gaussian_builder()
{
}

//=======================================================================

vpdfl_axis_gaussian& vpdfl_axis_gaussian_builder::gaussian(vpdfl_pdf_base& model) const
{
  // require a vpdfl_axis_gaussian
  assert(model.is_a("vpdfl_axis_gaussian"));
  return (vpdfl_axis_gaussian&) model;
}

vpdfl_pdf_base* vpdfl_axis_gaussian_builder::new_model() const
{
  return new vpdfl_axis_gaussian;
}

//=======================================================================
//: Define lower threshold on variance for built models
//=======================================================================
void vpdfl_axis_gaussian_builder::set_min_var(double min_var)
{
  min_var_ = min_var;
}

//=======================================================================
//: Get lower threshold on variance for built models
//=======================================================================
double vpdfl_axis_gaussian_builder::min_var()
{
  return min_var_;
}

void vpdfl_axis_gaussian_builder::build(vpdfl_pdf_base& model,
                                        const vnl_vector<double>& mean)
{
  vpdfl_axis_gaussian& g = gaussian(model);

  vnl_vector<double> var(mean.size());
  for (int i=0;i<mean.size();i++) var(i)=min_var_;

  g.set(mean,var);
}

void vpdfl_axis_gaussian_builder::build(vpdfl_pdf_base& model,
                                        mbl_data_wrapper<vnl_vector<double> >& data)
{
  vpdfl_axis_gaussian& g = gaussian(model);

  int n_samples = data.size();

  if (n_samples<2)
  {
    vcl_cerr<<"vpdfl_axis_gaussian_builder::build() Too few examples available."<<vcl_endl;
    vcl_abort();
  }

  int n_dims = data.current().size();
  vnl_vector<double> sum(n_dims),sum_sq(n_dims),var(n_dims);

  double* var_data = var.data_block();
  double* sum_data = sum.data_block();
  double* sum_sq_data = sum_sq.data_block();
  for (int j=0;j<n_dims;j++)
  {
    sum_data[j]=0.0;
    sum_sq_data[j]=0.0;
  }

  data.reset();
  for (int i=0;i<n_samples;i++)
  {
    const double *v = data.current().data_block();
    for (int j=0;j<n_dims;j++)
    {
      sum_data[j] += v[j];
      sum_sq_data[j] += v[j]*v[j];
    }

    data.next();
  }

  sum/=n_samples;
  for (int j=0;j<n_dims;j++)
  {
    var_data[j] = sum_sq_data[j]/n_samples - sum_data[j]*sum_data[j];
    if (var_data[j]<min_var_) var_data[j]=min_var_;
  }

  g.set(sum,var);
}

void vpdfl_axis_gaussian_builder::weighted_build(vpdfl_pdf_base& model,
                                                 mbl_data_wrapper<vnl_vector<double> >& data,
                                                 const vcl_vector<double>& wts)
{
  vpdfl_axis_gaussian& g = gaussian(model);

  int n_samples = data.size();

  if (n_samples<2)
  {
    vcl_cerr<<"vpdfl_axis_gaussian_builder::build() Too few examples available."<<vcl_endl;
    vcl_abort();
  }

  if (wts.size()!=n_samples)
  {
    vcl_cerr<<"vpdfl_axis_gaussian_builder::build() Weight array must contain "
            <<n_samples<<" not "<<wts.size()<<vcl_endl;
    vcl_abort();
  }

  int n_dims = data.current().size();

  data.reset();
  vnl_vector<double> sum = wts[0] * data.current();
  double w_sum = wts[0];

  for (int i=1;i<n_samples;i++)
  {
    data.next();
    sum += wts[i] * data.current();
    w_sum += wts[i];
  }

  vnl_vector<double> mean = sum/w_sum;


  vnl_vector<double> sum_sq(n_dims),var(n_dims);
  double* var_data = var.data_block();
  double* m_data = mean.data_block();
  double* sum_sq_data = sum_sq.data_block();
  for (int j=0;j<n_dims;j++)
  {
    sum_sq_data[j]=0.0;
  }

  data.reset();
  for (int i=0;i<n_samples;i++)
  {
    const double *v = data.current().data_block();
    double w = wts[i];
    for (int j=0;j<n_dims;j++)
    {
      double dx = v[j]-m_data[j];
      sum_sq_data[j] += w * dx*dx;
    }

    data.next();
  }

  for (int j=0;j<n_dims;j++)
  {
    var_data[j] = sum_sq_data[j]/w_sum;
    if (var_data[j]<min_var_) var_data[j]=min_var_;
  }

  g.set(mean,var);
}
//=======================================================================
// Method: is_a
//=======================================================================

vcl_string  vpdfl_axis_gaussian_builder::is_a() const
{
  return vcl_string("vpdfl_axis_gaussian_builder");
}

bool vpdfl_axis_gaussian_builder::is_a(vcl_string const& s) const
{
  return vpdfl_builder_base::is_a(s) || s==vcl_string("vpdfl_axis_gaussian_builder");
}

//=======================================================================
// Method: version_no
//=======================================================================

short vpdfl_axis_gaussian_builder::version_no() const
{
  return 1;
}

//=======================================================================
// Method: clone
//=======================================================================

vpdfl_builder_base* vpdfl_axis_gaussian_builder::clone() const
{
  return new vpdfl_axis_gaussian_builder(*this);
}

//=======================================================================
// Method: print
//=======================================================================

void vpdfl_axis_gaussian_builder::print_summary(vcl_ostream& os) const
{
  os << "Min. var. : "<< min_var_;
}

//=======================================================================
// Method: save
//=======================================================================

void vpdfl_axis_gaussian_builder::b_write(vsl_b_ostream& bfs) const
{
  vsl_b_write(bfs,version_no());
  vsl_b_write(bfs,min_var_);
}

//=======================================================================
// Method: load
//=======================================================================

void vpdfl_axis_gaussian_builder::b_read(vsl_b_istream& bfs)
{
  short version;
  vsl_b_read(bfs,version);
  switch (version)
  {
    case (1):
      vsl_b_read(bfs,min_var_);
      break;
    default:
      vcl_cerr << "vpdfl_axis_gaussian_builder::b_read() ";
      vcl_cerr << "Unexpected version number " << version << vcl_endl;
      vcl_abort();
  }
}

