#ifdef __GNUC__
#pragma implementation
#endif

//:
// \file
// \brief  Class to perform linear discriminant analysis
// \author Tim Cootes
// Converted to VXL by Gavin Wheeler

#include <vcl_cassert.h>
#include <vcl_cstddef.h> // for size_t
#include <mbl/mbl_lda.h>
#include <mbl/mbl_matxvec.h>
#include <mbl/mbl_matrix_products.h>
#include <vsl/vsl_indent.h>
#include <vnl/algo/vnl_svd.h>
#include <vnl/algo/vnl_symmetric_eigensystem.h>

#include <vnl/io/vnl_io_vector.h>
#include <vsl/vsl_vector_io.h>

//=======================================================================

mbl_lda::mbl_lda()
{
}

//=======================================================================

mbl_lda::~mbl_lda()
{
}

//=======================================================================

static void ZeroMatrix(vnl_matrix<double>& M)
{
  int nr=M.rows();
  int nc=M.columns();
  double** m = M.data_array();
  for (int i=0;i<nr;++i)
    for (int j=0;j<nc;++j)
      m[i][j]=0.0;
}

//=======================================================================

void mbl_lda::updateCovar(vnl_matrix<double>& S, const vnl_vector<double>& V)
{
  int n = V.size();
  if (S.rows()!=n)
  {
    S.resize(n,n);
    ZeroMatrix(S);
  }

  double** s = S.data_array();
  const double* v = V.data_block();
  for (int i=0;i<n;++i)
  {
    double *row = s[i];
    double vi = v[i];
    for (int j=0;j<n;++j)
      row[j] += vi*v[j];
  }
}

//=======================================================================
//: Perform LDA on data
// \param label  Array [0..n-1] of integers indices
// \param v  Set of vectors [0..n-1]
// label[i] gives class of v[i]
// Classes must be labelled from 0..m-1
void mbl_lda::build(const vnl_vector<double>* v, const int * label, int n,
                    const vnl_matrix<double>& wS, bool compute_wS)
{
  // Find range of class indices and count #valid
  int lo_i=n;
  int hi_i=-1;
  int n_valid = 0;
  for (int i=0;i<n;++i)
  {
    if (label[i]>=0)
    {
      if (label[i]<lo_i) lo_i=label[i];
      if (label[i]>hi_i) hi_i=label[i];
      n_valid++;
    }
  }

  assert(lo_i==0);

  // Compute mean of each class
  int n_classes = hi_i+1;
  mean_.resize(n_classes);
  n_samples_.resize(n_classes);
  for (int i=0;i<n_classes;++i)
    n_samples_[i]=0;

  for (int i=0;i<n;++i)
  {
    int l = label[i];
    if (l<0) continue;
    if (mean_[l].size()==0)
    {
      mean_[l] = v[i];
      n_samples_[l] = 1;
    }
    else
    {
      mean_[l] += v[i];
      n_samples_[l] += 1;
    }
  }

  int n_used_classes = 0;
  for (int i=0;i<n_classes;++i)
  {
    if (n_samples_[i]>0)
    {
      mean_[i]/=n_samples_[i];
      if (i==0) mean_class_mean_ = mean_[i];
      else      mean_class_mean_ += mean_[i];
      n_used_classes++;
    }
  }

  mean_class_mean_/=n_used_classes;

  // Build between class covariance
  // Zero to start:
  betweenS_.resize(0,0);

  for (int i=0;i<n_classes;++i)
  {
    if (n_samples_[i]>0)
      updateCovar(betweenS_,mean_[i] - mean_class_mean_);
  }

  betweenS_/=n_used_classes;

  if (compute_wS)
  {
    withinS_.resize(0,0);
    // Count number of samples used to build matrix
    int n_used=0;
    for (int i=0;i<n;++i)
    {
      int l=label[i];
      if (l>=0 && n_samples_[l]>1)
      {
        updateCovar(withinS_,v[i]-mean_[l]);
        n_used++;
      }
    }
    withinS_/=n_used;
  }
  else
    withinS_ = wS;

  vnl_matrix<double> wS_inv;
  //  NR_Inverse(wS_inv,withinS_);
  vnl_svd<double> wS_svd(withinS_); // vnl_svd<double> wS_svd(wS) is wrong
  wS_inv = wS_svd.inverse();

  vnl_matrix<double> A = betweenS_ * wS_inv;

  // Compute eigenvectors and eigenvalues (descending order)
  vnl_matrix<double> EVecs(A.rows(), A.columns());
  vnl_vector<double> evals(A.columns());
  //  NR_CalcSymEigens(A,EVecs,evals,false);
  vnl_symmetric_eigensystem_compute(A, EVecs, evals);

  //make the eigenvector matrix (columns) and eigenvalue vector in descending order.
  vnl_vector<double> temp_vect(EVecs.rows());
  double temp_val;
  int n_cols = EVecs.columns();

  for (int i=0; i<n_cols/2; ++i)
  {
    // swap t_EVects columes
    temp_vect=EVecs.get_column(i);
    EVecs.set_column(i, EVecs.get_column(n_cols-1-i));
    EVecs.set_column((n_cols-1-i), temp_vect);
    // swap t_evals elem
    temp_val=evals.get(i);
    evals.put(i, evals.get(n_cols-1-i));
    evals.put((n_cols-1-i), temp_val);
  }

  vcl_cout << "Should be largest eigenvalue first\n";
  vcl_cout << evals<< vcl_endl;

  // Record n_classes-1 vector basis
  int m = EVecs.rows();

  int t = n_used_classes-1;
  if (t>m) t=m;
  // Copy first t to basis_
  basis_.resize(m,t);

  double **E = EVecs.data_array();
  double **b = basis_.data_array();
  vcl_size_t bytes_per_row = t * sizeof(double);
  for (int i=0;i<m;++i)
  {
    memcpy(b[i]+1,E[i]+1,bytes_per_row);
  }

  evals_.resize(t);
  for (int i=0;i<t;++i)
    evals_[i] = evals[i];

  // Compute projection of mean into d space
  d_m_mean_.resize(t);
  mbl_matxvec_prod_vm(mean_class_mean_,basis_,d_m_mean_);

  // Project each mean into d-space
  d_mean_.resize(n_classes);
  for (int i=0;i<n_classes;++i)
    if (n_samples_[i]>0)
      x_to_d(d_mean_[i],mean_[i]);
}

//=======================================================================
//: Perform LDA on data
void mbl_lda::build(const vnl_vector<double>* v, const int* label, int n)
{
  build(v,label,n,vnl_matrix<double>(),true);
}

//=======================================================================
//: Perform LDA on data
void mbl_lda::build(const vnl_vector<double>* v, const vcl_vector<int>& label)
{
  build(v,&label.front(),label.size(),vnl_matrix<double>(),true);
}

//=======================================================================
//: Perform LDA on data
void mbl_lda::build(const vnl_vector<double>* v, const vcl_vector<int>& label,
                    const vnl_matrix<double>& wS)
{
  build(v,&label.front(),label.size(),wS,false);
}

//=======================================================================
//: Perform LDA on data
void mbl_lda::build(const vcl_vector<vnl_vector<double> >& v, const vcl_vector<int>& label)
{
  assert(v.size()==label.size());
  build(&v.front(),&label.front(),label.size(),vnl_matrix<double>(),true);
}

//=======================================================================
//: Perform LDA on data
void mbl_lda::build(const vcl_vector<vnl_vector<double> >& v, const vcl_vector<int>& label,
                    const vnl_matrix<double>& wS)
{
  assert(v.size()==label.size());
  build(&v.front(),&label.front(),label.size(),wS,false);
}

//=======================================================================
//: Perform LDA on data
//  Columns of M form example vectors
//  i'th column belongs to class label[i]
//  Note: label([1..n]) not label([0..n-1])
void mbl_lda::build(const vnl_matrix<double>& M, const vcl_vector<int>& label)
{
  int n_egs = M.columns();
  assert(n_egs==label.size());
  //  assert(label.lo()==1);
  vcl_vector<vnl_vector<double> > v(n_egs);
  for (int i=0;i<n_egs;++i)
  {
    v[i] = M.get_column(i);
  }
  build(&v.front(),&label.front()+1,n_egs,vnl_matrix<double>(),true);
}

//=======================================================================
//: Perform LDA on data
//  Columns of M form example vectors
//  i'th column belongs to class label[i]
//  Note: label([1..n]) not label([0..n-1])
void mbl_lda::build(const vnl_matrix<double>& M, const vcl_vector<int>& label,
                    const vnl_matrix<double>& wS)
{
  int n_egs = M.columns();
  assert(n_egs==label.size());
  //  assert(label.lo()==1);
  vcl_vector<vnl_vector<double> > v(n_egs);
  for (int i=0;i<n_egs;++i)
  {
    v[i] = M.get_column(i);
  }
  build(&v.front(),&label.front()+1,n_egs,wS,false);
}


//=======================================================================
//: Project x into discriminant space
void mbl_lda::x_to_d(vnl_vector<double>& d, const vnl_vector<double>& x) const
{
  d.resize(d_m_mean_.size());
  mbl_matxvec_prod_vm(x,basis_,d);
  d-=d_m_mean_;
}

//=======================================================================
//: Project d fron discriminant space into original space
void mbl_lda::d_to_x(vnl_vector<double>& x, const vnl_vector<double>& d) const
{
  mbl_matxvec_prod_mv(basis_,d,x);
  x+=mean_class_mean_;
}

//=======================================================================

short mbl_lda::version_no() const
{
  return 1;
}

//=======================================================================

vcl_string mbl_lda::is_a() const
{
  return vcl_string("NR_LDA");
}

//=======================================================================

void mbl_lda::print_summary(vcl_ostream& os) const
{
  // os << data_; // example of data output
  vcl_cerr << "mbl_lda::print_summary() NYI" << vcl_endl; vcl_abort();
}

//=======================================================================

void mbl_lda::b_write(vsl_b_ostream& bfs) const
{
  vsl_b_write(bfs,version_no());
  vsl_b_write(bfs,mean_);
  vsl_b_write(bfs,d_mean_);
  vsl_b_write(bfs,mean_class_mean_);
  vsl_b_write(bfs,n_samples_);
  vsl_b_write(bfs,withinS_);
  vsl_b_write(bfs,betweenS_);
  vsl_b_write(bfs,basis_);
  vsl_b_write(bfs,evals_);
  vsl_b_write(bfs,d_m_mean_);
}

//=======================================================================

void mbl_lda::b_read(vsl_b_istream& bfs)
{
  if (!bfs) return;

  short version;
  vsl_b_read(bfs,version);
  switch (version)
  {
    case (1):
      vsl_b_read(bfs,mean_);
      vsl_b_read(bfs,d_mean_);
      vsl_b_read(bfs,mean_class_mean_);
      vsl_b_read(bfs,n_samples_);
      vsl_b_read(bfs,withinS_);
      vsl_b_read(bfs,betweenS_);
      vsl_b_read(bfs,basis_);
      vsl_b_read(bfs,evals_);
      vsl_b_read(bfs,d_m_mean_);
      break;
    default:
      //CHECK FUNCTION SIGNATURE IS CORRECT
      vcl_cerr << "I/O ERROR: vsl_b_read(vsl_b_istream&, mbl_lda &) \n";
      vcl_cerr << "           Unknown version number "<< version << vcl_endl;
      bfs.is().clear(vcl_ios::badbit); // Set an unrecoverable IO error on stream
      return;
  }
}

//=======================================================================

void vsl_b_write(vsl_b_ostream& bfs, const mbl_lda& b)
{
  b.b_write(bfs);
}

//=======================================================================

void vsl_b_read(vsl_b_istream& bfs, mbl_lda& b)
{
  b.b_read(bfs);
}

//=======================================================================

vcl_ostream& operator<<(vcl_ostream& os,const mbl_lda& b)
{
  os << b.is_a() << ": ";
  vsl_indent_inc(os);
  b.print_summary(os);
  vsl_indent_dec(os);
  return os;
}
