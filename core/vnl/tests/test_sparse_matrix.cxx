#include <vcl_cstdlib.h>
#include <vcl_ctime.h>
#include <vcl_iostream.h>
#include <vcl_iomanip.h>
#include <vcl_cmath.h>

#include <vnl/vnl_sparse_matrix.h>
#include <vnl/vnl_test.h>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matops.h>
#include <vnl/algo/vnl_sparse_symmetric_eigensystem.h>
#include <vnl/algo/vnl_symmetric_eigensystem.h>

// Test the sparse matrix operations.
int doTest1()
{
  const unsigned int n = 20;

  vnl_sparse_matrix<double> m1(n,n);
  for (unsigned i=0; i<n; i++) {
    m1(i,i) = 2.0;
    m1(i,(i+3)%n) = 1.0;
  }

  vcl_cout << "m1:\n";
  for (unsigned i=0; i<n; i++) {
    for (unsigned j=0; j<n; j++)
      vcl_cout << m1(i,j) << " ";
    vcl_cout << vcl_endl;
  }
  
  vnl_sparse_matrix<double> m2(n,n);
  for (unsigned i=0; i<n; i++) {
    m2(i,i) = 2.0;
    m2(i,(i+n-3)%n) = 1.0;
  }

  vcl_cout << "m2:\n";
  for (unsigned i=0; i<n; i++) {
    for (unsigned j=0; j<n; j++)
      vcl_cout << m2(i,j) << " ";
    vcl_cout << vcl_endl;
  }

  vnl_sparse_matrix<double> prod;
  m1.mult(m2,prod);

  vcl_cout << "prod:\n";
  for (unsigned i=0; i<n; i++) {
    for (unsigned j=0; j<n; j++)
      vcl_cout << prod(i,j) << " ";
    vcl_cout << vcl_endl;
  }

  vnl_sparse_matrix<double> sum;
  m1.add(m2,sum);

  vcl_cout << "sum:\n";
  for (unsigned i=0; i<n; i++) {
    for (unsigned j=0; j<n; j++)
      vcl_cout << sum(i,j) << " ";
    vcl_cout << vcl_endl;
  }
  
  vnl_sparse_matrix<double> diff;
  m1.subtract(m2,diff);

  vcl_cout << "diff:\n";
  for (unsigned i=0; i<n; i++) {
    for (unsigned j=0; j<n; j++)
      vcl_cout << diff(i,j) << " ";
    vcl_cout << vcl_endl;
  }
  
  return 0;
}

int doTest2()
{
  clock_t t = clock();
  for (unsigned int n = 1000; n<4000; n+=1000) {
    vnl_sparse_matrix<double> m1(n,n);
    for (unsigned i=0; i<n; i++) {
      m1(i,i) = 2.0;
      m1(i,(i+3)%n) = 1.0;
    }
    
    vnl_sparse_matrix<double> m2(n,n);
    for (unsigned i=0; i<n; i++) {
      m2(i,i) = 2.0;
      m2(i,(i+n-3)%n) = 1.0;
    }
    
    vnl_sparse_matrix<double> prod;
    m1.mult(m2,prod);
    
    clock_t tn = clock();
    vcl_cout << n << " " << tn - t << vcl_endl;
    t = tn;
  }
  
  return 0;
}

int doTest3()
{
  const unsigned int n = 20;

  vnl_sparse_matrix<double> ms(n,n);
  vnl_matrix<double> md(n,n); md = 0.0; // Initialise to all zeros
  // The matrix must be symmetric
  for (unsigned i=0; i<n; i++) {
    ms(i,i)         = md(i,i)         = i+1.0;
    ms(i,(i+3)%n)   = md(i,(i+3)%n)   = 1.0;
    ms(i,(i+n-3)%n) = md(i,(i+n-3)%n) = 1.0;
    // ms(i,i) = md(i,i) = 1.0*(i+1)*(i+1);
  }

  vcl_cout << "ms:" << vcl_endl;
  for (unsigned i=0; i<n; i++) {
    for (unsigned j=0; j<n; j++)
      vcl_cout << ms(i,j) << " ";
    vcl_cout << vcl_endl;
  }
  vcl_cout << "md:" << vcl_endl << md << vcl_endl;
  
  const unsigned int nvals = 2;
  vnl_symmetric_eigensystem<double> ed(md);
  vnl_sparse_symmetric_eigensystem es;
  if (0 == es.CalculateNPairs(ms,nvals,true,20))
    vcl_cout << "vnl_sparse_symmetric_eigensystem::CalculateNPairs() succeeded\n";
  else
    vcl_cout << "vnl_sparse_symmetric_eigensystem::CalculateNPairs() failed\n";
  
  // Report 'em.
  for (unsigned i=0; i<nvals; i++) {
    vcl_cout << "Dense[" << i << "] : " << ed.D(i,i) << " -> " 
	 << ed.get_eigenvector(i) << vcl_endl;
    vcl_cout << "Sparse[" << i << "]: " << es.get_eigenvalue(i) << " -> " 
	 << es.get_eigenvector(i) << vcl_endl;
  }

  return 0;
}

int doTest4()
{
  const unsigned int n = 20;

  vnl_sparse_matrix<double> ms(n,n);
  vnl_matrix<double> md(n,n); md = 0.0; // Initialise to all zeros
  // The matrix must be symmetric
  for (unsigned i=0; i<n; i++) {
    ms(i,i)         = md(i,i)         = i+1.0;
    ms(i,(i+3)%n)   = md(i,(i+3)%n)   = 1.0;
    ms(i,(i+n-3)%n) = md(i,(i+n-3)%n) = 1.0;
    // ms(i,i) = md(i,i) = 1.0*(i+1)*(i+1);
  }

  const unsigned int nvals = 3;
  vnl_symmetric_eigensystem<double> ed(md);
  vnl_sparse_symmetric_eigensystem es;
  if (0 == es.CalculateNPairs(ms,nvals))
    vcl_cout << "vnl_sparse_symmetric_eigensystem::CalculateNPairs() succeeded\n";
  else
    vcl_cout << "vnl_sparse_symmetric_eigensystem::CalculateNPairs() failed\n";

  // Report 'em.
  for (unsigned i=0; i<nvals; i++) {
    double dense = ed.D(i,i);
    double sparse = es.get_eigenvalue(i);
    vcl_cout << "Dense[" << i << "] : " << dense << vcl_endl;
    vcl_cout << "Sparse[" << i << "]: " << sparse << vcl_endl;
    double err = fabs(dense - sparse);
    vcl_cout << "Error: " << err << vcl_endl;
    Assert("vnl_sparse_symmetric_eigensystem eigenvalue error", err < 1e-10);
  }
  return 0;
}

extern "C"
int test_sparse_matrix()
{
  vcl_cout << "Starting test 1\n";
  int r = doTest1();
  vcl_cout << "Starting test 2\n";
  r = r + doTest2();
  vcl_cout << "Starting test 3\n";
  r = r + doTest3();
  vcl_cout << "Starting test 4\n";
  r = r + doTest4();
  return r;
}

TESTMAIN(test_sparse_matrix);
