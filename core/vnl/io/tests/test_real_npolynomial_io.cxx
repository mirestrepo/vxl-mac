#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_utility.h>

#include <vnl/vnl_test.h>
#include <vnl/vnl_math.h>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_real_npolynomial.h>

#include <vnl/io/vnl_io_real_npolynomial.h>


void test_real_npolynomial_io()
{
  vcl_cout << "*************************" << vcl_endl;
  vcl_cout << "test_real_npolynomial_io" << vcl_endl;
  vcl_cout << "*************************" << vcl_endl;
  //// test constructors, accessors
  vnl_vector<double> coeffs(4),coeffs2;
  vnl_matrix<int> exponents(4,2);

  coeffs(0) = 0.1;
  coeffs(1) = 0.2;
  coeffs(2) = 0.3;
  coeffs(3) = 0.5;

  exponents(0,0) = 1;
  exponents(1,0) = 2;
  exponents(2,0) = 3;
  exponents(3,0) = 4;
  exponents(0,1) = 5;
  exponents(1,1) = 6;
  exponents(2,1) = 7;
  exponents(3,1) = 8;

  coeffs2 = coeffs*2.0;

  //vsl_print_summary(vcl_cout, coeffs);

  vnl_real_npolynomial poly_out(coeffs, exponents), poly_in0,poly_in1(coeffs2,exponents);

  vsl_b_ofstream bfs_out("vnl_real_npolynomial_test_io.bvl.tmp");
  TEST ("Created vnl_real_npolynomial_test_io.bvl.tmp for writing",
        (!bfs_out), false);
  vsl_b_write(bfs_out, poly_out);
  vsl_b_write(bfs_out, poly_out);
  bfs_out.close();

  vsl_b_ifstream bfs_in("vnl_real_npolynomial_test_io.bvl.tmp");
  TEST ("Opened vnl_real_npolynomial_test_io.bvl.tmp for reading",
        (!bfs_in), false);
  vsl_b_read(bfs_in, poly_in0);
  vsl_b_read(bfs_in, poly_in1);
  bfs_in.close();

  TEST ("poly_out.coefficients() == poly_in0.coefficients()",
        poly_out.coefficients() == poly_in0.coefficients(), true);
  TEST ("poly_out.coefficients() == poly_in1.coefficients()",
        poly_out.coefficients() == poly_in1.coefficients(), true);

  vsl_print_summary(vcl_cout, poly_out);
  vcl_cout << vcl_endl;
}

#if 0
void test_vector()
{
  vnl_vector_test_double_io();
}

TESTMAIN(test_vector);
#endif
