#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_utility.h>

#include <vnl/vnl_test.h>
#include <vnl/vnl_real_polynomial.h>
#include <vnl/io/vnl_io_real_polynomial.h>


void test_real_polynomial_io()
{
    vcl_cout << "***********************" << vcl_endl;
    vcl_cout << "Testing vnl_real_polynomial io" << vcl_endl;
    vcl_cout << "***********************" << vcl_endl;  
    //// test constructors, accessors
    const int n = 10;
    vnl_vector<double> v(n);

    for (int i=0; i<n; i++)
    {
        v(i) = (double)(i*i);
    }
    
  vnl_real_polynomial poly_out(v), poly_in0(0),poly_in1(v*2.0);

    
    
    vsl_b_ofstream bfs_out("vnl_real_polynomial_test_io.bvl.tmp");
    TEST ("Created vnl_real_polynomial_test_io.bvl.tmp for writing",
             (!bfs_out), false);
    vsl_b_write(bfs_out, poly_out);
    vsl_b_write(bfs_out, poly_out);
    bfs_out.close();
        
    vsl_b_ifstream bfs_in("vnl_real_polynomial_test_io.bvl.tmp");
    TEST ("Opened vnl_real_polynomial_test_io.bvl.tmp for reading",
             (!bfs_in), false);
    vsl_b_read(bfs_in, poly_in0);
    vsl_b_read(bfs_in, poly_in1);
    bfs_in.close();
        
    
    
    TEST ("poly_out.coefficients() == poly_in0.coefficients()",
             poly_out.coefficients() == poly_in0.coefficients(), true);
    TEST ("poly_out.coefficients() == poly_in1.coefficients()",
             poly_out.coefficients() == poly_in1.coefficients(), true);

  vsl_print_summary(vcl_cout, poly_in0);
  vcl_cout << vcl_endl;
}


void test_real_polynomial_prime()
{
  test_real_polynomial_io();
}


TESTMAIN(test_real_polynomial_prime); 