#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_utility.h>

#include <vil/vil_test.h>
#include <vil/io/vil_io_rgba.h>


void test_rgba_double_io()
{
  vcl_cout << "***********************" << vcl_endl;
  vcl_cout << "Testing vil_rgba<double> io" << vcl_endl;
  vcl_cout << "***********************" << vcl_endl;
  //// test constructors, accessors
  vil_rgba<double> p_out(1.2,3.4,5.6,7.8), p_in;


  vsl_b_ofstream bfs_out("vil_rgba_test_double_io.bvl.tmp");
  TEST ("Created vil_rgba_test_double_io.bvl.tmp for writing", 
    (!bfs_out), false);
  vsl_b_write(bfs_out, p_out);
  bfs_out.close();

  vsl_b_ifstream bfs_in("vil_rgba_test_double_io.bvl.tmp");
  TEST ("Opened vil_rgba_test_double_io.bvl.tmp for reading", 
    (!bfs_in), false);
  vsl_b_read(bfs_in, p_in);
  TEST ("Finished reading file successfully", (!bfs_in), false);
  bfs_in.close();



  TEST ("p_out == p_in", 
    p_out.R()==p_in.R() && 
    p_out.G()==p_in.G() && 
    p_out.B()==p_in.B() && 
    p_out.A()==p_in.A() , true);


  vsl_print_summary(vcl_cout, p_out);
  vcl_cout << vcl_endl;

}


void test_rgba_prime()
{
  test_rgba_double_io();
}


TESTMAIN(test_rgba_prime);
