#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_utility.h>

#include <vgl/vgl_test.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/io/vgl_io_point_3d.h>


void test_point_3d_double_io()
{
  vcl_cout << "***********************" << vcl_endl;
  vcl_cout << "Testing vgl_point_3d<double> io" << vcl_endl;
  vcl_cout << "***********************" << vcl_endl;
  //// test constructors, accessors
  vgl_point_3d<double> p_out(1.2,3.4,5.6), p_in;


  vsl_b_ofstream bfs_out("vgl_point_3d_test_double_io.bvl.tmp");
  TEST ("Created vgl_point_3d_test_double_io.bvl.tmp for writing",
        (!bfs_out), false);
  vsl_b_write(bfs_out, p_out);
  bfs_out.close();

  vsl_b_ifstream bfs_in("vgl_point_3d_test_double_io.bvl.tmp");
  TEST ("Opened vgl_point_3d_test_double_io.bvl.tmp for reading",
        (!bfs_in), false);
  vsl_b_read(bfs_in, p_in);
  bfs_in.close();



  TEST ("p_out == p_in", p_out == p_in, true);


  vsl_print_summary(vcl_cout, p_out);
  vcl_cout << vcl_endl;

}


void test_point_3d_prime()
{
  test_point_3d_double_io();
}


TESTMAIN(test_point_3d_prime);
