// This is vxl/vbl/io/tests/test_bounding_box_io.cxx
#include <vcl_iostream.h>
#include <vbl/vbl_bounding_box.h>
#include <vbl/io/vbl_io_bounding_box.h>
#include <vsl/vsl_binary_io.h>
#include <testlib/testlib_test.h>

void test_bounding_box_double_io()
{
  vcl_cout << "***********************************\n"
           << "Testing vbl_bounding_box<double> io\n"
           << "***********************************\n";
  //// test constructors, accessors
  vbl_bounding_box<double, 2> p_out, p_in;
  double X = 1.2;
  double Y = 3.4;

  p_out.update(X,Y);//Bounding box now has only one point and so no size
  X = 5.6;
  Y = 7.8;
  p_out.update(X,Y); // Second point now defines a bounding box

  vsl_b_ofstream bfs_out("vbl_bounding_box_test_double_io.bvl.tmp");
  TEST ("Created vbl_bounding_box_test_double_io.bvl.tmp for writing",
    (!bfs_out), false);
  vsl_b_write(bfs_out, p_out);
  bfs_out.close();


  vsl_b_ifstream bfs_in("vbl_bounding_box_test_double_io.bvl.tmp");
  TEST ("Opened vbl_bounding_box_test_double_io.bvl.tmp for reading",
    (!bfs_in), false);
  vsl_b_read(bfs_in, p_in);
  TEST ("Finished reading file successfully", (!bfs_in), false);
  bfs_in.close();


  TEST ("p_out.empty() == p_in.empty()", p_out.empty(), p_in.empty());
  TEST ("p_out.min()[0] == p_in.min()[0]", p_out.min()[0], p_in.min()[0]);
  TEST ("p_out.min()[1] == p_in.min()[1]", p_out.min()[1], p_in.min()[1]);
  TEST ("p_out.max()[0] == p_in.max()[0]", p_out.max()[0], p_in.max()[0]);
  TEST ("p_out.max()[1] == p_in.max()[1]", p_out.max()[1], p_in.max()[1]);

  vsl_print_summary(vcl_cout, p_out);
  vcl_cout << vcl_endl;
}


void test_bounding_box_prime()
{
  test_bounding_box_double_io();
}

TESTLIB_DEFINE_MAIN(test_bounding_box_prime);
