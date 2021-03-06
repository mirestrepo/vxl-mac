#include <testlib/testlib_test.h>
#include <vcl_iostream.h>
#include <vpgl/vpgl_proj_camera.h>
#include <vpgl/io/vpgl_io_proj_camera.h>
#include <vpl/vpl.h>
#include <vsl/vsl_binary_io.h>

static void test_proj_camera_io()
{
  //--------------------------------------------------------------------------
  
  vcl_cout << "Testing Projective camera" << vcl_endl;

  // Some matrices for testing.
  double random_list[12] = { 10.6, 1.009, .676, .5, -13, -10, 8, 5, 88, -2, -100, 11 };
  vnl_double_3x4 random_matrix( random_list );

  vcl_cout << "Matrix:\n" << random_matrix << vcl_endl;
  vpgl_proj_camera<double> p_cam(random_matrix);

  vsl_b_ofstream bp_out("test_proj_camera_io.tmp");
  TEST("Created test_proj_camera_io.tmp for writing",(!bp_out), false);

  vsl_b_write(bp_out, p_cam);
  bp_out.close();

  // test input binary stream

  vsl_b_ifstream bp_in("test_proj_camera_io.tmp");
  TEST("Opened test_proj_camera_io.tmp for reading",(!bp_in), false);

  vpgl_proj_camera<double> p_cam_r;
  vsl_b_read(bp_in, p_cam_r);
  bp_in.close();

  vcl_cout << "Recovered Matrix:\n" << p_cam_r.get_matrix() << vcl_endl;
  TEST("recovery from binary read", p_cam_r.get_matrix(), random_matrix );
  // remove file:
  vpl_unlink("test_proj_camera_io.tmp");
  
}

TESTMAIN(test_proj_camera_io);
