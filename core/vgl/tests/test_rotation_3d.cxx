// Test vgl_rotate_3d


#include <testlib/testlib_test.h>
#include <vgl/algo/vgl_rotation_3d.h>
#include <vgl/vgl_distance.h>
#include <vnl/vnl_double_3.h>
#include <vnl/vnl_double_3x3.h>
#include <vnl/vnl_random.h>


// Test conversions between various rotation representations
void test_conversions(const vgl_rotation_3d<double>& rot)
{
  vnl_double_3x3 R = rot.as_matrix();
  vgl_h_matrix_3d<double> H = rot.as_h_matrix_3d();
  vnl_double_3 rr = rot.as_rodriques();
  vnl_double_3 er = rot.as_euler_angles();
  vnl_quaternion<double> qr = rot.as_quaternion();

  double epsilon = 10e-8;

  qr *= (qr.real()>0.0)?1.0:-1.0;

  vnl_quaternion<double> qr_other = vgl_rotation_3d<double>(R).as_quaternion();
  qr_other *= (qr_other.real()>0.0)?1.0:-1.0;
  double diff1 = (qr_other - qr).magnitude();
  TEST_NEAR("Matrix conversion",diff1,0.0, epsilon);
  if(diff1 > epsilon){
    vcl_cout << "Matrix: \n" << R << vcl_endl;
  }

  qr_other = vgl_rotation_3d<double>(H).as_quaternion();
  qr_other *= (qr_other.real()>0.0)?1.0:-1.0;
  double diff2 = (qr_other - qr).magnitude();
  TEST_NEAR("3D Homography conversion",diff2,0.0, epsilon);
  if(diff2 > epsilon){
    vcl_cout << "3D Homography: \n" << H << vcl_endl;
  }

  qr_other = vgl_rotation_3d<double>(rr).as_quaternion();
  qr_other *= (qr_other.real()>0.0)?1.0:-1.0;
  double diff3 = (qr_other - qr).magnitude();
  TEST_NEAR("Rodriques conversion",diff3,0.0, epsilon);
  if(diff3 > epsilon){
    vcl_cout << "Rodriques: "<< rr << vcl_endl;
  }

  qr_other = vgl_rotation_3d<double>(er[0], er[1], er[2]).as_quaternion();
  qr_other *= (qr_other.real()>0.0)?1.0:-1.0;
  double diff4 = (qr_other - qr).magnitude();
  TEST_NEAR("Euler conversion",diff4,0.0, epsilon);
  if(diff4 > epsilon){
    vcl_cout << "Euler:  Rx=" << er[0]<< " Ry="<<er[1]<<" Rz="<<er[2] << vcl_endl;
  }

}


// Test that the inverse rotation works as expected
void test_inverse(const vgl_rotation_3d<double>& rot)
{
  double epsilon = 10e-8;
  vgl_rotation_3d<double> I = rot *rot.inverse();
  double diff = (I.as_quaternion() - vnl_quaternion<double>(0,0,0,1)).magnitude();
  TEST_NEAR("Inverse rotation", diff, 0.0,epsilon);
}


// Test application of rotation to other vgl objects
void test_application(const vgl_rotation_3d<double>& rot)
{
  vnl_double_3x3 R = rot.as_matrix();

  vgl_homg_point_3d<double> hpt(100,50,200);
  vnl_double_3 pt(hpt.x(),hpt.y(),hpt.z());

  vgl_homg_plane_3d<double> hpl(0,1,0,100);

  vgl_homg_point_3d<double> r_hpt = rot*hpt;
  vnl_double_3 r_pt = R*pt;

  vgl_homg_plane_3d<double> r_hpl = rot*hpl;

  double epsilon = 10e-8;
  TEST_NEAR("Rotated point-plane dist", vgl_distance(hpt,hpl),
            vgl_distance(r_hpt, r_hpl), epsilon);

  double diff = (r_pt - vnl_double_3(r_hpt.x()/r_hpt.w(),
                                     r_hpt.y()/r_hpt.w(),
                                     r_hpt.z()/r_hpt.w())).magnitude();
  TEST_NEAR("Rotated point", diff, 0.0, epsilon);

}


void test_rotation_3d()
{
  vcl_cout << "***************************\n"
           << " Testing vgl_rotation_3d\n"
           << "***************************\n\n";

  vgl_rotation_3d<double> rot_x90(1.5707963, 0.0, 0.0);
  test_conversions(rot_x90);
  test_inverse(rot_x90);
  test_application(rot_x90);


  vnl_random rnd;
  vgl_rotation_3d<double> rot_rand(rnd.normal(), rnd.normal(), rnd.normal());
  vcl_cout << "testing random rotation: "<< rot_rand.as_quaternion() <<vcl_endl;
  test_conversions(rot_rand);
  test_inverse(rot_rand);
}

TESTMAIN(test_rotation_3d);
