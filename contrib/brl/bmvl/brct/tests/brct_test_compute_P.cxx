// This is brl/bmvl/brct/tests/brct_test_compute_P.cxx
#include <vcl_iostream.h>
#include <vcl_vector.h>
#include <brct/brct_algos.h>
#include <vnl/vnl_double_3.h>
#include <vnl/vnl_double_4.h>
#include <vnl/vnl_double_3x3.h>
#include <vnl/vnl_double_3x4.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_point_3d.h>
#include <testlib/testlib_test.h>

static vnl_double_3x3 generate_K()
{
  vnl_double_3x3 K;
  // set up the intrinsic matrix of the camera
  K[0][0] = -2000;     K[0][1] = 0;        K[0][2] = 512;
  K[1][0] = 0;        K[1][1] = -2000;     K[1][2] = 384;
  K[2][0] = 0;        K[2][1] = 0;        K[2][2] = 1;
  return K;
}

static vnl_double_3x4 generate_P(vnl_double_3x3 const & K)
{
  vnl_double_3x4 P;
  P[0][0] = -0.25105;  P[0][1] = -0.9604;  P[0][2] = 0.00908; P[0][3] = 14.5064;
  P[1][0] = -0.1;      P[1][1] = 0.0355;   P[1][2] = 0.99;    P[1][3] = -0.14;
  P[2][0] = -0.96;     P[2][1] = 0.25;     P[2][2] = -0.1;    P[2][3] = 56.28;
  return K*P;
}

static void brct_test_compute_P()
{
  vnl_double_3x3 K = generate_K();
  vnl_double_3x4 P = generate_P(K);
  //generate a set of 3-d points (at least 6)
  //Eight vertices of a cube
  const unsigned int n = 11;
  vnl_double_4 pw[11];
  pw[0] = vnl_double_4(0.0, 0.0, 0.0, 1.0);
  pw[1] = vnl_double_4(10.0, 0.0, 0.0, 1.0);
  pw[2] = vnl_double_4(10.0, 10.0, 0.0, 1.0);
  pw[3] = vnl_double_4(0.0, 10.0, 0.0, 1.0);
  pw[4] = vnl_double_4(0.0, 0.0, -5.0, 1.0);
  pw[5] = vnl_double_4(10.0, 0.0, -5.0, 1.0);
  pw[6] = vnl_double_4(10.0, 10.0, -5.0, 1.0);
  pw[7] = vnl_double_4(0.0, 10.0, -5.0, 1.0);

  pw[8] = vnl_double_4(100.0, 0.0, 0, 1.0);
  pw[9] = vnl_double_4(0.0, 100.0, 0, 1.0);
  pw[10] = vnl_double_4(0.0, 0.0, 100.0, 1.0);
  vcl_vector<vgl_point_2d<double> > image_points;
  vcl_vector<vgl_point_3d<double> > world_points;
  //project the points
  vnl_double_3 pi[11];
  for (unsigned int i = 0; i<n; i++)
  {
    pi[i]=P*pw[i];
    double u = pi[i][0]/pi[i][2], v = pi[i][1]/pi[i][2];
    vgl_point_2d<double> gp(u, v);
    vcl_cout << "Pw" << pw[i] << "->  pi(" << gp.x()
             << ' ' << gp.y() << ")\n";
    image_points.push_back(gp);
    world_points.push_back(vgl_point_3d<double>(pw[i][0], pw[i][1], pw[i][2]));
  }
  vnl_double_3x4 Pout;
  brct_algos::compute_euclidean_camera(image_points, world_points, K, Pout);
  vcl_cout << "Projecting points From reconstituted camera\n";
  vnl_double_3 prp;
  for (unsigned int i = 0; i<n; i++)
  {
    prp = Pout*pw[i];
    vcl_cout << "Pw" << pw[i] << "->  prp(" << prp[0]/prp[2]
             << ' ' << prp[1]/prp[2] << ")\n";
  }
  vcl_cout << "Original P\n" << P << '\n';
  vcl_cout << "P in real world and image coordinates\n" << 120.82*Pout << '\n';
}

TESTMAIN(brct_test_compute_P);
