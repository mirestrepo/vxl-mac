//*****************************************************************************
// File name: test_gevd_noise.cxx
// Description: Test gevd_noise class
//-----------------------------------------------------------------------------
//
// Version |Date      | Author                   |Comment
// --------+----------+--------------------------+-----------------------------
// 1.0     |2003/02/02| Peter Vanroose           |Creation
//*****************************************************************************

#include <gevd/gevd_noise.h>
#include <testlib/testlib_test.h>
#include <vnl/vnl_sample.h>
#include <vcl_iostream.h>

void
test_gevd_noise()
{
  const int size=100000;
  vnl_sample_reseed();
  float data[size];
  for (int i=0; i<size; ++i) data[i]=(float)vnl_sample_uniform(0,2);
  gevd_noise noise_estim(data,size);
  float sensor_noise, texture_noise;
  noise_estim.EstimateSensorTexture(sensor_noise, texture_noise);
  vcl_cout << "sensor_noise = " << sensor_noise
           << ", texture_noise = " << texture_noise << vcl_endl;
  TEST("gevd_noise", sensor_noise <= 2 && sensor_noise >= 0 &&
                     texture_noise <= 2 && texture_noise >= 0, true);
}

TESTMAIN(test_gevd_noise);
