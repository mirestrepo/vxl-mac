#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_utility.h>

#include <vsl/vsl_test.h>
#include <vsl/vsl_binary_explicit_io.h>
#include <vcl_ctime.h>

#include <vcl_sstream.h>


void test_arbitrary_length_int_conversion_int()
{
  vcl_cout << "*********************************************************"
           << vcl_endl;
  vcl_cout << "Testing arbitrary length int conversion for unsigned ints"
           << vcl_endl;
  vcl_cout << "*********************************************************"
           << vcl_endl;

  signed int  * a = new signed int [25000000];
  signed int  * b = new signed int [25000000];

  int i;
  for (i = 0; i < 25000000; ++i)
    a[i] = ((i-12500000)*160) ;

  unsigned maxbuf =  VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(signed int)) *
    25000000;

  unsigned char * buf = new unsigned char[maxbuf];
  
  clock_t t1, t2;

  vcl_cout << " Starting encode " << vcl_endl;
  
  t1 = clock();
  unsigned long len = vsl_convert_to_arbitrary_length(a, buf, 25000000);
  t2 = clock();

  vcl_cout << " Required " << (double)(t2-t1) / CLOCKS_PER_SEC
           << " seconds to encode 25M ints." <<vcl_endl;

  vcl_cout << " Max required buffer size is " << maxbuf << " bytes. Used "
           << len << vcl_endl;

  TEST ("Checking that the buffer didn't overflow", len < maxbuf, true);

  vcl_cout << " Starting decode " << vcl_endl;
  t1 = clock();
  unsigned long len2 = vsl_convert_from_arbitrary_length(buf, b, 25000000);
  t2 = clock();
  vcl_cout << " Required " << (double)(t2-t1) / CLOCKS_PER_SEC
           << " seconds to decode and test 25M ints." <<vcl_endl;

  TEST ("Checking len == len2", len == len2, true);

  for (i=0; i <25000000; ++i)
    if (b[i] != (i-12500000)*160) break;

  TEST ("Checking that the results are correct", i == 25000000, true);
  if (i != 25000000)
    vcl_cout << "Failed at number " << i <<vcl_endl;
  delete a;
  delete b;
  delete buf;

}




void test_arbitrary_length_int_conversion_short()
{
  vcl_cout << "*********************************************************"
           << vcl_endl;
  vcl_cout << "Testing arbitrary length int conversion for signed shorts"
           << vcl_endl;
  vcl_cout << "*********************************************************"
           << vcl_endl;

  signed short  a[65538];
  signed short  b[65540];
  signed short * c = &b[1];

  int i;
  for (i = 0; i < 65536; ++i)
    a[i] = i-32768;
  a[65536] = 0;
  a[65537] = 1;

  unsigned maxbuf = 
    VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(unsigned short))
    * 65538;

  unsigned char * buf = new unsigned char[maxbuf];
  unsigned long len = vsl_convert_to_arbitrary_length(a, buf, 65538);
  vcl_cout << " Max required buffer size is " << maxbuf << ". Used " << len
           << vcl_endl;

  TEST ("Checking that the buffer didn't overflow", len < maxbuf, true);
  
  b[0] = (short) (0xc5c5);
  b[65539] = 0x5c5c;
  unsigned long len2 = vsl_convert_from_arbitrary_length(buf, c, 65538);

  TEST ("Checking that the result buffer didn't overflow",
    (b[0] == (short)0xc5c5) && (b[65539] == 0x5c5c), true);

  TEST ("Checking len == len2", len == len2, true);

  for (i=0; i <65536; ++i)
    if (c[i] != i-32768) break;
  TEST ("Checking that the results are correct", i == 65536, true);
  if (i != 65536)
    vcl_cout << "Failed at number " << i <<vcl_endl;

  TEST ("Checking the end conditions", c[65536] == 0 && c[65537] == 1, true);

  delete buf;

}




void test_arbitrary_length_int_conversion_ushort()
{
  vcl_cout << "***********************************************************"
           << vcl_endl;
  vcl_cout << "Testing arbitrary length int conversion for unsigned shorts"
           << vcl_endl;
  vcl_cout << "***********************************************************"
           << vcl_endl;

  unsigned short  a[65538];
  unsigned short  b[65540];
  unsigned short * c = &b[1];

  int i;
  for (i = 0; i < 65536; ++i)
    a[i] = i;
  a[65536] = 0;
  a[65537] = 1;

  unsigned maxbuf = 
    VSL_MAX_ARBITRARY_INT_BUFFER_LENGTH(sizeof(unsigned short))
    * 65538;

  unsigned char * buf = new unsigned char[maxbuf];
  unsigned long len = vsl_convert_to_arbitrary_length(a, buf, 65538);
  vcl_cout << " Max required buffer size is " << maxbuf << ". Used " << len
           << vcl_endl;

  TEST ("Checking that the buffer didn't overflow", len < maxbuf, true);
  
  b[0] = 0xc5c5;
  b[65539] = 0x5c5c;
  unsigned len2 = vsl_convert_from_arbitrary_length(buf, c, 65538); 

  TEST ("Checking that the result buffer didn't overflow", (b[0] == 0xc5c5)
    && (b[65539] == 0x5c5c), true);
  TEST ("Checking len == len2", len == len2, true);

  for (i=0; i <65536; ++i)
    if (c[i] != i) break;
  TEST ("Checking that the results are correct", i == 65536, true);
  if (i != 65536)
    vcl_cout << "Failed at number " << i <<vcl_endl;

  TEST ("Checking the end conditions", c[65536] == 0 && c[65537] == 1, true);
  delete buf;


}

void test_explicit_int_io()
{
  vcl_cout << "**********************************" << vcl_endl;
  vcl_cout << "Testing explicit length integer io" << vcl_endl;
  vcl_cout << "**********************************" << vcl_endl;

  int i;

  vsl_b_ofstream bfs_out("vsl_explicit_int_io_test.bvl.tmp",
    vcl_ios_openmode(vcl_ios_out | vcl_ios_binary));
  TEST ("Created vsl_explicit_int_io_test.bvl.tmp for writing",
    (!bfs_out), false);
  for (i = 0; i < 65536; ++i)
    vsl_b_write_uint_16(bfs_out, i);
  bfs_out.close();

  vsl_b_ifstream bfs_in("vsl_explicit_int_io_test.bvl.tmp",
    vcl_ios_openmode(vcl_ios_in | vcl_ios_binary));
  TEST ("Opened vsl_explicit_int_io_test.bvl.tmp for reading",
    (!bfs_in), false);
  for (i = 0; i < 65536; ++i)
  {
    unsigned long n;
    vsl_b_read_uint_16(bfs_in, n);
    if (n != i) break;
  }

// Commented out because stringstream not yet properly supported on gcc-2.95.2
#if 0
  vcl_stringstream ss(vcl_ios_in | vcl_ios_out | vcl_ios_binary);
  const char *b= ss.str().c_str();
  {
    vsl_b_ostream bss(&ss);
    TEST ("Created stringstream for writing", (!bss), false);
    for (i = 0; i < 65536; ++i)
      vsl_b_write_uint_16(bss, i);
  }

  vcl_stringstream ss2(ss.str());
  {
    vsl_b_istream bss(&ss2);
    TEST ("Opened stringstream for reading", (!bss), false);
    for (i = 0; i < 65536; ++i)
    {
      unsigned long n;
      vsl_b_read_uint_16(bss, n);
      if (n != i) break;
    }
  }
#endif

  TEST ("Checking that the results are correct", i == 65536, true);
  if (i != 65536)
    vcl_cout << "Failed at number " << i <<vcl_endl;






}
void test_extreme_int_io()
{
  vcl_cout << "*************************************" << vcl_endl;
  vcl_cout<<  "Testing largest/smallest integer I/O"<<vcl_endl;
  vcl_cout << "************************************" << vcl_endl;

  // Some fudges to get the max values
  // vcl_numeric_limits doesn't seem to work yet
  long min_long = 1L<<(8*sizeof(long)-1);
  long max_long = ~min_long;
  unsigned long max_ulong = ~0;

  vsl_b_ofstream bfs_out("vsl_extreme_int_io_test.bvl.tmp",
    vcl_ios_openmode(vcl_ios_out | vcl_ios_binary));
  TEST ("Created vsl_extreme_int_io_test.bvl.tmp for writing",
    (!bfs_out), false);

  vsl_b_write(bfs_out,min_long);
  vsl_b_write(bfs_out,max_long);
  vsl_b_write(bfs_out,max_ulong);
  bfs_out.close();

  long min_long_in = 77;
  long max_long_in = 77;
  unsigned long max_ulong_in = 77;


  vsl_b_ifstream bfs_in("vsl_extreme_int_io_test.bvl.tmp",
    vcl_ios_openmode(vcl_ios_in | vcl_ios_binary));
  TEST ("Opened vsl_extreme_int_io_test.bvl.tmp for reading",
    (!bfs_in), false);
  vsl_b_read(bfs_in,min_long_in);
  vsl_b_read(bfs_in,max_long_in);
  vsl_b_read(bfs_in,max_ulong_in);
  bfs_in.close();

  TEST ("min_long == min_long_in", min_long == min_long_in, true);
  TEST ("max_long == max_long_in", max_long == max_long_in, true);
  TEST ("max_ulong == max_ulong_in", max_ulong == max_ulong_in, true);

}



void test_arbitrary_length_int_conversion()
{
  test_arbitrary_length_int_conversion_ushort();
  test_arbitrary_length_int_conversion_short();
  test_explicit_int_io();
  test_extreme_int_io();
//  test_arbitrary_length_int_conversion_int();
}


TESTMAIN(test_arbitrary_length_int_conversion);
