#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_utility.h>

#include <vbl/vbl_test.h>
#include <vbl/io/vbl_io_sparse_array_2d.h>
#include <vbl/vbl_sparse_array_2d.h>


void test_sparse_array_io()
{
  vcl_cout << "**************************************" << vcl_endl;
  vcl_cout << "Testing vbl_sparse_array_2d<double> io" << vcl_endl;
  vcl_cout << "**************************************" << vcl_endl;

  vbl_sparse_array_2d<double> v_out, v_in;

  // fill v_in with incorrect values
   v_in(4,5) = 3.0;

  // create a sparse array - more than 5 elements so only
  // the first 5 are written out
  v_out(1,1)=0.4;
  v_out(2000,10000)=1e5;
  v_out(10,10)=0.0;
  for (unsigned k=60; k<70; k++)
    v_out(k,400)=30.3;

  vsl_b_ofstream bfs_out("vbl_sparse_array_test_io.bvl.tmp");
  TEST ("Created vbl_sparse_array_test_io.bvl.tmp for writing",
    (!bfs_out), false);
  vsl_b_write(bfs_out, v_out);
  bfs_out.close();

  vsl_b_ifstream bfs_in("vbl_sparse_array_test_io.bvl.tmp");
  TEST ("Opened vbl_sparse_array_test_io.bvl.tmp for reading",
    (!bfs_in), false);
  vsl_b_read(bfs_in, v_in);
  TEST ("Finished reading file successfully", (!bfs_in), false);
  bfs_in.close();

  bool test_result = true;
  //same number of non zero elements?
  if(v_out.count_nonempty() != v_in.count_nonempty())
    test_result=false;
  else {
    //check every key/data pair, require same order too.
  vbl_sparse_array_2d<double>::const_iterator s = v_in.begin();
  vbl_sparse_array_2d<double>::const_iterator r;
  //N.B. relies on sensible == operator for <T>
  for(r = v_out.begin(); r != v_out.end(); ++r){
    if(!((*s).first == (*r).first) || !((*s).second == (*r).second))
      test_result=false;
    s++;
  }
  }
  TEST ("v_out == v_in",test_result, true);

  vsl_print_summary(vcl_cout, v_in);
  vcl_cout << vcl_endl;
}

void test_sparse_array_prime()
{
  test_sparse_array_io();
}


TESTMAIN(test_sparse_array_prime);
