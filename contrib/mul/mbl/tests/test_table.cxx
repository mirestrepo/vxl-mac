// This is mul/mbl/tests/test_table.cxx


//: \file
// \brief Test program for the mbl_table class
// \author Kevin de Souza
// \date 2005-02-02


#include <vcl_iostream.h>
#include <mbl/mbl_table.h>
#include <testlib/testlib_test.h>


//========================================================================
// Test the piecemeal construction of a table 
//========================================================================
void test_table1()
{
  vcl_cout << "------------------------------- \n"
           << " Testing piecemeal construction \n"
           << "------------------------------- \n";
   
  char delim = '\t';
  vcl_vector<vcl_string> headers(3);
  headers[0] = "x coord";
  headers[1] = "y coord";
  headers[2] = "z coord";

  mbl_table table(delim, headers);
  vcl_vector<double> row0(3);
  row0[0] = 1.23;
  row0[1] = 3.45;
  row0[2] = 6.78;
  table.append_row(row0);
  vcl_vector<double> row1(3);
  row1[0] = -1.23;
  row1[1] = -3.45;
  row1[2] = -6.78;
  table.append_row(row1);
  TEST("ncols==3", table.num_cols()==3, true);
  TEST("nrows==2", table.num_rows()==2, true);

  vcl_vector<double> col_vals;
  TEST("get_column(): success", table.get_column(headers[1], col_vals), true);
  TEST("get_column(): size", col_vals.size()==2, true);
  TEST("get_column(): values", (col_vals[0]==3.45 && col_vals[1]==-3.45), true);

  vcl_vector<double> row_vals;
  TEST("get_row(): success", table.get_row(1, row_vals), true);
  TEST("get_row(): size", row_vals.size()==3, true);
  TEST("get_row(): values", (row_vals[0]==-1.23 && 
                             row_vals[1]==-3.45 && 
                             row_vals[2]==-6.78), true);
}


//========================================================================
// Run a series of tests
//========================================================================
void test_table()
{
  vcl_cout << "========================================\n"
           << " Testing mbl_table                      \n"
           << "========================================\n";
  
  test_table1();
}



TESTMAIN(test_table);
