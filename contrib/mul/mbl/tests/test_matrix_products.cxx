#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_utility.h>
#include <vcl_cmath.h> // for vcl_sqrt()

#include <vnl/vnl_test.h>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>
#include <mbl/mbl_matrix_products.h>

void test_matrix_products()
{
  vcl_cout << "******************************" << vcl_endl;
  vcl_cout << " Testing mbl_matrix_products " << vcl_endl;
  vcl_cout << "*****************************" << vcl_endl;

  vnl_matrix<double> A(4,5),B(4,6);
  for (int i=0;i<A.rows();++i)
     for (int j=0;j<A.cols();++j)
         A(i,j) = 1+double(i+j*j);

  for (int i=0;i<B.rows();++i)
     for (int j=0;j<B.cols();++j)
         B(i,j) = 1+i*i-double(j);
  
  vnl_matrix<double> AtB;
  mbl_matrix_product_at_b(AtB,A,B);

  vnl_matrix<double> dAtB = AtB - A.transpose() * B;
  TEST("mbl_matrix_product_at_b",dAtB.absolute_value_max()<1e-6,true);

  mbl_matrix_product(AtB,A.transpose(),B);
  dAtB = AtB - A.transpose() * B;
  TEST("mbl_matrix_productb",dAtB.absolute_value_max()<1e-6,true);


  vnl_matrix<double> C = A.transpose();
  vnl_matrix<double> D = B.transpose();
  vnl_matrix<double> CDt;
  mbl_matrix_product_a_bt(CDt,C,D);
  vnl_matrix<double> dCDt = CDt - C * D.transpose();
  TEST("mbl_matrix_product_a_bt",dCDt.absolute_value_max()<1e-6,true);

  vnl_vector<double> d(4);
  vnl_matrix<double> W(4,4);
  W.fill(0);
  for (int i=0;i<d.size();++i) { d(i)=i; W(i,i)=d(i); }

  vnl_matrix<double> CWB;
  mbl_matrix_product_adb(CWB,C,d,B);
  vnl_matrix<double> dCWB = CWB - C * W * B;
  TEST("mbl_matrix_product_adb",dCWB.absolute_value_max()<1e-6,true);
}

TESTMAIN(test_matrix_products);
