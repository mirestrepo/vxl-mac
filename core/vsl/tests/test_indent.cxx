#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_utility.h>
#include <vcl_string.h>

#include <vsl/vsl_test.h>
#include <vsl/vsl_indent.h>

void test_indent1()
{
  vcl_cout<<"Tab Size is "<<vsl_indent_tab(vcl_cout)<<vcl_endl
          <<vsl_indent()<<"No Indent\n";
  vsl_indent_inc(vcl_cout);
  vcl_cout<<vsl_indent()<<"1 Indent\n";
  vsl_indent_inc(vcl_cout);
  vcl_cout<<vsl_indent()<<"2 Indent\n";
  vsl_indent_dec(vcl_cout);
  vcl_cout<<vsl_indent()<<"1 Indent\n";
  vsl_indent_dec(vcl_cout);
  vcl_cout<<vsl_indent()<<"No Indent\n\n";
}

void test_indent()
{
  vcl_cout << "*******************\n"
           << "Testing vsl_indent\n"
           << "*******************\n\n";

  vcl_cout<<"Using Tab size 2:\n\n";
  vsl_indent_set_tab(vcl_cout,2);
  test_indent1();


  vcl_cout<<"Using Tab size 4:\n\n";
  vsl_indent_set_tab(vcl_cout,4);
  test_indent1();

  vcl_cout<<vsl_indent()<<"Indent 0\n";
  vsl_indent_inc(vcl_cout);
  vcl_cout<<vsl_indent()<<"Indent 1\n";
  vsl_indent_inc(vcl_cout);
  vcl_cout<<vsl_indent()<<"Indent 2\n";
  vsl_indent_clear(vcl_cout);
  vcl_cout<<vsl_indent()<<"Indent Cleared\n";
}

TESTMAIN(test_indent);
