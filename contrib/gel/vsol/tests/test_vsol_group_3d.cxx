//*****************************************************************************
// File name: test_vsol_group_3d.cxx
// Description: Test the vsol_group_3d class
//-----------------------------------------------------------------------------
// Language: C++
//
// Version |Date      | Author                   |Comment
// --------+----------+--------------------------+-----------------------------
// 1.0     |2000/05/09| Fran�ois BERTEL          |Creation
//*****************************************************************************

//:
//  \file

#include <vcl_iostream.h>
#include <vcl_cassert.h>
#include <vsol/vsol_group_3d.h>
#include <vsol/vsol_point_3d.h>

//-----------------------------------------------------------------------------
//: Entry point of the test program
//-----------------------------------------------------------------------------
int main(int argc,
         char *argv[])
{
  int result=0;
  vsol_group_3d_sptr group1=new vsol_group_3d;
  vsol_point_3d_sptr p=new vsol_point_3d(10,4,5);
  vsol_group_3d_sptr group2;

  vcl_cout<<"size()"<<vcl_endl;
  vcl_cout<<group1->size()<<vcl_endl;
  assert(group1->size()==0);

  vcl_cout<<"deep_size()"<<vcl_endl;
  vcl_cout<<group1->deep_size()<<vcl_endl;
  assert(group1->deep_size()==0);

  group1->add_object(p.ptr());

  vcl_cout<<"size()"<<vcl_endl;
  vcl_cout<<group1->size()<<vcl_endl;
  assert(group1->size()==1);

  vcl_cout<<"deep_size()"<<vcl_endl;
  vcl_cout<<group1->deep_size()<<vcl_endl;
  assert(group1->deep_size()==1);

  group2=new vsol_group_3d;

  group1->add_object(group2.ptr());

  vcl_cout<<"size()"<<vcl_endl;
  vcl_cout<<group1->size()<<vcl_endl;
  assert(group1->size()==2);

  vcl_cout<<"deep_size()"<<vcl_endl;
  vcl_cout<<group1->deep_size()<<vcl_endl;
  assert(group1->deep_size()==1);

  vcl_cout<<"object()"<<vcl_endl;
  assert(*(group1->object(0))==*p);

  vcl_cout<<"remove_object()"<<vcl_endl;
  group1->remove_object(0);
  assert(group1->size()==1);
  assert(group1->object(0)->cast_to_group()!=0); // It is group2 now

  return result;
}
