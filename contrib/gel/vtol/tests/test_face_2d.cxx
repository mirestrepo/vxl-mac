// This is gel/vtol/tests/test_face_2d.cxx
#include <testlib/testlib_test.h>
#include <vtol/vtol_vertex_2d_sptr.h>
#include <vtol/vtol_vertex_2d.h>
#include <vtol/vtol_edge_2d.h>
#include <vtol/vtol_edge.h>
#include <vtol/vtol_edge_sptr.h>
#include <vtol/vtol_one_chain_sptr.h>
#include <vtol/vtol_one_chain.h>
#include <vtol/vtol_face_2d.h>
#include <vtol/vtol_face_2d_sptr.h>

static void test_face_2d()
{
  vcl_cout << "testing face 2d\n";

  vtol_vertex_2d_sptr v1 = new vtol_vertex_2d(0.0,0.0);
  vtol_vertex_2d_sptr v2 = new vtol_vertex_2d(1.0,1.0);
  vtol_vertex_2d_sptr v3 = new vtol_vertex_2d(2.0,2.0);
  vtol_vertex_2d_sptr v4 = new vtol_vertex_2d(3.0,3.0);

  vertex_list v_list;

  v_list.push_back(v1->cast_to_vertex());
  v_list.push_back(v2->cast_to_vertex());
  v_list.push_back(v3->cast_to_vertex());
  v_list.push_back(v4->cast_to_vertex());

  vtol_face_2d_sptr f1 = new vtol_face_2d(v_list);
  f1->describe(vcl_cout,8);

  vtol_edge_sptr e12 = new vtol_edge_2d(*v1,*v2);
  vtol_edge_sptr e23 = new vtol_edge_2d(*v2,*v3);
  vtol_edge_sptr e34 = new vtol_edge_2d(*v3,*v4);
  vtol_edge_sptr e41 = new vtol_edge_2d(*v4,*v1);

  edge_list e_list;

  e_list.push_back(e12);
  e_list.push_back(e23);
  e_list.push_back(e34);
  e_list.push_back(e41);

  vtol_one_chain_sptr oc1 = new vtol_one_chain(e_list);

  one_chain_list oc_list;
  oc_list.push_back(oc1);

  vtol_face_2d_sptr f2 = new vtol_face_2d(oc_list);
  f2->describe(vcl_cout,8);

  TEST("vtol_face_2d::get_one_chain()", f2->get_one_chain(), oc1.ptr());
  TEST("vtol_face_2d::get_boundary_cycle()", f2->get_boundary_cycle(), oc1.ptr());

  edge_list he_list;

  e_list.push_back(e12);
  e_list.push_back(e23);
  e_list.push_back(e34);

  vtol_one_chain_sptr oc1_hole = new vtol_one_chain(he_list);

  f2->add_hole_cycle(*oc1_hole);

  vcl_vector<vtol_one_chain_sptr> *holes = f2->get_hole_cycles();
  TEST("vtol_face_2d::get_hole_cycles()", holes->size(), 1);
  TEST("vtol_face_2d::get_hole_cycles()", *((*holes)[0]), *oc1_hole);
  delete holes;

  vtol_face_2d_sptr f3 = new vtol_face_2d(*oc1);
  f3->describe(vcl_cout,8);
  TEST("vtol_face_2d inequality", *f1 == *f3, false);
  TEST("vtol_face_2d inequality", *f1->cast_to_face() == *f3->cast_to_face(), false);

  vtol_face_2d_sptr f1_copy = new vtol_face_2d(*f1);

  TEST("vtol_face_2d copy constructor", *f1, *f1_copy);
  TEST("vtol_face_2d::cast_to_face_2d()", f1->cast_to_face_2d()==0, false);
  TEST("vtol_face_2d::cast_to_face()", *f1->cast_to_face(), *f1_copy->cast_to_face());

  vsol_spatial_object_2d_sptr f1_clone = f1->clone();
  TEST("vtol_face_2d::clone()", *f1, *f1_clone);
  f1_clone->describe(vcl_cout,8);

  vsol_region_2d_sptr surf = f1->surface();
  TEST("vtol_face_2d::surface()", surf.ptr()==0, false);

  vtol_vertex_2d_sptr v5 = new vtol_vertex_2d(4.0,4.0);
  v_list.push_back(v5->cast_to_vertex());

  vtol_face_2d_sptr new_f = new vtol_face_2d(v_list);

  TEST("vtol_face_2d inequality", *f1==*new_f, false);
  TEST("vtol_face_2d::topology_type()", f1->topology_type(), vtol_topology_object::FACE);
  TEST("vtol_face_2d::cast_to_face()", f1->cast_to_face()==0, false);
  TEST("vtol_face_2d::valid_inferior_type()", f1->valid_inferior_type(*oc1), true);
  TEST("vtol_face_2d::valid_inferior_type()", f1->valid_inferior_type(*e12), false);
  TEST("vtol_one_chain::valid_superior_type()", oc1->valid_superior_type(*f1), true);
  TEST("vtol_face_2d::valid_superior_type()", f1->valid_superior_type(*oc1), false);
  TEST("vtol_face_2d::get_num_edges()", f1->get_num_edges(), 4);
  TEST("vtol_face_2d::shares_edge_with()", f1->shares_edge_with(*new_f), true);
}

TESTLIB_DEFINE_MAIN(test_face_2d);
