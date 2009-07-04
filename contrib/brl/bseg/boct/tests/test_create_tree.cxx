
#include <testlib/testlib_test.h>

#include <boct/boct_tree.h>

MAIN( test_create_tree )
{
  START ("CREATE TREE");
  short nlevels=5;
  boct_tree<short,vgl_point_3d<double> > * block=new boct_tree<short,vgl_point_3d<double> >(nlevels);
  TEST("No of Max levels of tree",nlevels, block->num_levels());

  block->split(); //block->print();

  vcl_vector<boct_tree_cell<short,vgl_point_3d<double> >*> leaves = block->leaf_cells();
  TEST("No of Leaf Cells", 8, leaves.size());
  for (unsigned i=0; i<leaves.size(); i++)
    leaves[i]->print();


  boct_tree<short,vgl_point_3d<double> > *init_tree = new boct_tree<short,vgl_point_3d<double> >(5, 3);
  vcl_vector<boct_tree_cell<short,vgl_point_3d<double> >*> leaves2 = init_tree->leaf_cells();
  TEST("No of Leaf Cells after 3 levels", 8*8, leaves2.size());
  //init_tree->print();
  delete block;
  delete init_tree;
  SUMMARY();
}
