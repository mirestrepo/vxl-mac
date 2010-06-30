#include <testlib/testlib_test.h>
//:
// \file
#include <testlib/testlib_root_dir.h>
#include "open_cl_test_data.h"
#include <boxm/ocl/boxm_refine_manager.h>
#include <boxm/boxm_scene.h>
#include <boct/boct_tree.h>
#include <boct/boct_tree_cell.h>
#include <boct/boct_loc_code.h>
#include <vnl/vnl_vector_fixed.h>
#include <vcl_vector.h>
#include <vcl_stack.h>
#include <boxm/ocl/boxm_ocl_utils.h>
#include <boxm/algo/boxm_refine.h>


//: Verifies that a tree is in the canonical format in the array
// Depth first search
bool verify_format(vcl_vector<vnl_vector_fixed<int, 4> > cell_array)
{
  unsigned curr_index = 0;
  vcl_stack<int> open;
  open.push(0);
  while (!open.empty()) {
    int currNode = open.top();
    open.pop();
    int child_ptr = cell_array[currNode][1];

    // if the current node has no children, nothing to verify
    if (child_ptr < 0) {
      continue;
    }
    // if child pointer isn't to the right place..
    if (child_ptr != curr_index+1) {
      return false;
    }

    // push children on stack in reverse order
    for (int i=7; i>=0; i--) {
      open.push(child_ptr+i);
    }
    curr_index += 8;
  }
  return true;
}

bool test_refine_simple_scene()
{
  // Set up test tree
  typedef boxm_sample<BOXM_APM_MOG_GREY> data_type; 
  typedef boct_tree<short,data_type> tree_type;
  tree_type* tree = open_cl_test_data::simple_tree<data_type>();
  float prob_thresh = .3f;

  //-------- GPU side: refine the scene using the opencl refine manager -------
  boxm_refine_manager<data_type>* mgr = boxm_refine_manager<data_type>::instance();
  if (!mgr->init(tree, prob_thresh)) {
    TEST("Error : boxm_refine : mgr->init() failed\n", false, true);
    return false;
  }
  if (!mgr->run_tree()) {
    TEST("Error : boxm_refine : mgr->run_tree() failed\n", false, true);
    return false;
  }

  // extract the output scene from the manager
  int* tree_array = mgr->get_tree();
  int  tree_size = mgr->get_tree_size();
  float* data = mgr->get_data();
  int  data_size = mgr->get_data_size();
  if (!tree_array) {
    TEST("Error : boxm_refine : mgr->get_tree() returned NULL\n", false, true);
    return false;
  }
  if (!tree_size) {
    TEST("ERROR : boxm_refine : mgr->get_tree_size() returned NULL\n", false, true);
    return false;
  }

  // Verify that the tree is formatted correctly
  vcl_vector<vnl_vector_fixed<int,4> > tree_vector;
  for (int i=0,j=0; j<tree_size; i+=4,j++) {
    vnl_vector_fixed<int,4> cell;
    for (unsigned k=0; k<4; k++) 
      cell[k] = tree_array[i+k];
    tree_vector.push_back(cell);
  }
  bool correctFormat = verify_format(tree_vector);
  TEST("test_refine_simple_scene output format", correctFormat, true);
  
  
  //--------- CPU side refine ------------------------------
  unsigned num_split = 0;
  typedef boxm_block<tree_type> block_type;
  block_type block(tree->bounding_box(), tree);
  boxm_refine_block(&block, prob_thresh, num_split); 
  
  //use vectors to build the tree up
  vcl_vector<vnl_vector_fixed<int, 4> > cell_input;
  vcl_vector<vnl_vector_fixed<float, 16>  > data_input;

  // put the root into the cell array and its data in the data array
  int cell_ptr = 0;
  vnl_vector_fixed<int, 4> root_cell(0);
  root_cell[0]=-1; // no parent
  root_cell[1]=-1; // no children at the moment
  root_cell[2]=-1; // no data at the moment
  cell_input.push_back(root_cell);
  boct_tree_cell<short,data_type>* root = tree->root();
  boxm_ocl_convert<data_type>::copy_to_arrays(root, cell_input, data_input, cell_ptr);
  
  //verify that tree_vector and cell_input are same size
  TEST("CPU refine and GPU refine tree output same size ", (tree_vector.size()), cell_input.size());
  TEST("CPU refine and GPU refine data output same size ", (data_input.size()), data_size);
  
  //Verify the tree's structure is correct
  bool good = true;
  for(unsigned i=0; i<tree_vector.size(); i++){
    for(int j=0; j<2; j++) //0 and 1 are parent and child pointers
      good = good && (tree_vector[i][j] == cell_input[i][j]);
  }
  TEST("CPU/GPU refine tree output same parent/child pointers ", good, true);
  
  //verify that the data for each node is the same
  float ssd = 0;
  for(unsigned i=0; i<tree_vector.size(); i++){
    
    //cpu side data
    int dataIndex = cell_input[i][2];
    vnl_vector_fixed<float, 16> datum = data_input[dataIndex];
    
    //gpu side data
    dataIndex = 16*tree_vector[i][2];
    
    //compare
    for(int j=0; j<16; j++)
      ssd += (datum[j]-data[dataIndex+j])*(datum[j]-data[dataIndex+j]);
  }
  TEST("CPU/GPU refine tree output same data ", (ssd<10e-8), true);  
  vcl_cout<<"SSD between cpu/gpu data = "<<ssd<<vcl_endl;

  // free memory used by the manager
  mgr->clean_refine();
}

static void test_refine()
{
  if (test_refine_simple_scene())
    vcl_cout<<"test_refine, simple scene"<<vcl_endl;
}

TESTMAIN(test_refine);
