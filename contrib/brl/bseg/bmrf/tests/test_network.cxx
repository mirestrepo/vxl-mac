//:
// \file
#include <testlib/testlib_test.h>
#include <vsl/vsl_binary_io.h>
#include <vbl/io/vbl_io_smart_ptr.h>
#include <vpl/vpl.h>

#include <bmrf/bmrf_epi_seg.h>
#include <bmrf/bmrf_node.h>
#include <bmrf/bmrf_network.h>

//: Test the network class
void test_network()
{
  bmrf_node_sptr node_1 = new bmrf_node(new bmrf_epi_seg, 1, 0.1);
  bmrf_node_sptr node_2 = new bmrf_node(new bmrf_epi_seg, 1, 0.2);
  bmrf_node_sptr node_3 = new bmrf_node(new bmrf_epi_seg, 2, 0.3);
  bmrf_node_sptr node_4 = new bmrf_node(new bmrf_epi_seg, 2, 0.4);
  bmrf_node_sptr node_5 = new bmrf_node(new bmrf_epi_seg, 3, 0.5);

  bmrf_node_sptr bad_node = new bmrf_node(NULL, 5, 0.0);

  bmrf_network_sptr the_network = new bmrf_network;

  TEST("Testing add_node()",
       the_network->add_node(node_1) &&
       the_network->add_node(node_2) &&
       the_network->add_node(node_3) &&
       the_network->add_node(node_4) &&
       the_network->add_node(node_5) &&
       !the_network->add_node(node_5) && // can't add a node twice
       !the_network->add_node(bad_node), // can't add a node with no segment
       true);
  
  TEST("Testing remove_node()",
       the_network->remove_node(node_3) &&
       the_network->remove_node(node_4) &&
       !the_network->remove_node(node_4) && // can't remove a node not in the network
       !the_network->remove_node(bad_node), // can't remove a node with no segment
       true);

  // make the arcs
  TEST("Testing add_arc()",     
       the_network->add_arc(node_1, node_2, bmrf_node::ALPHA) &&
       the_network->add_arc(node_2, node_1, bmrf_node::ALPHA) &&
       the_network->add_arc(node_1, node_3, bmrf_node::TIME) && // node_3 added back in
       the_network->add_arc(node_4, node_2, bmrf_node::TIME) && // node_4 added back in
       the_network->add_arc(node_2, node_4, bmrf_node::TIME) &&
       the_network->add_arc(node_4, node_3, bmrf_node::SPACE) &&
       the_network->add_arc(node_4, node_5, bmrf_node::TIME) &&
       the_network->add_arc(node_3, node_5, bmrf_node::SPACE) &&
       the_network->add_arc(node_3, node_5, bmrf_node::ALPHA) &&
       the_network->add_arc(node_5, node_3, bmrf_node::TIME) &&
       !the_network->add_arc(node_5, node_3, bmrf_node::TIME) && // can't add the same arc twice
       !the_network->add_arc(node_1, node_1, bmrf_node::ALPHA), // can't arc to self
       true);

  // remove arcs
  TEST("Testing remove_arc()",
       the_network->remove_arc(node_3, node_5, bmrf_node::ALPHA) &&
       the_network->remove_arc(node_3, node_5) &&
       !the_network->remove_arc(node_3, node_5), // can't remove an arc not in the graph
       true);


//----------------------------------------------------------------------------------------
// I/O Tests
//----------------------------------------------------------------------------------------

  // binary test output file stream
  vsl_b_ofstream bfs_out("test_network_io.tmp");
  TEST("Created test_network_io.tmp for writing",(!bfs_out), false);
  vsl_b_write(bfs_out, the_network);
  bfs_out.close();

  bmrf_network_sptr network_in;

  // binary test input file stream
  vsl_b_ifstream bfs_in("test_network_io.tmp");
  TEST("Opened test_network_io.tmp for reading",(!bfs_in), false);
  vsl_b_read(bfs_in, network_in);
  bfs_in.close();

  // remove the temporary file
  vpl_unlink ("test_node_io.tmp");
}


MAIN( test_network )
{
  START( "bmrf_network" );
  test_network();
  SUMMARY();
}
