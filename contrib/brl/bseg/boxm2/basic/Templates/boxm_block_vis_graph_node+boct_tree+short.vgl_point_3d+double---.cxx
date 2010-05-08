#include <boxm2/basic/boxm_block_vis_graph_node.h>
#include <boct/boct_tree.txx>
#include <vgl/io/vgl_io_point_3d.h>

typedef boct_tree<short, vgl_point_3d<double> > tree_type;
BOXM2_BLOCK_VIS_GRAPH_NODE_INSTANTIATE(tree_type);
