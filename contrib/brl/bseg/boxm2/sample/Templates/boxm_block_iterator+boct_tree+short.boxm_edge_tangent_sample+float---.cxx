#include <boxm2/boxm_scene.txx>
#include <boxm2/sample/boxm_edge_tangent_sample.h>
#include <boxm2/sample/boxm_inf_line_sample.h>
#include <boct/boct_tree.h>

typedef boct_tree<short,boxm_edge_tangent_sample<float> >  aux_tree_type;
BOXM_BLOCK_ITERATOR_INSTANTIATE(aux_tree_type);

typedef boct_tree<short,boxm_inf_line_sample<float> >  tree_type;
BOXM_BLOCK_ITERATOR_INSTANTIATE(tree_type);
