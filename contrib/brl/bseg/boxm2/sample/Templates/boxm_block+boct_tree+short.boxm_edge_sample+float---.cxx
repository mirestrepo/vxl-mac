#include <boxm2/boxm_block.txx>
#include <boct/boct_tree.h>
#include <boxm2/sample/boxm_edge_sample.h>

typedef boct_tree<short,boxm_edge_sample<float> > tree;
BOXM2_BLOCK_INSTANTIATE(tree);

typedef boct_tree<short,boxm_aux_edge_sample<float> > aux_tree;
BOXM2_BLOCK_INSTANTIATE(aux_tree);

#include <boxm2/boxm_scene.txx>
BOXM2_BLOCK_ITERATOR_INSTANTIATE(tree);
BOXM2_BLOCK_ITERATOR_INSTANTIATE(aux_tree);
