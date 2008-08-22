// This is core/vil/algo/vil_suppress_non_max_edges.h
#ifndef vil_suppress_non_max_edges_h_
#define vil_suppress_non_max_edges_h_
//:
// \file
// \brief Given gradient image, compute magnitude and zero any non-maximal values
// \author Tim Cootes
//
// \verbatim
//  Modifications
//  Matt Leotta  -  8/22/08  -  Added a parabolic subpixel version
// \endverbatim

#include <vil/vil_image_view.h>

//: Given gradient images, computes magnitude image containing maximal edges
//  Computes magnitude image.  Zeros any below a threshold.
//  Points with magnitude above a threshold are tested against gradient
//  along normal to the edge and retained only if they are higher than
//  their neighbours.
//
//  Note: Currently assumes single plane only.
//  2 pixel border around output set to zero.
//  If two neighbouring edges have exactly the same strength, it retains
//  both (ie an edge is eliminated if it is strictly lower than a neighbour,
//  but not if it is the same as two neighbours).
template<class srcT, class destT>
void vil_suppress_non_max_edges(const vil_image_view<srcT>& grad_i,
                                const vil_image_view<srcT>& grad_j,
                                double grad_mag_threshold,
                                vil_image_view<destT>& grad_mag);


//: Given gradient images, computes magnitude image of maximal subpixel edges
//  Computes magnitude image.  Zeros any below a threshold.
//  Points with magnitude above a threshold are tested against gradient
//  along normal to the edge and retained only if they are higher than
//  their neighbours.  The magnitude of retained points is revised using
//  parabolic interpolation in the normal direction.  The same interpolation
//  provides a subpixel offset from the integral pixel location.
//
//  Note: Currently assumes single plane only.
//  2 pixel border around output set to zero.
//  If two neighbouring edges have exactly the same strength, it retains
//  both (ie an edge is eliminated if it is strictly lower than a neighbour,
//  but not if it is the same as two neighbours).
template<class srcT, class destT>
void vil_suppress_non_max_edges_subpixel(const vil_image_view<srcT>& grad_i,
                                         const vil_image_view<srcT>& grad_j,
                                         double grad_mag_threshold,
                                         vil_image_view<destT>& grad_mag_offset);

#endif // vil_suppress_non_max_edges_h_
