#ifndef mil_wrappers_h_
#define mil_wrappers_h_
//:
// \file
// \brief Various wrappers to help convert between image types.
// \author Tim Cootes

#include <mil/mil_image_2d_of.h>
#include <vimt/vimt_image_2d_of.h>
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h> // for M.get_row()

//: Create a vimt image that wraps a mil image - should only be used read only.
//  Sorry about breaking the naming conventions, but I don't want this in vimt
template <class T>
inline vimt_image_2d_of<T> vimt_wrap_mil_image(const mil_image_2d_of<T>& im)
{
  // Set transform, assuming it is no more complex than affine
  vimt_transform_2d vt;
  vnl_matrix<double> M;
  im.world2im().matrix(M);
  vnl_matrix<double> A(2,3);
  A.set_row(0,M.get_row(0));
  A.set_row(1,M.get_row(1));
  vt.set_affine(A);

  vimt_image_2d_of<T> vim;
  vim.set_world2im(vt);
  int planestep=0;
  if (im.n_planes()>1) planestep=im.plane(1)-im.plane(0);
  vim.image().set_to_memory(im.plane(0),im.nx(),im.ny(),im.n_planes(),
                            im.xstep(),im.ystep(),planestep);
  return vim;
}

#endif // mil_wrappers_h_
