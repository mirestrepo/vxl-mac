#ifndef vbl_sparse_array_3d_txx_
#define vbl_sparse_array_3d_txx_

// This is vxl/vbl/vbl_sparse_array_3d.txx


#include "vbl_sparse_array_3d.h"
#include "vbl_sparse_array_base.txx"
#include <vcl_iostream.h>


//: Print the array to a stream in "(i,j,k): value" format.
template <class T>
vcl_ostream& vbl_sparse_array_3d<T>::print(vcl_ostream& out) const
{
  for(const_iterator p = storage_.begin(); p != storage_.end(); ++p)
    out << "(" << (*p).first.i
        << "," << (*p).first.j
        << "," << (*p).first.k
        << "): " << (*p).second << vcl_endl;
  return out;
}

#undef VBL_SPARSE_ARRAY_3D_INSTANTIATE
#define VBL_SPARSE_ARRAY_3D_INSTANTIATE(T) \
VBL_SPARSE_ARRAY_BASE_INSTANTIATE(T, vbl_index_3d); \
template class vbl_sparse_array_3d<T >; \
VCL_INSTANTIATE_INLINE(vcl_ostream& operator<< (vcl_ostream&, const vbl_sparse_array_3d<T > &))

#endif
