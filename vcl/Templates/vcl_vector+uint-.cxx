// OK, really big hack, if the compiler sees
// bvector, then it causes undefined symbols of the type
// bvec_iterator, so we hide the bvector.h file from the compiler,
// and it does not seem to need it, as it shouldn't since we don't
// want a bvec here!
#define __SGI_STL_BVECTOR_H
#include <vcl_algorithm.txx>
#include <vcl_vector.txx>

VCL_VECTOR_INSTANTIATE(unsigned int);
VCL_SWAP_INSTANTIATE(unsigned int);
#ifdef VCL_GCC_295
template void fill(unsigned*, unsigned*, int const&);
#endif
#if defined (__osf__) && defined(VCL_GCC_295) && !defined(GNU_LIBSTDCXX_V3)
template unsigned int* fill_n(unsigned int*, unsigned int, unsigned int const &);
#endif

#if defined(GNU_LIBSTDCXX_V3)
// In V3, vector<T>::iterator is not a T *.
VCL_FIND_INSTANTIATE(vcl_vector<unsigned>::iterator, unsigned);
VCL_FIND_INSTANTIATE(vcl_vector<unsigned>::const_iterator, unsigned);
#endif
