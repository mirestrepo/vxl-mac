#if (defined _MSC_VER) && _MSC_VER == 1200
// disable warning buried deep in the standard library
// warning C4018: '<' : signed/unsigned mismatch: vector(159,163,174)
# pragma warning(disable: 4018)
#endif

#include <vcl_complex.h>
#include <vnl/vnl_vector.h>
#include <vcl_algorithm.txx>
VCL_CONTAINABLE_INSTANTIATE(vnl_vector<vcl_complex<double> >);
