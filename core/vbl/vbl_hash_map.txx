// This is vxl/vbl/vbl_hash_map.txx

// -*- c++ -*-
#ifndef vbl_hash_map_txx_
#define vbl_hash_map_txx_

#include <vbl/vbl_hash_map.h>

#ifndef WIN32
#undef VBL_HASHTABLE_INSTANTIATE
#define VBL_HASHTABLE_INSTANTIATE(K, T) \
template class vbl_hash_map<K, T >;
#else
#undef VBL_HASHTABLE_INSTANTIATE
#define VBL_HASHTABLE_INSTANTIATE(K, T) 
#endif

#endif
