#ifndef vbl_sparse_array_base_txx_
#define vbl_sparse_array_base_txx_

// This is vxl/vbl/vbl_sparse_array_base.txx

//:
// \file
// \brief Contains a base class for sparse arrays.
// \author: Ian Scott

#include "vbl_sparse_array_base.h"

#include <vcl_cassert.h>
#include <vcl_iostream.h>
#include <vcl_utility.h>

//: DEPRECATED Return contents of (i).  Assertion failure if not yet filled.
// Use operator () instead.
template <class T, class Index>
T const & vbl_sparse_array_base<T, Index>::operator [] (Index i) const
{
  Map::const_iterator p = storage_.find(i);
  
  assert(p != storage_.end());

  return (*p).second;
}


//: Empty the sparse matrix.
template <class T, class Index>
void vbl_sparse_array_base<T, Index>::clear()
{
  storage_.clear();
}


//: Return contents of (i).  Assertion failure if not yet filled.
template <class T, class Index>
T const & vbl_sparse_array_base<T, Index>::operator () (Index i) const
{
  Map::const_iterator p = storage_.find(i);
  
  assert(p != storage_.end());

  return (*p).second;
}

//: Return the memory address of location (i).  0 if not yet filled.
template <class T, class Index>
T* vbl_sparse_array_base<T, Index>::get_addr(Index i)
{
  Map::iterator p = storage_.find(i);

  if (p == storage_.end())
    return 0;

  return &(*p).second;
}

//: Return true if location (i) has been filled.
template <class T, class Index>
bool vbl_sparse_array_base<T, Index>::fullp(Index i) const
{
  return storage_.find(i) != storage_.end();
}

//: Put a value into location (i).
template <class T, class Index>
bool vbl_sparse_array_base<T, Index>::put(Index i, const T& t)
{
  vcl_pair<Map::iterator,bool> res = storage_.insert(Map::value_type(i,t));

  return res.second;
} 


#undef VBL_SPARSE_ARRAY_BASE_INSTANTIATE
#define VBL_SPARSE_ARRAY_BASE_INSTANTIATE(T, I)\
template class vbl_sparse_array_base<T , I >


#endif
