#include <vcl/vcl_compiler.h>
#if !VCL_USE_NATIVE_STL

#define __PUT_STATIC_DATA_MEMBERS_HERE

#include "vcl_list.h"
#include "vcl_map.h"
#include "vcl_multimap.h"
#include "vcl_set.h"
#include "vcl_multiset.h"
#include "vcl_tree.h"
// #include "vcl_stack.h"
#include "vcl_algorithm.h"
#include "vcl_alloc.h"

// STL
template class __malloc_alloc<0>;

#ifdef __STL_USE_NEWALLOC
template class __new_alloc<0>;
#else
#ifndef __STL_USE_MALLOC
template class __alloc<false, 0>;
template class __alloc<true, 0>;
#endif
#endif

#include <vcl/vcl_iostream.h>
void vcl_alloc_throw_bad_alloc(char const *FILE, int LINE)
{
  cerr << FILE << ":" << LINE << " : out of memory" << endl;
  exit(1);
}

#endif
