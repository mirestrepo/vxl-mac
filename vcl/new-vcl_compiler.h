//-*- c++ -*-------------------------------------------------------------------
#ifndef vcl_compiler_h_
#define vcl_compiler_h_

// It's much better to determine the compiler automatically here than to depend
// on command-line flags being set.

#if defined(__sgi) && !defined(__GNUC__)
# ifndef _COMPILER_VERSION
#  define VCL_SGI_CC_6
# else
#  if (_COMPILER_VERSION >= 700)
#   define VCL_SGI_CC_7
#  else
#   define VCL_SGI_CC_6
#  endif
#  define VCL_SGI_CC
# endif
#endif

#if defined(__SUNPRO_CC)
# define VCL_SUNPRO_CC
# if (__SUNPRO_CC>=0x500)
#  define VCL_SUNPRO_CC_50
# else
#  undef VCL_SUNPRO_CC_50
# endif
# ifdef INSTANTIATE_TEMPLATES
#  define _RWSTD_COMPILE_INSTANTIATE
# endif
#endif

#if defined(__GNUC__)
# define VCL_GCC
# if (__GNUC__<=1)
#  error "forget it."
# elif (__GNUC__==2)
#  if (__GNUC_MINOR__>=100)
#   error "I need some help here."
#  elif (__GNUC_MINOR__>=95)
#   define VCL_GCC_295
#  elif (__GNUC_MINOR__>8)
#   define VCL_EGCS
#  elif (__GNUC_MINOR__>7)
#   define VCL_GCC_28
#  elif (__GNUC_MINOR__>6)
#   define VCL_GCC_27
#  endif
#  if (__GNUC_MINOR__>7)
#   define VCL_GCC_EGCS // so this is the union of EGCS, GCC_28 and GCC_295
#  endif
# else
#  define VCL_GCC_30
# endif
# if defined (GNU_LIBSTDCXX_V3)
#  define VCL_GCC_WITH_GNU_LIBSTDCXX_V3
# else
#  define VCL_GCC_WITH_GNU_LIBSTDCXX_V2
# endif
#endif

#if defined(_WIN32) || defined(WIN32)
# define VCL_WIN32
# if defined(_MSC_VER)
#  if _MSC_VER >= 1200
#   define VCL_VC60 1
#  else
#   define VCL_VC50 1
#  endif
# endif
#endif

#include <vcl_config.h>

// backwards compatibility.
#define VXL_BIG_ENDIAN                VCL_BIG_ENDIAN
#define VXL_LITTLE_ENDIAN             VCL_LITTLE_ENDIAN
#define VCL_STL_NULL_TMPL_ARGS        VCL_NULL_TMPL_ARGS

// -------------------- default template parameters
#if VCL_CAN_DO_COMPLETE_DEFAULT_TYPE_PARAMETER
# define VCL_DFL_TYPE_PARAM_STLDECL(A, a) class A = a
#else
# define VCL_DFL_TYPE_PARAM_STLDECL(A, a) class A
#endif

#if VCL_CAN_DO_TEMPLATE_DEFAULT_TYPE_PARAMETER
# define VCL_DFL_TMPL_PARAM_STLDECL(A, a) class A = a
#else
# define VCL_DFL_TMPL_PARAM_STLDECL(A, a) class A
#endif

#define VCL_DFL_TMPL_ARG(classname) , classname

#if VCL_USE_NATIVE_STL
# define VCL_SUNPRO_ALLOCATOR_HACK(T) T VCL_SUNPRO_CLASS_SCOPE_HACK(std::allocator<T >)
#else
# define VCL_SUNPRO_ALLOCATOR_HACK(T) T // FIXIT
#endif

// -------------------- template instantiation
#undef VCL_INSTANTIATE_INLINE
#undef VCL_UNINSTANTIATE_SPECIALIZATION
#undef VCL_UNINSTANTIATE_UNSEEN_SPECIALIZATION
#undef VCL_INSTANTIATE_STATIC_TEMPLATE_MEMBER
#undef VCL_UNINSTANTIATE_STATIC_TEMPLATE_MEMBER

// -----------------------------------------------------------------------------

// work-around to get template instantiation to work correctly with SunPro
// check flag to turn on inlining
#undef IUEi_STL_INLINE
#if defined(INLINE_EXPLICIT_FLAG) && defined(VCL_SUNPRO_CC) && defined(INSTANTIATE_TEMPLATES)
# define IUEi_STL_INLINE
#else
# define IUEi_STL_INLINE inline
#endif

// gcc
#ifdef VCL_GCC_27
#define VCL_INSTANTIATE_INLINE(fn_decl) template fn_decl ;
#define VCL_UNINSTANTIATE_SPECIALIZATION(x)
#define VCL_UNINSTANTIATE_UNSEEN_SPECIALIZATION(x) extern x;
#define VCL_INSTANTIATE_STATIC_TEMPLATE_MEMBER(x) x;
#define VCL_UNINSTANTIATE_STATIC_TEMPLATE_MEMBER(x)
#endif

// egcs
#if defined(VCL_EGCS)
#define VCL_INSTANTIATE_INLINE(fn_decl) 
#define VCL_UNINSTANTIATE_SPECIALIZATION(x)
#define VCL_UNINSTANTIATE_UNSEEN_SPECIALIZATION(x)
#define VCL_INSTANTIATE_STATIC_TEMPLATE_MEMBER(x) x;
#define VCL_UNINSTANTIATE_STATIC_TEMPLATE_MEMBER(x)
#endif

// gcc 2.8.x
#if defined(VCL_GCC_28)
#define VCL_INSTANTIATE_INLINE(fn_decl) 
#define VCL_UNINSTANTIATE_SPECIALIZATION(x)
#define VCL_UNINSTANTIATE_UNSEEN_SPECIALIZATION(x)
#define VCL_INSTANTIATE_STATIC_TEMPLATE_MEMBER(x) x;
#define VCL_UNINSTANTIATE_STATIC_TEMPLATE_MEMBER(x)
#endif

// gcc 2.95
#if defined(VCL_GCC_295)
#define VCL_INSTANTIATE_INLINE(fn_decl) 
#define VCL_UNINSTANTIATE_SPECIALIZATION(x)
#define VCL_UNINSTANTIATE_UNSEEN_SPECIALIZATION(x)
#define VCL_INSTANTIATE_STATIC_TEMPLATE_MEMBER(x) x;
#define VCL_UNINSTANTIATE_STATIC_TEMPLATE_MEMBER(x)
#endif

// windows (vc50)
#ifdef VCL_WIN32
#define VCL_INSTANTIATE_INLINE(fn_decl) template fn_decl ;
#define VCL_UNINSTANTIATE_SPECIALIZATION(x)
#define VCL_UNINSTANTIATE_UNSEEN_SPECIALIZATION(x)
#define VCL_INSTANTIATE_STATIC_TEMPLATE_MEMBER(x) x;
#define VCL_UNINSTANTIATE_STATIC_TEMPLATE_MEMBER(x)
#endif

// native sgi < 7
#if defined(VCL_SGI_CC_6)
#define VCL_INSTANTIATE_INLINE(fn_decl)
#define VCL_UNINSTANTIATE_SPECIALIZATION(text) @pragma do_not_instantiate text@
#define VCL_UNINSTANTIATE_UNSEEN_SPECIALIZATION(x) @pragma do_not_instantiate x@
#define VCL_UNINSTANTIATE_STATIC_TEMPLATE_MEMBER(x) @pragma do_not_instantiate x@
#define VCL_INSTANTIATE_STATIC_TEMPLATE_MEMBER(x) x;
#endif

// native sgi 7
#ifdef VCL_SGI_CC_7
#define VCL_INSTANTIATE_INLINE(fn_decl)
#define VCL_UNINSTANTIATE_SPECIALIZATION(text)
#define VCL_UNINSTANTIATE_UNSEEN_SPECIALIZATION(x) @pragma do_not_instantiate x@
#define VCL_UNINSTANTIATE_STATIC_TEMPLATE_MEMBER(x) @pragma do_not_instantiate x@
#define VCL_INSTANTIATE_STATIC_TEMPLATE_MEMBER(x) x;
#endif

// SunPro < 5 (eg 4.2)
#if defined(VCL_SUNPRO_CC) && !defined(VCL_SUNPRO_CC_50)
#define VCL_INSTANTIATE_INLINE(fn_decl) 
#define VCL_INSTANTIATE_STATIC_TEMPLATE_MEMBER(x) x; 
#define VCL_UNINSTANTIATE_STATIC_TEMPLATE_MEMBER(x)
#define VCL_UNINSTANTIATE_SPECIALIZATION(x)
#define VCL_UNINSTANTIATE_UNSEEN_SPECIALIZATION(x)
#endif

// SunPro 5.0
#if defined(VCL_SUNPRO_CC_50)
#define VCL_INSTANTIATE_INLINE(fn_decl) 
#define VCL_INSTANTIATE_STATIC_TEMPLATE_MEMBER(x) x; 
#define VCL_UNINSTANTIATE_STATIC_TEMPLATE_MEMBER(x)
#define VCL_UNINSTANTIATE_SPECIALIZATION(x)
#define VCL_UNINSTANTIATE_UNSEEN_SPECIALIZATION(x)
#endif

//--------------------------------------------------------------------------------

// Decide at configuration time whether you want to use double instead
// of long double. On most machines it's too slow, but the default is
// to use it because turning it off by default would be weird.
#if VCL_USE_LONG_DOUBLE
typedef long double VCL_long_double;
#else
typedef double VCL_long_double;
#endif

#if VCL_FOR_SCOPE_HACK
# undef for
# define for if (false) { } else for
typedef int saw_VCL_FOR_SCOPE_HACK;
#endif

// fix to instantiate template functions
#define VCL_INSTANTIATE_NONINLINE(fn_decl) template fn_decl ;

#endif
