// This is ./vxl/vul/vul_root_dir.cxx
#include <vul/vul_root_dir.h>
#include <vcl_cstdlib.h>
#include <vcl_iostream.h>

//:
// \file
//
// The following should have been created automatically by the
// configuration scripts from vcl_where_root_dir.h.in
// If it isn't we need to check for its existence and do something else.
#ifdef VCL_WHERE_ROOT_DIR_H_EXISTS
#include <vcl_where_root_dir.h>
//: Return source root directory (ie the one just below vcl).
vcl_string vul_root_dir()
{
  return vcl_string(VCL_SOURCE_ROOT_DIR);
}
#else
//: Return source root directory (ie the one just below vcl and vxl).
vcl_string vul_root_dir()
{
  char* ptr;

  ptr= vcl_getenv("VXLSRC");
  if (ptr)
    return vcl_string(ptr);

  ptr= vcl_getenv("VCLSRC");
  if (ptr)
    return vcl_string(ptr);

  ptr= vcl_getenv("VXL_SRC");
  if (ptr)
    return vcl_string(ptr);

  vcl_cerr<<"ERROR: vul_root_dir() Unable to retrieve directory from "<<vcl_endl;
  vcl_cerr<<"$VCLSRC or $VXLSRC or $VXL_SRC.  Sorry."<<vcl_endl;
  return vcl_string("");
}

#endif
