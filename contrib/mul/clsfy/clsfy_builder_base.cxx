
//	Copyright: (C) 2000 Britsh Telecommunications plc

//:
// \file
// \brief Implement bits of an abstract classifier builder
// \author Ian Scott
// \date 2000/05/10
// Modifications
// \verbatim
// 2 May 2001 IMS Converted to VXL
// \endverbatim



//=======================================================================

#include "clsfy_builder_base.h"
#include <vcl_cstdlib.h>
#include <vsl/vsl_indent.h>
#include <vsl/vsl_binary_loader.h>

//=======================================================================

clsfy_builder_base::clsfy_builder_base()
{
}

//=======================================================================

clsfy_builder_base::~clsfy_builder_base()
{
}


//=======================================================================

void vsl_add_to_binary_loader(const clsfy_builder_base& b)
{
  vsl_binary_loader<clsfy_builder_base>::instance().add(b);
}

//=======================================================================

vcl_string clsfy_builder_base::is_a() const
{
  return vcl_string("clsfy_builder_base");
}

//=======================================================================

void vsl_b_write(vsl_b_ostream& os, const clsfy_builder_base& b)
{
  b.b_write(os);
}

//=======================================================================

void vsl_b_read(vsl_b_istream& bfs, clsfy_builder_base& b)
{
  b.b_read(bfs);
}

//=======================================================================

vcl_ostream& operator<<(vcl_ostream& os,const clsfy_builder_base& b)
{
  os << b.is_a() << ": ";
  vsl_indent_inc(os);
  b.print_summary(os);
  vsl_indent_dec(os);
  return os;
}

//=======================================================================

vcl_ostream& operator<<(vcl_ostream& os,const clsfy_builder_base* b)
{
  if (b)	
    return os << *b;
  else			
    return os << "No clsfy_builder_base defined.";
}

