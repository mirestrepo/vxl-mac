// This is mul/mil/mil_image_pyramid_builder.cxx
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma implementation
#endif
//:
//  \file
//  \brief Class to load and save images from named files
//  \author Tim Cootes

#include "mil_image_pyramid_builder.h"
#include <vsl/vsl_indent.h>
#include <vsl/vsl_binary_loader.h>

//=======================================================================

mil_image_pyramid_builder::mil_image_pyramid_builder()
{
}

//=======================================================================

mil_image_pyramid_builder::~mil_image_pyramid_builder()
{
}

//=======================================================================

short mil_image_pyramid_builder::version_no() const
{
    return 1;
}

//=======================================================================

void vsl_add_to_binary_loader(const mil_image_pyramid_builder& b)
{
    vsl_binary_loader<mil_image_pyramid_builder>::instance().add(b);
}

//=======================================================================

vcl_string mil_image_pyramid_builder::is_a() const
{
  return vcl_string("mil_image_pyramid_builder");
}

//=======================================================================

bool mil_image_pyramid_builder::is_class(vcl_string const& s) const
{
  return s==mil_image_pyramid_builder::is_a();
}

//=======================================================================

void vsl_b_write(vsl_b_ostream& bfs, const mil_image_pyramid_builder& b)
{
    b.b_write(bfs);
}

//=======================================================================

void vsl_b_read(vsl_b_istream& bfs, mil_image_pyramid_builder& b)
{
    b.b_read(bfs);
}

//=======================================================================

vcl_ostream& operator<<(vcl_ostream& os,const mil_image_pyramid_builder& b)
{
    os << b.is_a() << ": ";
    vsl_indent_inc(os);
    b.print_summary(os);
    vsl_indent_dec(os);
    return os;
}

//=======================================================================

vcl_ostream& operator<<(vcl_ostream& os,const mil_image_pyramid_builder* b)
{
    if (b)
        return os << *b;
    else
        return os << "No mil_image_pyramid_builder defined.";
}
