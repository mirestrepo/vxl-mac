#ifdef __GNUC__
#pragma implementation
#endif

//: \file
//  \brief Class to load and save images from named files
//  \author Tim Cootes

#include <vcl_cstdlib.h>
#include <mil/mil_image_pyramid_builder.h>
#include <vsl/vsl_indent.h>
#include <vsl/vsl_binary_loader.h>

//=======================================================================
// Dflt ctor
//=======================================================================

mil_image_pyramid_builder::mil_image_pyramid_builder()
{
}

//=======================================================================
// Destructor
//=======================================================================

mil_image_pyramid_builder::~mil_image_pyramid_builder()
{
}

//=======================================================================
// Method: version_no
//=======================================================================

short mil_image_pyramid_builder::version_no() const 
{ 
	return 1; 
}

//=======================================================================
// Method: vsl_add_to_binary_loader
//=======================================================================

void vsl_add_to_binary_loader(const mil_image_pyramid_builder& b)
{
	vsl_binary_loader<mil_image_pyramid_builder>::instance().add(b);
}

//=======================================================================
// Method: is_a
//=======================================================================

vcl_string mil_image_pyramid_builder::is_a() const
{
  return vcl_string("mil_image_pyramid_builder");
}

//=======================================================================
// Associated function: operator<< 
//=======================================================================

void vsl_b_write(vsl_b_ostream& bfs, const mil_image_pyramid_builder* b)
{
    if (b)	
	{
		vsl_b_write(bfs,b->is_a());
		b->b_write(bfs);
	}
    else	
		vsl_b_write(bfs,vcl_string("VSL_NULL_PTR"));

    
}

//=======================================================================
// Associated function: operator<< 
//=======================================================================

void vsl_b_write(vsl_b_ostream& bfs, const mil_image_pyramid_builder& b)
{
    b.b_write(bfs);
    
}

//=======================================================================
// Associated function: operator>> 
//=======================================================================

void vsl_b_read(vsl_b_istream& bfs, mil_image_pyramid_builder& b)
{
    b.b_read(bfs);
    
}

//=======================================================================
// Associated function: operator<< 
//=======================================================================

vcl_ostream& operator<<(vcl_ostream& os,const mil_image_pyramid_builder& b)
{
	os << b.is_a() << ": ";
	vsl_inc_indent(os);
	b.print_summary(os);
	vsl_dec_indent(os);
	return os;
}

//=======================================================================
// Associated function: operator<< 
//=======================================================================

vcl_ostream& operator<<(vcl_ostream& os,const mil_image_pyramid_builder* b)
{
    if (b)	
		return os << *b;
    else			
		return os << "No mil_image_pyramid_builder defined.";
}
