// This is mul/mil/mil_image.h
#ifndef mil_image_h_
#define mil_image_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \brief A base class for images of any dimension and type
// \author Tim Cootes

#include <vcl_string.h>
#include <vcl_iosfwd.h>
#include <vsl/vsl_binary_io.h>

//: A base class for images of any dimension and type
//  Derived classes tend to have world to image transformations
//  attached to them, and to be able to act as `views' of
//  external data.
//
//  The existing vil1_image does not suffice in this respect.
class mil_image
{
 public:

    //: Dflt ctor
    mil_image() {}

    //: Destructor
    virtual ~mil_image() {}

    //: Number of dimensions of image
    virtual int n_dims() const = 0;

    //: Width of image
    virtual int nx() const = 0;

    //: Height of image
    virtual int ny() const =0;

    //: Depth of image (set to 1 for 2D images)
    virtual int nz() const =0;

    //: Number of planes of image
    virtual int n_planes() const = 0;

    //: Version number for I/O
    short version_no() const;

    //: Name of the class
    virtual vcl_string is_a() const = 0;

    //: Does the name of the class match the argument?
    virtual bool is_class(vcl_string const&) const = 0;

    //: Create a copy on the heap and return base class pointer
    virtual mil_image* clone() const = 0;

    //: Print class to os
    virtual void print_summary(vcl_ostream& os) const = 0;

    //: Print whole image to os
    virtual void print_all(vcl_ostream& os) const = 0;

    //: Save class to binary file stream
    virtual void b_write(vsl_b_ostream& bfs) const = 0;

    //: Load class from binary file stream
    virtual void b_read(vsl_b_istream& bfs) = 0;
};

//: Allows derived class to be loaded by base-class pointer
//  A loader object exists which is invoked by calls
//  of the form "vsl_b_read(bfs,base_ptr);".  This loads derived class
//  objects from the disk, places them on the heap and
//  returns a base class pointer.
//  In order to work the loader object requires
//  an instance of each derived class that might be
//  found.  This function gives the model class to
//  the appropriate loader.
void vsl_add_to_binary_loader(const mil_image& b);

//: Binary file stream output operator for class reference
void vsl_b_write(vsl_b_ostream& bfs, const mil_image& b);

//: Binary file stream input operator for class reference
void vsl_b_read(vsl_b_istream& bfs, mil_image& b);

//: Stream output operator for class reference
vcl_ostream& operator<<(vcl_ostream& os,const mil_image& b);

//: Stream output operator for class pointer
vcl_ostream& operator<<(vcl_ostream& os,const mil_image* b);

//: Print class to os
void vsl_print_summary(vcl_ostream& os, const mil_image& im);

#endif // mil_image_h_
