// This is contrib/brl/bbas/bxml/bxml_read.h
#ifndef bxml_read_h_
#define bxml_read_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \brief functions to parse XML documents
// \author Matt Leotta (Brown)
// \date   October 5, 2006
//
// \verbatim
//  Modifications
// \endverbatim
//

#include "bxml_document.h"
#include <vcl_iostream.h>


//: Read the entire contents of \p filepath into an XML document class
bxml_document bxml_read(const vcl_string& filepath);


//: Read the entire data stream \p is into an XML document class
bxml_document bxml_read(vcl_istream& is);


//: Read an XML stream one element at a time
class bxml_stream_read
{
  public:
    //: Constructor
    // only elements with depth <= max_depth are returned
    explicit bxml_stream_read(int max_depth = -1);

    //: Destructor
    ~bxml_stream_read();

    //: Reset the state of the reader
    void reset();

    //: Read the next element
    bxml_data_sptr next_element(vcl_istream& is, unsigned int& depth);

  private:
    //: Private implementation
    class pimpl;
    pimpl* p_;

};

#endif // bxml_read_h_
