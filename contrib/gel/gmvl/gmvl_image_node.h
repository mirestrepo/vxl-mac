// This is gel/gmvl/gmvl_image_node.h
#ifndef gmvl_image_node_h_
#define gmvl_image_node_h_
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma interface
#endif
//:
// \file
// \author crossge@crd.ge.com

#include <vil1/vil1_image.h>
#include <gmvl/gmvl_node.h>

class gmvl_image_node : public gmvl_node
{
 public:

  gmvl_image_node( vil1_image image);
  ~gmvl_image_node();

  vil1_image image() const { return image_; }

 protected:

  vil1_image image_;
};

#endif // gmvl_image_node_h_
