// This avoids automatic instantiation of other smart pointers:
#define vsol_box_3d_sptr_h
class vsol_box_3d_sptr {};

#include <vsol/vsol_spatial_object_3d.h>
#include <vbl/vbl_smart_ptr.txx>

VBL_SMART_PTR_INSTANTIATE(vsol_spatial_object_3d);
