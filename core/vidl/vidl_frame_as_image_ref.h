//*****************************************************************************
// File name: vidl_frame_as_image_ref.h
// Description: A smart pointer on a vidl_frame_as_image
//-----------------------------------------------------------------------------
// Language: C++
//
// Version |Date      | Author                   |Comment
// --------+----------+--------------------------+-----------------------------
// 1.0     |2000/05/26| Julien ESTEVE            |Creation
//*****************************************************************************
#ifndef vidl_frame_as_image_ref_h
#define vidl_frame_as_image_ref_h

//
// typedef for class vbl_smart_ptr<vidl_frame_as_image>
// Include this file to use the smart pointer vidl_frame_as_image_ref
//

class vidl_frame_as_image;

#include <vbl/vbl_smart_ptr.h>

typedef vbl_smart_ptr<vidl_frame_as_image> vidl_frame_as_image_ref;

#endif // ifndef vidl_frame_as_image_ref_h

