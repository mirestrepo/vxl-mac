//*****************************************************************************
// File name: vidl_avicodec_sptr.h
// Description: A smart pointer on a vidl_avicodec
//-----------------------------------------------------------------------------
// Language: C++
//
// Version |Date      | Author                   |Comment
// --------+----------+--------------------------+-----------------------------
// 1.0     |2000/09/06| Julien ESTEVE            |Creation
//*****************************************************************************
#ifndef VIDL_AVICODEC_REF_H
#define VIDL_AVICODEC_REF_H

//
// typedef for class vbl_smart_ptr<vidl_avicodec>
// Include this file to use the smart pointer vidl_avicodec_sptr
//

class vidl_avicodec;

#include <vbl/vbl_smart_ptr.h>

typedef vbl_smart_ptr<vidl_avicodec> vidl_avicodec_sptr;

#endif // ifndef VIDL_AVICODEC_REF_H
