//*****************************************************************************
// File name: vtol_face_2d_sptr.h
// Description: A smart pointer on a vtol_face_2d
//-----------------------------------------------------------------------------
// Language: C++
//
// Version |Date      | Author                   |Comment
// --------+----------+--------------------------+-----------------------------
// 1.0     |2000/05/15| Fran�ois BERTEL          |Creation
//*****************************************************************************
#ifndef VTOL_FACE_2D_REF_H
#define VTOL_FACE_2D_REF_H

class vtol_face_2d;

//*****************************************************************************
// External declarations for values
//*****************************************************************************
#include <vbl/vbl_smart_ptr.h>

typedef vbl_smart_ptr<vtol_face_2d> vtol_face_2d_sptr;

#endif // #ifndef VTOL_FACE_2D_REF_H
