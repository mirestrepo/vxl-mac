//*****************************************************************************
// File name: vsol_curve_2d_ref.h
// Description: A smart pointer on a vsol_curve_2d
//-----------------------------------------------------------------------------
// Language: C++
//
// Version |Date      | Author                   |Comment
// --------+----------+--------------------------+-----------------------------
// 1.0     |2000/04/27| Fran�ois BERTEL          |Creation
//*****************************************************************************
#ifndef VSOL_CURVE_2D_REF_H
#define VSOL_CURVE_2D_REF_H

class vsol_curve_2d;

//*****************************************************************************
// External declarations for values
//*****************************************************************************
#include <vbl/vbl_smart_ptr.h>

typedef vbl_smart_ptr<vsol_curve_2d> vsol_curve_2d_ref;

#endif // #ifndef VSOL_CURVE_2D_REF_H
