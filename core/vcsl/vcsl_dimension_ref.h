//*****************************************************************************
// File name: vcsl_dimension_ref.h
// Description: Smart pointer on a vcsl_dimension
//-----------------------------------------------------------------------------
// Language: C++
//
// Version |Date      | Author                   |Comment
// --------+----------+--------------------------+-----------------------------
// 1.0     |2000/06/28| Fran�ois BERTEL          |Creation
//*****************************************************************************
#ifndef VCSL_DIMENSION_REF_H
#define VCSL_DIMENSION_REF_H

class vcsl_dimension;

//*****************************************************************************
// External declarations for values
//*****************************************************************************
#include <vbl/vbl_smart_ptr.h>

typedef vbl_smart_ptr<vcsl_dimension> vcsl_dimension_ref;

#endif // #ifndef VCSL_DIMENSION_REF_H
