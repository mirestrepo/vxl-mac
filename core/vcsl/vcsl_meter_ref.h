//*****************************************************************************
// File name: vcsl_meter_ref.h
// Description: Smart pointer on a vcsl_meter
//-----------------------------------------------------------------------------
// Language: C++
//
// Version |Date      | Author                   |Comment
// --------+----------+--------------------------+-----------------------------
// 1.0     |2000/06/28| Fran�ois BERTEL          |Creation
//*****************************************************************************
#ifndef VCSL_METER_REF_H
#define VCSL_METER_REF_H

class vcsl_meter;

//*****************************************************************************
// External declarations for values
//*****************************************************************************
#include <vbl/vbl_smart_ptr.h>

typedef vbl_smart_ptr<vcsl_meter> vcsl_meter_ref;

#endif // #ifndef VCSL_METER_REF_H
