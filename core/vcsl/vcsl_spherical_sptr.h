//*****************************************************************************
// File name: vcsl_spherical_sptr.h
// Description: Smart pointer on a vcsl_spherical
//-----------------------------------------------------------------------------
// Language: C++
//
// Version |Date      | Author                   |Comment
// --------+----------+--------------------------+-----------------------------
// 1.0     |2000/06/28| Fran�ois BERTEL          |Creation
//*****************************************************************************
#ifndef VCSL_SPHERICAL_REF_H
#define VCSL_SPHERICAL_REF_H

class vcsl_spherical;

//*****************************************************************************
// External declarations for values
//*****************************************************************************
#include <vbl/vbl_smart_ptr.h>

typedef vbl_smart_ptr<vcsl_spherical> vcsl_spherical_sptr;

#endif // #ifndef VCSL_SPHERICAL_REF_H
