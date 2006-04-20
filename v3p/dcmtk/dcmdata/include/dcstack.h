/*
 *
 *  Copyright (C) 1994-2001, OFFIS
 *
 *  This software and supporting documentation were developed by
 *
 *    Kuratorium OFFIS e.V.
 *    Healthcare Information and Communication Systems
 *    Escherweg 2
 *    D-26121 Oldenburg, Germany
 *
 *  THIS SOFTWARE IS MADE AVAILABLE,  AS IS,  AND OFFIS MAKES NO  WARRANTY
 *  REGARDING  THE  SOFTWARE,  ITS  PERFORMANCE,  ITS  MERCHANTABILITY  OR
 *  FITNESS FOR ANY PARTICULAR USE, FREEDOM FROM ANY COMPUTER DISEASES  OR
 *  ITS CONFORMITY TO ANY SPECIFICATION. THE ENTIRE RISK AS TO QUALITY AND
 *  PERFORMANCE OF THE SOFTWARE IS WITH THE USER.
 *
 *  Module:  dcmdata
 *
 *  Author:  Gerd Ehlers
 *
 *  Purpose: stack class
 *
 *  Last Update:      Author: amithaperera 
 *  Update Date:      Date: 2004/01/14 04:01:09 
 *  Source File:      Source: /cvsroot/vxl/vxl/v3p/dcmtk/dcmdata/include/dcstack.h,v 
 *  CVS/RCS Revision: Revision: 1.1 
 *  Status:           State: Exp 
 *
 *  CVS/RCS Log at end of file
 *
 */

#ifndef DCSTACK_H
#define DCSTACK_H

#include "osconfig.h"    /* make sure OS specific configuration is included first */

#include "dctypes.h"

class DcmObject;    // forward declaration


class DcmStackNode
{
    friend class DcmStack;
    DcmStackNode *link;
    DcmObject *objNodeValue;

 // --- declarations to avoid compiler warnings
 
    DcmStackNode(const DcmStackNode &);
    DcmStackNode &operator=(const DcmStackNode &);

public:
    DcmStackNode( DcmObject *obj );
    ~DcmStackNode();
    DcmObject *value();
};

/*  only pointers to elements are managed on the stack.
 *  clear() or destructor deletes the stack but not the elements pointed to.
 */

class DcmStack {
    DcmStackNode *topNode;
    unsigned long cardinality;

 // --- declarations to avoid compiler warnings
 
    DcmStack &operator=(const DcmStack &);

public:
    DcmStack();
    DcmStack( const DcmStack &newStack );
    ~DcmStack();

    DcmObject* push( DcmObject *obj );
    DcmObject* pop();
    DcmObject* top();
    DcmObject* elem(const unsigned long number);
    OFBool empty();
    unsigned long card();
    void clear();
};


#endif  // DCSTACK_H

/*
 * CVS/RCS Log:
 * Log: dcstack.h,v 
 * Revision 1.1  2004/01/14 04:01:09  amithaperera
 * Add better DICOM support by wrapping DCMTK, and add a stripped down
 * version of DCMTK to v3p. Add more DICOM test cases.
 *
 * Revision 1.9  2001/06/01 15:48:44  meichel
 * Updated copyright header
 *
 * Revision 1.8  2000/03/08 16:26:18  meichel
 * Updated copyright header.
 *
 * Revision 1.7  1999/03/31 09:24:47  meichel
 * Updated copyright header in module dcmdata
 *
 *
 */
