/*
 *
 *  Copyright (C) 1994-2002, OFFIS
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
 *  Update Date:      Date: 2004/01/14 04:01:10 
 *  Source File:      Source: /cvsroot/vxl/vxl/v3p/dcmtk/dcmdata/libsrc/dcstack.cxx,v 
 *  CVS/RCS Revision: Revision: 1.1 
 *  Status:           State: Exp 
 *
 *  CVS/RCS Log at end of file
 *
 */

#include "osconfig.h"    /* make sure OS specific configuration is included first */

#define INCLUDE_CSTDLIB
#include "ofstdinc.h"

#include "ofstream.h"
#include "dcstack.h"
#include "dcdebug.h"


// ************************************************
// *** DcmStackNode() *****************************
// ************************************************


DcmStackNode::DcmStackNode( DcmObject *obj )
  : link(NULL),
    objNodeValue(obj)
{
}


// ********************************


DcmStackNode::~DcmStackNode()
{
}


// ********************************


DcmObject* DcmStackNode::value()
{
    return objNodeValue;
}



// ************************************************
// *** DcmStack() *********************************
// ************************************************


DcmStack::DcmStack()
  : topNode(NULL),
    cardinality(0)
{
}


// ************************************************


DcmStack::DcmStack( const DcmStack & oldStack)
  : topNode(NULL),
    cardinality(oldStack.cardinality)
{
    if (cardinality)
    {
        topNode = new DcmStackNode(oldStack.topNode->objNodeValue);
        DcmStackNode * oldPtr = oldStack.topNode->link;
        DcmStackNode * newPtr = topNode;
        while (oldPtr)
        {
            newPtr->link = new DcmStackNode(oldPtr->objNodeValue);
            oldPtr = oldPtr->link;
            newPtr = newPtr->link;
        }
    }
}


// ********************************


DcmStack::~DcmStack()
{
    clear();
}


// ********************************


void DcmStack::clear()
{
    DcmStackNode *node;
    while (topNode != (DcmStackNode*)NULL)
    {
        node = topNode;
        topNode = topNode->link;
        delete node;
    }
    cardinality = 0;
}


// ********************************


DcmObject* DcmStack::push( DcmObject *obj )
{
    if ( obj != (DcmObject*)NULL )
    {
        DcmStackNode *node = new DcmStackNode( obj );
        node->link = topNode;
        topNode = node;
        cardinality++;
    }
    return obj;
}


// ********************************


DcmObject* DcmStack::pop()
{
    DcmObject *obj;
    DcmStackNode *node;
    if ( topNode != (DcmStackNode*)NULL )
    {
        obj = topNode->value();
        node = topNode;
        topNode = topNode->link;
        delete node;
        cardinality--;
    }
    else
        obj = (DcmObject*)NULL;
    return obj;
}


// ********************************


DcmObject* DcmStack::elem(const unsigned long number)
{
    unsigned long num = number;
    DcmObject *obj;
    DcmStackNode *node = topNode;
    while ( num-- > 0 && node != (DcmStackNode*)NULL )
         node = node->link;

    if ( node != (DcmStackNode*)NULL )
        obj = node->value();
    else
        obj = (DcmObject*)NULL;
    return obj;
}


// ********************************


DcmObject* DcmStack::top()
{
    DcmObject *obj;
    if ( topNode != (DcmStackNode*)NULL )
        obj = topNode->value();
    else
        obj = (DcmObject*)NULL;
    return obj;
}


// ********************************


unsigned long DcmStack::card()
{
    return cardinality;
}


// ********************************


OFBool DcmStack::empty()
{
    return (OFBool)( topNode == (DcmStackNode*)NULL );
}


/*
 * CVS/RCS Log:
 * Log: dcstack.cxx,v 
 * Revision 1.1  2004/01/14 04:01:10  amithaperera
 * Add better DICOM support by wrapping DCMTK, and add a stripped down
 * version of DCMTK to v3p. Add more DICOM test cases.
 *
 * Revision 1.15  2002/11/27 12:06:52  meichel
 * Adapted module dcmdata to use of new header file ofstdinc.h
 *
 * Revision 1.14  2002/04/16 13:43:21  joergr
 * Added configurable support for C++ ANSI standard includes (e.g. streams).
 * Thanks to Andreas Barth <Andreas.Barth@bruker-biospin.de> for his
 * contribution.
 *
 * Revision 1.13  2001/06/01 15:49:09  meichel
 * Updated copyright header
 *
 * Revision 1.12  2000/03/08 16:26:41  meichel
 * Updated copyright header.
 *
 * Revision 1.11  2000/02/23 15:12:02  meichel
 * Corrected macro for Borland C++ Builder 4 workaround.
 *
 * Revision 1.10  2000/02/01 10:12:10  meichel
 * Avoiding to include <stdlib.h> as extern "C" on Borland C++ Builder 4,
 *   workaround for bug in compiler header files.
 *
 * Revision 1.9  1999/03/31 09:25:39  meichel
 * Updated copyright header in module dcmdata
 *
 *
 */
