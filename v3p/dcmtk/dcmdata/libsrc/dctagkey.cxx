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
 *  Author:  Andrew Hewett
 *
 *  Purpose: Basis class for dicom tags.
 *
 *  Last Update:      Author: amithaperera 
 *  Update Date:      Date: 2004/01/14 04:01:10 
 *  Source File:      Source: /cvsroot/vxl/vxl/v3p/dcmtk/dcmdata/libsrc/dctagkey.cxx,v 
 *  CVS/RCS Revision: Revision: 1.1 
 *  Status:           State: Exp 
 *
 *  CVS/RCS Log at end of file
 *
 */

#include "osconfig.h"    /* make sure OS specific configuration is included first */
#include "dctagkey.h"

#define INCLUDE_CSTDIO
#include "ofstdinc.h"

/*
 * DcmTagKey member functions
 */

OFString DcmTagKey::toString() const
{
    char tagBuf[16];

    if (group == 0xffff && element == 0xffff)
    {
        strcpy(tagBuf, "(\?\?\?\?,\?\?\?\?)"); // prevent trigraph expansion in string constant
    } else {
        sprintf(tagBuf, "(%04x,%04x)", (unsigned)group, (unsigned)element);
    }
    return tagBuf;
}


OFBool DcmTagKey::isSignableTag() const
{
  //no group length tags (element number of 0000)
  if (element == 0) return OFFalse;

  // no Length to End Tag
  if ((group == 0x0008)&&(element==0x0001)) return OFFalse;

  //no tags with group number less than 0008
  if (group < 8) return OFFalse;

  //no tags from group FFFA (digital signatures sequence)
  if (group == 0xfffa) return OFFalse;

  // no MAC Parameters sequence
  if ((group == 0x4ffe)&&(element==0x0001)) return OFFalse;

  //no Data Set trailing Padding
  if ((group == 0xfffc)&&(element==0xfffc)) return OFFalse;

  //no Sequence or Item Delimitation Tag
  if ((group == 0xfffe)&&((element==0xe00d)||(element==0xe0dd))) return OFFalse;

  return OFTrue;
}

/*
** DcmTagKey friend functions
*/

ostream& operator<<(ostream& s, const DcmTagKey& k) 
{
    s << k.toString();
    return s;
}


/*
** CVS/RCS Log:
** Log: dctagkey.cxx,v 
** Revision 1.1  2004/01/14 04:01:10  amithaperera
** Add better DICOM support by wrapping DCMTK, and add a stripped down
** version of DCMTK to v3p. Add more DICOM test cases.
**
** Revision 1.10  2002/11/27 12:06:53  meichel
** Adapted module dcmdata to use of new header file ofstdinc.h
**
** Revision 1.9  2001/11/19 15:23:29  meichel
** Cleaned up signature code to avoid some gcc warnings.
**
** Revision 1.8  2001/11/02 13:18:52  meichel
** Removed character sequences that could be interpreted as ISO C++ trigraphs
**
** Revision 1.7  2001/06/01 15:49:11  meichel
** Updated copyright header
**
** Revision 1.6  2000/11/07 16:56:23  meichel
** Initial release of dcmsign module for DICOM Digital Signatures
**
** Revision 1.5  2000/03/08 16:26:43  meichel
** Updated copyright header.
**
** Revision 1.4  2000/02/07 14:45:17  meichel
** Removed const qualifier from DcmTagKey::toString(), avoids warning on Irix.
**
** Revision 1.3  1999/03/31 09:25:42  meichel
** Updated copyright header in module dcmdata
**
** Revision 1.2  1999/03/17 11:08:58  meichel
** added method DcmTagKey::toString()
**
** Revision 1.1  1995/11/23 17:02:55  hewett
** Updated for loadable data dictionary.  Some cleanup (more to do).
**
*/
