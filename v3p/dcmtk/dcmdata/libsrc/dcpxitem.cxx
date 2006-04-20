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
 *  Author:  Gerd Ehlers, Andreas Barth
 *
 *  Purpose: Implementation of class DcmPixelItem
 *
 *  Last Update:      Author: amithaperera 
 *  Update Date:      Date: 2004/01/14 04:01:10 
 *  Source File:      Source: /cvsroot/vxl/vxl/v3p/dcmtk/dcmdata/libsrc/dcpxitem.cxx,v 
 *  CVS/RCS Revision: Revision: 1.1 
 *  Status:           State: Exp 
 *
 *  CVS/RCS Log at end of file
 *
 */


#include "osconfig.h"    /* make sure OS specific configuration is included first */

#define INCLUDE_CSTDLIB
#define INCLUDE_CSTDIO
#define INCLUDE_CSTRING
#include "ofstdinc.h"

#include "ofstream.h"
#include "dcpxitem.h"
#include "dcswap.h"
#include "ofstring.h"
#include "ofstd.h"
#include "dcistrma.h"    /* for class DcmInputStream */
#include "dcostrma.h"    /* for class DcmOutputStream */


// ********************************


DcmPixelItem::DcmPixelItem(const DcmTag &tag,
                           const Uint32 len)
  : DcmOtherByteOtherWord(tag, len)
{
    Tag.setVR(EVR_pixelItem);
}


DcmPixelItem::DcmPixelItem(const DcmPixelItem &old)
  : DcmOtherByteOtherWord(old)
{
}


DcmPixelItem::~DcmPixelItem()
{
}


// ********************************


OFCondition DcmPixelItem::writeTagAndLength(DcmOutputStream &outStream,
                                            const E_TransferSyntax oxfer,
                                            Uint32 &writtenBytes) const
{
    OFCondition l_error = outStream.status();
    if (l_error.good())
    {
        /* write tag information */
        l_error = writeTag(outStream, Tag, oxfer);
        writtenBytes = 4;
        /* prepare to write the value field */
        Uint32 valueLength = Length;
        DcmXfer outXfer(oxfer);
        /* check byte-ordering */
        const E_ByteOrder oByteOrder = outXfer.getByteOrder();
        if (oByteOrder == EBO_unknown)
        {
            return EC_IllegalCall;
        }
        swapIfNecessary(oByteOrder, gLocalByteOrder, &valueLength, 4, 4);
        // availability of four bytes space in output buffer
        // has been checked by caller.
        writtenBytes += outStream.write(&valueLength, 4);
    } else
        writtenBytes = 0;
    return l_error;
}


void DcmPixelItem::print(ostream &out,
                         const size_t flags,
                         const int level,
                         const char *pixelFileName,
                         size_t *pixelCounter)
{
    /* call inherited method */
    printPixel(out, flags, level, pixelFileName, pixelCounter);
}


OFCondition DcmPixelItem::createOffsetTable(const DcmOffsetList &offsetList)
{
    OFCondition result = EC_Normal;

    unsigned long numEntries = offsetList.size();
    if (numEntries > 0)
    {
        Uint32 current = 0;
        Uint32 *array = new Uint32[numEntries];
        if (array)
        {
            OFListIterator(Uint32) first = offsetList.begin();
            OFListIterator(Uint32) last = offsetList.end();
            unsigned long idx = 0;
            while (first != last)
            {
                array[idx++] = current;
                current += *first;
                ++first;
            }
            result = swapIfNecessary(EBO_LittleEndian, gLocalByteOrder,
                array, numEntries * sizeof(Uint32), sizeof(Uint32));
            if (result.good())
                result = putUint8Array((Uint8 *)array, numEntries * sizeof(Uint32));
            delete[] array;
        } else
            result = EC_MemoryExhausted;
    }
    return result;
}


OFCondition DcmPixelItem::writeXML(ostream &out,
                                   const size_t flags)
{
    /* XML start tag for "item" */
    out << "<pixel-item";
    /* value length in bytes = 0..max */
    out << " len=\"" << Length << "\"";
    /* value loaded = no (or absent)*/
    if (!valueLoaded())
        out << " loaded=\"no\"";
    /* pixel item contains binary data */
    if (!(flags & DCMTypes::XF_writeBinaryData))
        out << " binary=\"hidden\"";
    else if (flags & DCMTypes::XF_encodeBase64)
        out << " binary=\"base64\"";
    else
        out << " binary=\"yes\"";
    out << ">";
    /* write element value (if loaded) */
    if (valueLoaded() && (flags & DCMTypes::XF_writeBinaryData))
    {
        OFString value;
        /* encode binary data as Base64 */
        if (flags & DCMTypes::XF_encodeBase64)
        {
            /* pixel items always contain 8 bit data, therefore, byte swapping not required */
            out << OFStandard::encodeBase64((Uint8 *)getValue(), (size_t)Length, value);
        } else {
            /* encode as sequence of hexadecimal numbers */
            if (getOFStringArray(value).good())
                out << value;
        }
    }
    /* XML end tag for "item" */
    out << "</pixel-item>" << endl;
    /* always report success */
    return EC_Normal;
}


/*
** CVS/RCS Log:
** Log: dcpxitem.cxx,v 
** Revision 1.1  2004/01/14 04:01:10  amithaperera
** Add better DICOM support by wrapping DCMTK, and add a stripped down
** version of DCMTK to v3p. Add more DICOM test cases.
**
** Revision 1.25  2002/12/06 13:16:59  joergr
** Enhanced "print()" function by re-working the implementation and replacing
** the boolean "showFullData" parameter by a more general integer flag.
** Made source code formatting more consistent with other modules/files.
**
** Revision 1.24  2002/11/27 12:06:51  meichel
** Adapted module dcmdata to use of new header file ofstdinc.h
**
** Revision 1.23  2002/08/27 16:55:55  meichel
** Initial release of new DICOM I/O stream classes that add support for stream
**   compression (deflated little endian explicit VR transfer syntax)
**
** Revision 1.22  2002/05/24 14:51:51  meichel
** Moved helper methods that are useful for different compression techniques
**   from module dcmjpeg to module dcmdata
**
** Revision 1.21  2002/05/14 08:21:52  joergr
** Added support for Base64 (MIME) encoded binary data.
**
** Revision 1.20  2002/04/25 10:25:49  joergr
** Added support for XML output of DICOM objects.
**
** Revision 1.19  2002/04/16 13:43:20  joergr
** Added configurable support for C++ ANSI standard includes (e.g. streams).
** Thanks to Andreas Barth <Andreas.Barth@bruker-biospin.de> for his
** contribution.
**
** Revision 1.18  2001/11/16 15:55:04  meichel
** Adapted digital signature code to final text of supplement 41.
**
** Revision 1.17  2001/09/25 17:19:53  meichel
** Adapted dcmdata to class OFCondition
**
** Revision 1.16  2001/06/01 15:49:08  meichel
** Updated copyright header
**
** Revision 1.15  2000/04/14 15:55:06  meichel
** Dcmdata library code now consistently uses ofConsole for error output.
**
** Revision 1.14  2000/03/08 16:26:40  meichel
** Updated copyright header.
**
** Revision 1.13  2000/03/03 14:05:35  meichel
** Implemented library support for redirecting error messages into memory
**   instead of printing them to stdout/stderr for GUI applications.
**
** Revision 1.12  2000/02/23 15:12:00  meichel
** Corrected macro for Borland C++ Builder 4 workaround.
**
** Revision 1.11  2000/02/10 10:52:22  joergr
** Added new feature to dcmdump (enhanced print method of dcmdata): write
** pixel data/item value fields to raw files.
**
** Revision 1.10  2000/02/03 16:31:26  joergr
** Fixed bug: encapsulated data (pixel items) have never been loaded using
** method 'loadAllDataIntoMemory'. Therefore, encapsulated pixel data was
** never printed with 'dcmdump'.
** Corrected bug that caused wrong calculation of group length for sequence
** of items (e.g. encapsulated pixel data).
**
** Revision 1.9  2000/02/01 10:12:09  meichel
** Avoiding to include <stdlib.h> as extern "C" on Borland C++ Builder 4,
**   workaround for bug in compiler header files.
**
** Revision 1.8  1999/03/31 09:25:37  meichel
** Updated copyright header in module dcmdata
**
** Revision 1.7  1998/11/12 16:48:19  meichel
** Implemented operator= for all classes derived from DcmObject.
**
** Revision 1.6  1997/07/07 07:52:29  andreas
** - Enhanced (faster) byte swapping routine. swapIfNecessary moved from
**   a method in DcmObject to a general function.
**
** Revision 1.5  1997/07/03 15:10:03  andreas
** - removed debugging functions Bdebug() and Edebug() since
**   they write a static array and are not very useful at all.
**   Cdebug and Vdebug are merged since they have the same semantics.
**   The debugging functions in dcmdata changed their interfaces
**   (see dcmdata/include/dcdebug.h)
**
** Revision 1.4  1997/05/22 16:57:16  andreas
** - Corrected errors for writing of pixel sequences for encapsulated
**   transfer syntaxes.
**
** Revision 1.3  1996/01/05 13:27:41  andreas
** - changed to support new streaming facilities
** - unique read/write methods for file and block transfer
** - more cleanups
**
*/
