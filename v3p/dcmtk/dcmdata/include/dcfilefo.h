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
 *  Purpose: Interface of class DcmFileFormat
 *
 *  Last Update:      Author: amithaperera 
 *  Update Date:      Date: 2004/01/14 04:01:09 
 *  Source File:      Source: /cvsroot/vxl/vxl/v3p/dcmtk/dcmdata/include/dcfilefo.h,v 
 *  CVS/RCS Revision: Revision: 1.1 
 *  Status:           State: Exp 
 *
 *  CVS/RCS Log at end of file
 *
 */


#ifndef DCFILEFO_H
#define DCFILEFO_H

#include "osconfig.h"    /* make sure OS specific configuration is included first */

#include "ofconsol.h"
#include "dctypes.h"
#include "dcerror.h"
#include "dcsequen.h"
#include "dcdatset.h"


// forward declarations
class DcmMetaInfo;
class DcmInputStream;
class DcmOutputStream;
class DcmRepresentationParameter;


/** a class handling the DICOM file format (with meta header)
 */
class DcmFileFormat
  : public DcmSequenceOfItems
{

 public:

    /** default constructor
     */
    DcmFileFormat();

    DcmFileFormat(DcmDataset *dataset);

    /** copy constructor
     *  @param old element to be copied
     */
    DcmFileFormat(const DcmFileFormat &old);

    /** destructor
     */
    virtual ~DcmFileFormat();

    /** assignment operator
     *  @param obj fileformat to be assigned/copied
     *  @return reference to this object
     */
    DcmFileFormat &operator=(const DcmFileFormat &obj);

    /** get type identifier
     *  @return type identifier of this class (EVR_fileFormat)
     */
    virtual DcmEVR ident() const;

    /** print meta header and dataset to a stream
     *  @param out output stream
     *  @param flags optional flag used to customize the output (see DCMTypes::PF_xxx)
     *  @param level current level of nested items. Used for indentation.
     *  @param pixelFileName not used
     *  @param pixelCounter not used
     */
    virtual void print(ostream &out,
                       const size_t flags = 0,
                       const int level = 0,
                       const char *pixelFileName = NULL,
                       size_t *pixelCounter = NULL);

    virtual OFCondition validateMetaInfo(E_TransferSyntax oxfer);

    DcmMetaInfo *getMetaInfo();
    DcmDataset  *getDataset();
    DcmDataset  *getAndRemoveDataset();

    virtual Uint32 calcElementLength(const E_TransferSyntax xfer,
                                     const E_EncodingType enctype);

    virtual OFBool canWriteXfer(const E_TransferSyntax newXfer,
                                const E_TransferSyntax oldXfer = EXS_Unknown);

    virtual OFCondition read(DcmInputStream &inStream,
                             const E_TransferSyntax xfer = EXS_Unknown,
                             const E_GrpLenEncoding glenc = EGL_noChange,
                             const Uint32 maxReadLength = DCM_MaxReadLength);

    /** write fileformat to a stream
     *  @param outStream DICOM output stream
     *  @param oxfer output transfer syntax
     *  @param enctype encoding types (undefined or explicit length)
     *  @return status, EC_Normal if successful, an error code otherwise
     */
    virtual OFCondition write(DcmOutputStream &outStream,
                              const E_TransferSyntax oxfer,
                              const E_EncodingType enctype = EET_UndefinedLength);

    virtual OFCondition write(DcmOutputStream &outStream,
                              const E_TransferSyntax oxfer,
                              const E_EncodingType enctype,
                              const E_GrpLenEncoding glenc,
                              const E_PaddingEncoding padenc = EPD_noChange,
                              const Uint32 padlen = 0,
                              const Uint32 subPadlen = 0,
                              Uint32 instanceLength = 0);

    /** write object in XML format.
     *  The XML declaration (e.g. <?xml version="1.0"?>) is not written by this function.
     *  @param out output stream to which the XML document is written
     *  @param flags optional flag used to customize the output (see DCMTypes::XF_xxx)
     *  @return status, EC_Normal if successful, an error code otherwise
     */
    virtual OFCondition writeXML(ostream &out,
                                 const size_t flags = 0);

    /** load object from a DICOM file.
     *  This method supports DICOM objects stored as a file (with meta header) or as a
     *  dataset (without meta header). Presence of a meta header is detected automatically.
     *  @param fileName name of the file to load
     *  @param readXfer transfer syntax used to read the data (auto detection if EXS_Unknown)
     *  @param groupLength flag, specifying how to handle the group length tags
     *  @param maxReadLength maximum number of bytes to be read for an element value.
     *    Element values with a larger size are not loaded until their value is retrieved
     *    (with getXXX()) or loadAllDataElements() is called.
     *  @param isDataset if true, meta header detection is disabled and loading of a
     *    dataset without meta header is forced.
     *  @return status, EC_Normal if successful, an error code otherwise
     */
    virtual OFCondition loadFile(const char *fileName,
                                 const E_TransferSyntax readXfer = EXS_Unknown,
                                 const E_GrpLenEncoding groupLength = EGL_noChange,
                                 const Uint32 maxReadLength = DCM_MaxReadLength,
                                 OFBool isDataset = OFFalse);

    /** save object to a DICOM file.
     *  @param fileName name of the file to save
     *  @param writeXfer transfer syntax used to write the data (EXS_Unknown means use current)
     *  @param encodingType flag, specifying the encoding with undefined or explicit length
     *  @param groupLength flag, specifying how to handle the group length tags
     *  @param padEncoding flag, specifying how to handle the padding tags
     *  @param padLength number of bytes used for the dataset padding (has to be an even number)
     *  @param subPadLength number of bytes used for the item padding (has to be an even number)
     *  @param isDataset if true, file is stored without meta header, i.e. as pure dataset
     *  @return status, EC_Normal if successful, an error code otherwise
     */
    virtual OFCondition saveFile(const char *fileName,
                                 const E_TransferSyntax writeXfer = EXS_Unknown,
                                 const E_EncodingType encodingType = EET_UndefinedLength,
                                 const E_GrpLenEncoding groupLength = EGL_recalcGL,
                                 const E_PaddingEncoding padEncoding = EPD_noChange,
                                 const Uint32 padLength = 0,
                                 const Uint32 subPadLength = 0,
                                 OFBool isDataset = OFFalse);

    // methods for different pixel representations

    // choose Representation changes the representation of
    // PixelData Elements in the data set to the given representation
    // If the representation does not exists it creates one.
    OFCondition chooseRepresentation(const E_TransferSyntax repType,
                                     const DcmRepresentationParameter *repParam)
    {
        return getDataset()->chooseRepresentation(repType, repParam);
    }

    // checks if all PixelData elements have a conforming representation
    // (for definition of conforming representation see dcpixel.h).
    // if one PixelData element has no conforming representation
    // OFFalse is returned.
    OFBool hasRepresentation(const E_TransferSyntax repType,
                             const DcmRepresentationParameter *repParam)
    {
        return getDataset()->hasRepresentation(repType, repParam);
    }

    /** removes all but the original representation in all pixel data elements
     */
    void removeAllButOriginalRepresentations()
    {
        getDataset()->removeAllButOriginalRepresentations();
    }

    /** removes all but the current representation in all pixel data elements.
     *  Makes the current representation original.
     */
    void removeAllButCurrentRepresentations()
    {
        getDataset()->removeAllButCurrentRepresentations();
    }


// The following methods have no meaning in DcmFileFormat and shall not be
// called. Since it is not possible to delete inherited methods from a class
// stubs are defined that create an error.

    virtual OFCondition insertItem(DcmItem *item,
                                   const unsigned long where = DCM_EndOfListIndex);
    virtual DcmItem *remove(const unsigned long num);
    virtual DcmItem *remove(DcmItem *item);
    virtual OFCondition clear();


  private:

    OFCondition checkValue(DcmMetaInfo *metainfo,
                           DcmDataset *dataset,
                           const DcmTagKey &atagkey,
                           DcmObject *obj,
                           const E_TransferSyntax oxfer);

    E_TransferSyntax lookForXfer(DcmMetaInfo *metainfo);
};


#endif // DCFILEFO_H


/*
** CVS/RCS Log:
** Log: dcfilefo.h,v 
** Revision 1.1  2004/01/14 04:01:09  amithaperera
** Add better DICOM support by wrapping DCMTK, and add a stripped down
** version of DCMTK to v3p. Add more DICOM test cases.
**
** Revision 1.22  2002/12/06 12:49:10  joergr
** Enhanced "print()" function by re-working the implementation and replacing
** the boolean "showFullData" parameter by a more general integer flag.
** Added doc++ documentation.
** Made source code formatting more consistent with other modules/files.
**
** Revision 1.21  2002/08/27 16:55:32  meichel
** Initial release of new DICOM I/O stream classes that add support for stream
**   compression (deflated little endian explicit VR transfer syntax)
**
** Revision 1.20  2002/08/20 12:18:35  meichel
** Changed parameter list of loadFile and saveFile methods in class
**   DcmFileFormat. Removed loadFile and saveFile from class DcmObject.
**
** Revision 1.19  2002/04/25 09:39:47  joergr
** Added support for XML output of DICOM objects.
**
** Revision 1.18  2002/04/11 12:22:51  joergr
** Added new methods for loading and saving DICOM files.
**
** Revision 1.17  2001/09/25 17:19:26  meichel
** Adapted dcmdata to class OFCondition
**
** Revision 1.16  2001/06/01 15:48:40  meichel
** Updated copyright header
**
** Revision 1.15  2000/04/14 15:31:32  meichel
** Removed default value from output stream passed to print() method.
**   Required for use in multi-thread environments.
**
** Revision 1.14  2000/03/08 16:26:14  meichel
** Updated copyright header.
**
** Revision 1.13  2000/03/03 14:05:23  meichel
** Implemented library support for redirecting error messages into memory
**   instead of printing them to stdout/stderr for GUI applications.
**
** Revision 1.12  2000/02/10 10:50:51  joergr
** Added new feature to dcmdump (enhanced print method of dcmdata): write
** pixel data/item value fields to raw files.
**
** Revision 1.11  1999/03/31 09:24:38  meichel
** Updated copyright header in module dcmdata
**
** Revision 1.10  1998/11/12 16:47:39  meichel
** Implemented operator= for all classes derived from DcmObject.
**
** Revision 1.9  1997/07/21 08:14:39  andreas
** - New environment for encapsulated pixel representations. DcmPixelData
**   can contain different representations and uses codecs to convert
**   between them. Codecs are derived from the DcmCodec class. New error
**   codes are introduced for handling of representations. New internal
**   value representation (only for ident()) for PixelData
** - Replace all boolean types (BOOLEAN, CTNBOOLEAN, DICOM_BOOL, BOOL)
**   with one unique boolean type OFBool.
**
** Revision 1.8  1997/05/27 13:48:27  andreas
** - Add method canWriteXfer to class DcmObject and all derived classes.
**   This method checks whether it is possible to convert the original
**   transfer syntax to an new transfer syntax. The check is used in the
**   dcmconv utility to prohibit the change of a compressed transfer
**   syntax to a uncompressed.
**
** Revision 1.7  1997/05/16 08:23:47  andreas
** - Revised handling of GroupLength elements and support of
**   DataSetTrailingPadding elements. The enumeratio E_GrpLenEncoding
**   got additional enumeration values (for a description see dctypes.h).
**   addGroupLength and removeGroupLength methods are replaced by
**   computeGroupLengthAndPadding. To support Padding, the parameters of
**   element and sequence write functions changed.
** - Added a new method calcElementLength to calculate the length of an
**   element, item or sequence. For elements it returns the length of
**   tag, length field, vr field, and value length, for item and
**   sequences it returns the length of the whole item. sequence including
**   the Delimitation tag (if appropriate).  It can never return
**   UndefinedLength.
**
** Revision 1.6  1996/08/05 08:45:21  andreas
** new print routine with additional parameters:
**         - print into files
**         - fix output length for elements
** corrected error in search routine with parameter ESM_fromStackTop
**
** Revision 1.5  1996/07/29 15:46:23  andreas
** Add method getAndRemoveDataset to remove connection of dataset from fileformat
**
** Revision 1.4  1996/01/09 11:06:15  andreas
** New Support for Visual C++
** Correct problems with inconsistent const declarations
**
** Revision 1.3  1996/01/05 13:22:55  andreas
** - changed to support new streaming facilities
** - more cleanups
** - merged read / write methods for block and file transfer
**
*/
