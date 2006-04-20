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
 *  Purpose: Implementation of class DcmSequenceOfItems
 *
 *  Last Update:      Author: peter_vanroose 
 *  Update Date:      Date: 2004/05/28 17:59:56 
 *  Source File:      Source: /cvsroot/vxl/vxl/v3p/dcmtk/dcmdata/libsrc/dcsequen.cxx,v 
 *  CVS/RCS Revision: Revision: 1.2 
 *  Status:           State: Exp 
 *
 *  CVS/RCS Log at end of file
 *
 */


#include "osconfig.h"    /* make sure OS specific configuration is included first */

#define INCLUDE_CSTDLIB
#define INCLUDE_CSTDIO
#include "ofstdinc.h"

#include "ofstream.h"
#include "ofstd.h"

#include "dcsequen.h"
#include "dcitem.h"
#include "dcdirrec.h"
#include "dcvr.h"
#include "dcpxitem.h"
#include "dcswap.h"
#include "dcdebug.h"
#include "dcmetinf.h"
#include "dcdeftag.h"
#include "dcistrma.h"    /* for class DcmInputStream */
#include "dcostrma.h"    /* for class DcmOutputStream */


// ********************************


DcmSequenceOfItems::DcmSequenceOfItems(const DcmTag &tag,
                                       const Uint32 len)
  : DcmElement(tag, len),
    itemList(NULL),
    lastItemComplete(OFTrue),
    fStartPosition(0)
{
    itemList = new DcmList;
}


// ********************************


DcmSequenceOfItems::DcmSequenceOfItems(const DcmSequenceOfItems &old)
  : DcmElement(old),
    itemList(NULL),
    lastItemComplete(OFTrue),
    fStartPosition(old.fStartPosition)
{
    itemList = new DcmList;

    switch (old.ident())
    {
        case EVR_SQ:
        case EVR_pixelSQ:
        case EVR_fileFormat:
            if (!old.itemList->empty())
            {
                DcmObject *oldDO;
                DcmObject *newDO;
                itemList->seek(ELP_first);
                old.itemList->seek(ELP_first);
                do {
                    oldDO = old.itemList->get();
                    switch (oldDO->ident())
                    {
                        case EVR_item:
                            newDO = new DcmItem(*(DcmItem*)oldDO);
                            break;
                        case EVR_pixelItem:
                            newDO = new DcmPixelItem(*(DcmPixelItem*)oldDO);
                            break;
                        case EVR_metainfo:
                            newDO = new DcmMetaInfo(*(DcmMetaInfo*)oldDO);
                            break;
                        case EVR_dataset:
                            newDO = new DcmDataset(*(DcmDataset*)oldDO);
                            break;
                        default:
                            newDO = new DcmItem(oldDO->getTag());
                            ofConsole.lockCerr() << "Error: DcmSequenceOfItems(): Element("
                                 << hex << setfill('0')
                                 << setw(4) << oldDO->getGTag() << ","
                                 << setw(4) << oldDO->getETag()
                                 << dec << setfill(' ')
                                 << ") found, which was not an Item" << endl;
                            ofConsole.unlockCerr();
                            break;
                    }

                    itemList->insert(newDO, ELP_next);
                } while (old.itemList->seek(ELP_next));
            }
            break;
        default:
            // wrong use of copy constructor, should never happen
            break;
    }
}


// ********************************


DcmSequenceOfItems::~DcmSequenceOfItems()
{
    DcmObject *dO;
    itemList->seek(ELP_first);
    while (!itemList->empty())
    {
        dO = itemList->remove();
        if (dO != (DcmObject*)NULL)
            delete dO;
    }
    delete itemList;
}


// ********************************


DcmSequenceOfItems &DcmSequenceOfItems::operator=(const DcmSequenceOfItems &obj)
{
    DcmElement::operator=(obj);
    lastItemComplete = obj.lastItemComplete;
    fStartPosition = obj.fStartPosition;

    DcmList *newList = new DcmList; // DcmList has no copy constructor. Need to copy ourselves.
    if (newList)
    {
        switch (obj.ident())
        {
            case EVR_SQ:
            case EVR_pixelSQ:
            case EVR_fileFormat:
                if (!obj.itemList->empty())
                {
                    DcmObject *oldDO;
                    DcmObject *newDO;
                    newList->seek(ELP_first);
                    obj.itemList->seek(ELP_first);
                    do {
                        oldDO = obj.itemList->get();
                        switch (oldDO->ident())
                        {
                            case EVR_item:
                                newDO = new DcmItem(*(DcmItem*)oldDO);
                                break;
                            case EVR_pixelItem:
                                newDO = new DcmPixelItem(*(DcmPixelItem*)oldDO);
                                break;
                            case EVR_metainfo:
                                newDO = new DcmMetaInfo(*(DcmMetaInfo*)oldDO);
                                break;
                            case EVR_dataset:
                                newDO = new DcmDataset(*(DcmDataset*)oldDO);
                                break;
                            default:
                                newDO = new DcmItem(oldDO->getTag());
                                ofConsole.lockCerr() << "Error: DcmSequenceOfItems(): Element("
                                     << hex << oldDO->getGTag() << "," << oldDO->getETag()
                                     << dec << ") found, which was not an Item" << endl;
                                ofConsole.unlockCerr();
                                break;
                        }
                        newList->insert(newDO, ELP_next);
                    } while (obj.itemList->seek(ELP_next));
                }
                break;
            default:
                // wrong use of assignment operator, should never happen
                break;
        }
    }
    delete itemList;
    itemList = newList;

    return *this;
}


// ********************************


void DcmSequenceOfItems::print(ostream &out,
                               const size_t flags,
                               const int level,
                               const char *pixelFileName,
                               size_t *pixelCounter)
{
    /* print sequence start line */
    if (flags & DCMTypes::PF_showTreeStructure)
    {
        /* empty text */
        printInfoLine(out, flags, level);
        /* print sequence content */
        if (!itemList->empty())
        {
            /* reset internal flags */
            const size_t newFlags = flags & ~DCMTypes::PF_lastEntry;
            /* print all items contained in the sequence */
            DcmObject *dO;
            itemList->seek(ELP_first);
            do {
                dO = itemList->get();
                dO->print(out, newFlags, level + 1, pixelFileName, pixelCounter);
            } while (itemList->seek(ELP_next));
        }
    } else {
        OFOStringStream oss;
        oss << "(Sequence ";
        if (Length == DCM_UndefinedLength)
            oss << "undefined";
        else
            oss << "explicit";
        oss << " length #=" << card() << ")" << OFStringStream_ends;
        OFSTRINGSTREAM_GETSTR(oss, tmpString)
        printInfoLine(out, flags, level, tmpString);
        OFSTRINGSTREAM_FREESTR(tmpString)
        /* print sequence content */
        if (!itemList->empty())
        {
            DcmObject *dO;
            itemList->seek(ELP_first);
            do {
                dO = itemList->get();
                dO->print(out, flags, level + 1, pixelFileName, pixelCounter);
            } while (itemList->seek(ELP_next));
        }
        /* print sequence end line */
        DcmTag delimItemTag(DCM_SequenceDelimitationItem);
        if (Length == DCM_UndefinedLength)
            printInfoLine(out, flags, level, "(SequenceDelimitationItem)", &delimItemTag);
        else
            printInfoLine(out, flags, level, "(SequenceDelimitationItem for re-encod.)", &delimItemTag);
    }
}


// ********************************


OFCondition DcmSequenceOfItems::writeXML(ostream &out,
                                         const size_t flags)
{
    OFString xmlString;
    DcmVR vr(Tag.getVR());
    /* XML start tag for "sequence" */
    out << "<sequence";
    /* attribute tag = (gggg,eeee) */
    out << " tag=\"";
    out << hex << setfill('0')
        << setw(4) << Tag.getGTag() << ","
        << setw(4) << Tag.getETag() << "\""
        << dec << setfill(' ');
    /* value representation = VR */
    out << " vr=\"" << vr.getVRName() << "\"";
    /* cardinality (number of items) = 1..n */
    out << " card=\"" << card() << "\"";
    /* value length in bytes = 0..max (if not undefined) */
    if (Length != DCM_UndefinedLength)
        out << " len=\"" << Length << "\"";
    /* tag name (if known) */
    out << " name=\"" << OFStandard::convertToMarkupString(Tag.getTagName(), xmlString) << "\"";
    out << ">" << endl;
    /* write sequence content */
    if (!itemList->empty())
    {
        /* write content of all children */
        DcmObject *dO;
        itemList->seek(ELP_first);
        do
        {
            dO = itemList->get();
            dO->writeXML(out, flags);
        } while (itemList->seek(ELP_next));
    }
    /* XML end tag for "sequence" */
    out << "</sequence>" << endl;
    /* always report success */
    return EC_Normal;
}


// ********************************


OFBool DcmSequenceOfItems::canWriteXfer(const E_TransferSyntax newXfer,
                                        const E_TransferSyntax oldXfer)
{
    OFBool canWrite = OFTrue;

    if (newXfer == EXS_Unknown)
        canWrite = OFFalse;
    else if (!itemList->empty())
    {
        DcmObject *dO;
        itemList->seek(ELP_first);
        do
        {
            dO = itemList->get();
            canWrite = dO -> canWriteXfer(newXfer, oldXfer);
        } while (itemList->seek(ELP_next) && canWrite);
    }

    return canWrite;
}


// ********************************


Uint32 DcmSequenceOfItems::calcElementLength(const E_TransferSyntax xfer,
                                             const E_EncodingType enctype)
{
    Uint32 seqlen = DcmElement::calcElementLength(xfer, enctype);
    if (enctype == EET_UndefinedLength)
        seqlen += 8;     // for Sequence Delimitation Tag
    return seqlen;
}


// ********************************


Uint32 DcmSequenceOfItems::getLength(const E_TransferSyntax xfer,
                                     const E_EncodingType enctype)
{
    Uint32 seqlen = 0;
    if (!itemList->empty())
    {
        DcmItem *dI;
        itemList->seek(ELP_first);
        do {
            dI = (DcmItem*)(itemList->get());
            seqlen += dI->calcElementLength(xfer, enctype);
        } while (itemList->seek(ELP_next));
    }
    return seqlen;
}


// ********************************


OFCondition DcmSequenceOfItems::computeGroupLengthAndPadding(const E_GrpLenEncoding glenc,
                                                             const E_PaddingEncoding padenc,
                                                             const E_TransferSyntax xfer,
                                                             const E_EncodingType enctype,
                                                             const Uint32 padlen,
                                                             const Uint32 subPadlen,
                                                             Uint32 instanceLength)
{
    OFCondition l_error = EC_Normal;

    if (!itemList->empty())
    {
        itemList->seek(ELP_first);
        do {
            DcmItem *dO = (DcmItem*)itemList->get();
            l_error = dO->computeGroupLengthAndPadding
                (glenc, padenc, xfer, enctype, padlen,
                 subPadlen, instanceLength);
        } while (itemList->seek(ELP_next));
    }
    return l_error;
}


// ********************************


OFCondition DcmSequenceOfItems::makeSubObject(DcmObject *&subObject,
                                              const DcmTag &newTag,
                                              const Uint32 newLength)
{
    OFCondition l_error = EC_Normal;
    DcmItem *subItem = NULL;

    switch (newTag.getEVR())
    {
        case EVR_na:
            if (newTag.getXTag() == DCM_Item)
            {
                if (getTag().getXTag() == DCM_DirectoryRecordSequence)
                    subItem = new DcmDirectoryRecord(newTag, newLength);
                else
                    subItem = new DcmItem(newTag, newLength);
            }
            else if (newTag.getXTag() == DCM_SequenceDelimitationItem)
                l_error = EC_SequEnd;
            else if (newTag.getXTag() == DCM_ItemDelimitationItem)
                l_error = EC_ItemEnd;
            else
                l_error = EC_InvalidTag;
            break;

        default:
            subItem = new DcmItem(newTag, newLength);
            l_error = EC_CorruptedData;
            break;
    }
    subObject = subItem;
    return l_error;
}


// ********************************


OFCondition DcmSequenceOfItems::readTagAndLength(DcmInputStream &inStream,
                                                 const E_TransferSyntax xfer,
                                                 DcmTag &tag,
                                                 Uint32 &length)
{
    Uint16 groupTag = 0xffff;
    Uint16 elementTag = 0xffff;

    OFCondition l_error = EC_Normal;
    if (inStream.avail() < 8)
        l_error = EC_StreamNotifyClient;

    if (l_error.good())
    {
        DcmXfer iXfer(xfer);
        const E_ByteOrder iByteOrder = iXfer.getByteOrder();
        if (iByteOrder == EBO_unknown)
            return EC_IllegalCall;
        inStream.mark();
        inStream.read(&groupTag, 2);
        inStream.read(&elementTag, 2);
        swapIfNecessary(gLocalByteOrder, iByteOrder, &groupTag, 2, 2);
        swapIfNecessary(gLocalByteOrder, iByteOrder, &elementTag, 2, 2);
        // Tag ist gelesen

        DcmTag newTag(groupTag, elementTag);

        Uint32 valueLength = 0;
        inStream.read(&valueLength, 4);
        swapIfNecessary(gLocalByteOrder, iByteOrder, &valueLength, 4, 4);
        if ((valueLength & 1)&&(valueLength != (Uint32) -1))
        {
            ofConsole.lockCerr() << "Warning: parse error in DICOM object: length of item in sequence " << Tag << " is odd" << endl;
            ofConsole.unlockCerr();
        }
        length = valueLength;
        tag = newTag; // return value: assignment-operator
    }

    debug(4, ("in Sequ.readTag errorFlag = %s", l_error.text()));
    return l_error;
}


// ********************************


OFCondition DcmSequenceOfItems::readSubItem(DcmInputStream &inStream,
                                            const DcmTag &newTag,
                                            const Uint32 newLength,
                                            const E_TransferSyntax xfer,
                                            const E_GrpLenEncoding glenc,
                                            const Uint32 maxReadLength)
{
    // For DcmSequenceOfItems, subObject is always inherited from DcmItem
    // For DcmPixelSequence, subObject is always inherited from DcmPixelItem
    DcmObject * subObject = NULL;
    OFCondition l_error = makeSubObject(subObject, newTag, newLength);
    if (l_error.good() && subObject != NULL)
    {
        // inStream.UnsetPutbackMark(); // not needed anymore with new stream architecture
        itemList->insert(subObject, ELP_next);
        l_error = subObject->read(inStream, xfer, glenc, maxReadLength); // read sub-item
        return l_error; // prevent subObject from getting deleted
    }
    else if (l_error == EC_InvalidTag)  // try to recover parsing
    {
        inStream.putback();
        ofConsole.lockCerr() << "Warning: DcmSequenceOfItems::readSubItem(): parse error occurred: " << newTag << endl;
        ofConsole.unlockCerr();
        debug(1, ("Warning: DcmSequenceOfItems::readSubItem(): parse error occurred:"
            " (0x%4.4hx,0x%4.4hx)", newTag.getGTag(), newTag.getETag()));
    }
    else if (l_error != EC_SequEnd)
    {
        // inStream.UnsetPutbackMark(); // not needed anymore with new stream architecture
        ofConsole.lockCerr() << "Error: DcmSequenceOfItems::readSubItem(): cannot create SubItem " << newTag << endl;
        ofConsole.unlockCerr();
        debug(1, ("Error: DcmSequenceOfItems::readSubItem(): cannot create SubItem"
            " (0x%4.4hx,0x%4.4hx)", newTag.getGTag(), newTag.getETag()));
    } else {
        // inStream.UnsetPutbackMark(); // not needed anymore with new stream architecture
    }

    if (subObject) delete subObject; // only executed if makeSubObject() has returned an error
    return l_error;
}


// ********************************


OFCondition DcmSequenceOfItems::read(DcmInputStream &inStream,
                                     const E_TransferSyntax xfer,
                                     const E_GrpLenEncoding glenc,
                                     const Uint32 maxReadLength)
{
    if (fTransferState == ERW_notInitialized)
        errorFlag = EC_IllegalCall;
    else
    {
        errorFlag = inStream.status();

        if (errorFlag.good() && inStream.eos())
            errorFlag = EC_EndOfStream;
        else if (errorFlag.good() && fTransferState != ERW_ready)
        {
            if (fTransferState == ERW_init)
            {
                fStartPosition = inStream.tell();   // Position Sequence-Value
                fTransferState = ERW_inWork;
            }

            itemList->seek(ELP_last); // append data at end
            while (inStream.good() && (fTransferredBytes < Length || !lastItemComplete))
            {
                DcmTag newTag;
                Uint32 newValueLength = 0;

                if (lastItemComplete)
                {
                    errorFlag = readTagAndLength(inStream, xfer, newTag, newValueLength);

                    if (errorFlag.bad())
                        break;                  // finish while loop
                    else
                        fTransferredBytes += 8;

                    lastItemComplete = OFFalse;
                    errorFlag = readSubItem(inStream, newTag, newValueLength,
                                            xfer, glenc, maxReadLength);
                    if (errorFlag.good())
                        lastItemComplete = OFTrue;
                }
                else
                {
                    errorFlag = itemList->get()->read(inStream, xfer, glenc,
                                                      maxReadLength);
                    if (errorFlag.good())
                        lastItemComplete = OFTrue;
                }
                fTransferredBytes = inStream.tell() - fStartPosition;

                if (errorFlag.bad())
                    break;

            } //while
            if ((fTransferredBytes < Length || !lastItemComplete) && errorFlag.good())
                errorFlag = EC_StreamNotifyClient;
        } // else errorFlag

        if (errorFlag == EC_SequEnd)
            errorFlag = EC_Normal;
        if (errorFlag.good())
            fTransferState = ERW_ready;      // sequence is complete
    }
    return errorFlag;
}


// ********************************


OFCondition DcmSequenceOfItems::write(DcmOutputStream & outStream,
                                      const E_TransferSyntax oxfer,
                                      const E_EncodingType enctype)
  {
    if (fTransferState == ERW_notInitialized)
        errorFlag = EC_IllegalCall;
    else
    {
        errorFlag = outStream.status();
        if (errorFlag.good() && fTransferState != ERW_ready)
        {
            if (fTransferState == ERW_init)
            {
                if (outStream.avail() >= DCM_TagInfoLength) // header might be smaller than this
                {
                    if (enctype == EET_ExplicitLength)
                        Length = getLength(oxfer, enctype);
                    else
                        Length = DCM_UndefinedLength;
                    Uint32 written_bytes = 0;
                    errorFlag = writeTagAndLength(outStream, oxfer, written_bytes);
                    if (errorFlag.good())
                    {
                        fTransferState = ERW_inWork;
                        itemList->seek(ELP_first);
                    }
                } else
                    errorFlag = EC_StreamNotifyClient;
            }
            if (fTransferState == ERW_inWork)
            {
                // itemList->get() can be NULL if buffer was full after
                // writing the last item but before writing the sequence delimitation.
                if (!itemList->empty() && (itemList->get() != NULL))
                {
                    DcmObject *dO;
                    do
                    {
                      dO = itemList->get();
                      if (dO->transferState() != ERW_ready)
                          errorFlag = dO->write(outStream, oxfer, enctype);
                    } while (errorFlag.good() && itemList->seek(ELP_next));
                }
                if (errorFlag.good())
                {
                    fTransferState = ERW_ready;
                    if (Length == DCM_UndefinedLength)
                    {
                        if (outStream.avail() >= 8)
                        {
                            // write sequence delimitation item
                            DcmTag delim(DCM_SequenceDelimitationItem);
                            errorFlag = writeTag(outStream, delim, oxfer);
                            Uint32 delimLen = 0L;
                            outStream.write(&delimLen, 4); // 4 bytes length
                        } else {
                            // the complete sequence is written but it
                            // is not possible to write the delimination item into the buffer.
                            errorFlag = EC_StreamNotifyClient;
                            fTransferState = ERW_inWork;
                        }
                    }
                }
            }
        }
    }
    return errorFlag;
}

// ********************************


OFCondition DcmSequenceOfItems::writeTagAndVR(DcmOutputStream &outStream,
                                              const DcmTag &tag,
                                              DcmEVR vr,
                                              const E_TransferSyntax oxfer)
{
    OFCondition l_error = outStream.status();
    if (l_error.good())
    {
        /* write the tag information (a total of 4 bytes, group number and element */
        /* number) to the stream. Mind the transfer syntax's byte ordering. */
        l_error = writeTag(outStream, tag, oxfer);
        /* create an object which represents the transfer syntax */
        DcmXfer oxferSyn(oxfer);
        /* if the transfer syntax is one with explicit value representation */
        /* this value's data type also has to be written to the stream. Do so */
        /* and also write the length information to the stream. */
        if (oxferSyn.isExplicitVR())
        {
            /* Create an object that represents this object's data type */
            DcmVR myvr(vr);
            /* get name of data type */
            const char *vrname = myvr.getValidVRName();
            /* write data type name to the stream (a total of 2 bytes) */
            outStream.write(vrname, 2);
            /* create another data type object on the basis of the above created object */
            DcmVR outvr(myvr.getValidEVR());
            /* in case we are dealing with a transfer syntax with explicit VR (see if above) */
            /* and the actual VR uses extended length encoding (see DICOM standard (year 2000) */
            /* part 5, section 7.1.2) (or the corresponding section in a later version of the */
            /* standard) we have to add 2 reserved bytes (set to a value of 00H) to the data */
            /* type field and the actual length field is 4 bytes wide. Write the corresponding */
            /* information to the stream. */
            if (outvr.usesExtendedLengthEncoding())
            {
              Uint16 reserved = 0;
              outStream.write(&reserved, 2);  // write 2 reserved bytes to stream
            }
        }
    }
    /* return result */
    return l_error;
}


OFCondition DcmSequenceOfItems::writeSignatureFormat(DcmOutputStream &outStream,
                                                     const E_TransferSyntax oxfer,
                                                     const E_EncodingType enctype)
{
    if (fTransferState == ERW_notInitialized)
        errorFlag = EC_IllegalCall;
    else
    {
        errorFlag = outStream.status();
        if (errorFlag.good() && fTransferState != ERW_ready)
        {
            if (fTransferState == ERW_init)
            {
                if (outStream.avail() >= DCM_TagInfoLength) // header might be smaller than this
                {
                    if (enctype == EET_ExplicitLength)
                        Length = getLength(oxfer, enctype);
                    else
                        Length = DCM_UndefinedLength;
                    errorFlag = writeTagAndVR(outStream, Tag, getVR(), oxfer);
                    /* we don't write the sequence length */
                    if (errorFlag.good())
                    {
                        fTransferState = ERW_inWork;
                        itemList->seek(ELP_first);
                    }
                } else
                    errorFlag = EC_StreamNotifyClient;
            }
            if (fTransferState == ERW_inWork)
            {
                // itemList->get() can be NULL if buffer was full after
                // writing the last item but before writing the sequence delimitation.
                if (!itemList->empty() && (itemList->get() != NULL))
                {
                    DcmObject *dO;
                    do {
                        dO = itemList->get();
                        if (dO->transferState() != ERW_ready)
                            errorFlag = dO->writeSignatureFormat(outStream, oxfer, enctype);
                    } while (errorFlag.good() && itemList->seek(ELP_next));
                }
                if (errorFlag.good())
                {
                    fTransferState = ERW_ready;
                    /* we always write a sequence delimitation item tag, but no length */
                    if (outStream.avail() >= 4)
                    {
                        // write sequence delimitation item
                        DcmTag delim(DCM_SequenceDelimitationItem);
                        errorFlag = writeTag(outStream, delim, oxfer);
                    } else {
                        // Every subelement of the item was written but it
                        // is not possible to write the delimination item
                        // into the buffer.
                        fTransferState = ERW_inWork;
                        errorFlag = EC_StreamNotifyClient;
                    }
                }
            }
        }
    }
    return errorFlag;
}


// ********************************


void DcmSequenceOfItems::transferInit()
{
    DcmObject::transferInit();
    fStartPosition = 0;
    lastItemComplete = OFTrue;
    if (!itemList->empty())
    {
        itemList->seek(ELP_first);
        do {
            itemList->get()->transferInit();
        } while (itemList->seek(ELP_next));
    }
}


// ********************************


void DcmSequenceOfItems::transferEnd()
{
    DcmObject::transferEnd();
    if (!itemList->empty())
    {
        itemList->seek(ELP_first);
        do {
            itemList->get()->transferEnd();
        } while (itemList->seek(ELP_next));
    }
}


// ********************************


unsigned long DcmSequenceOfItems::card()
{
    return itemList->card();
}


// ********************************


OFCondition DcmSequenceOfItems::prepend(DcmItem *item)
{
    errorFlag = EC_Normal;
    if (item != (DcmItem*)NULL)
        itemList->prepend(item);
    else
        errorFlag = EC_IllegalCall;

    return errorFlag;
}

// ********************************


OFCondition DcmSequenceOfItems::insert(DcmItem *item,
                                       unsigned long where,
                                       OFBool before)
{
    errorFlag = EC_Normal;
    if (item != (DcmItem*)NULL)
    {
        itemList->seek_to(where);
        // insert before or after "where"
        E_ListPos whichSide = (before) ? (ELP_prev) : (ELP_next);
        itemList->insert(item, whichSide);
        if (before)
        {
            debug(3, ("DcmSequenceOfItems::insert() item inserted before position %d", where));
        } else {
            debug(3, ("DcmSequenceOfItems::insert() item inserted after position %d", where));
        }
    } else
        errorFlag = EC_IllegalCall;
    return errorFlag;
}


// ********************************


OFCondition DcmSequenceOfItems::append(DcmItem *item)
{
    errorFlag = EC_Normal;
    if (item != (DcmItem*)NULL)
        itemList->append(item);
    else
        errorFlag = EC_IllegalCall;
    return errorFlag;
}


// ********************************


DcmItem* DcmSequenceOfItems::getItem(const unsigned long num)
{
    errorFlag = EC_Normal;
    DcmItem *item;
    item = (DcmItem*)(itemList->seek_to(num));  // read item from list
    if (item == (DcmItem*)NULL)
        errorFlag = EC_IllegalCall;
    return item;
}


// ********************************


DcmObject *DcmSequenceOfItems::nextInContainer(const DcmObject *obj)
{
    if (!obj)
        return itemList->get(ELP_first);
    else
    {
        if (itemList->get() != obj)
        {
            for (DcmObject *search_obj = itemList -> seek(ELP_first);
                search_obj && search_obj != obj;
                search_obj = itemList -> seek(ELP_next)
               )
            { /* do nothing */ }
        }
        return itemList -> seek(ELP_next);
    }
}


// ********************************


OFCondition DcmSequenceOfItems::nextObject(DcmStack &stack,
                                           const OFBool intoSub)
{
    OFCondition l_error = EC_Normal;
    DcmObject *container = NULL;
    DcmObject *obj = NULL;
    DcmObject *result = NULL;
    OFBool examSub = intoSub;

    if (stack.empty())
    {
        stack.push(this);
        examSub = OFTrue;
    }

    obj = stack.top();
    if (obj->isLeaf() || !intoSub)
    {
        stack.pop();
        if (stack.card() > 0)
        {
            container = stack.top();
            result = container -> nextInContainer(obj);
        }
    } else if (examSub)
        result = obj -> nextInContainer(NULL);

    if (result)
        stack.push(result);
    else if (intoSub)
        l_error = nextUp(stack);
    else
        l_error = EC_SequEnd;

    return l_error;
}


// ********************************


DcmItem *DcmSequenceOfItems::remove(const unsigned long num)
{
    errorFlag = EC_Normal;
    DcmItem *item;
    item = (DcmItem*)(itemList->seek_to(num));  // read item from list
    if (item != (DcmItem*)NULL)
        itemList->remove();
    else
        errorFlag = EC_IllegalCall;
    return item;
}


// ********************************


DcmItem *DcmSequenceOfItems::remove(DcmItem *item)
{
    DcmItem *retItem = (DcmItem*)NULL;
    errorFlag = EC_IllegalCall;
    if (!itemList->empty() && item != (DcmItem*)NULL)
    {
        DcmObject *dO;
        itemList->seek(ELP_first);
        do {
            dO = itemList->get();
            if (dO == item)
            {
                itemList->remove();         // removes element from list but does not delete it
                errorFlag = EC_Normal;
                break;
            }
        } while (itemList->seek(ELP_next));
    }
    if (errorFlag == EC_IllegalCall)
        retItem = (DcmItem*)NULL;
    else
        retItem = item;
    return retItem;
}


// ********************************


OFCondition DcmSequenceOfItems::clear()
{
    errorFlag = EC_Normal;
    DcmObject *dO;
    itemList->seek(ELP_first);
    while (!itemList->empty())
    {
        dO = itemList->remove();
        if (dO != (DcmObject*)NULL)
        {
            delete dO;
            dO = (DcmObject*)NULL;
        }
    }
    Length = 0;
    return errorFlag;
}


// ********************************


OFCondition DcmSequenceOfItems::verify(const OFBool autocorrect)
{
    errorFlag = EC_Normal;
    if (!itemList->empty())
    {
        DcmObject *dO;
        itemList->seek(ELP_first);
        do {
            dO = itemList->get();
            if (dO->verify(autocorrect).bad())
                errorFlag = EC_CorruptedData;
        } while (itemList->seek(ELP_next));
    }
    if (autocorrect == OFTrue)
        Length = getLength();

    return errorFlag;
}


/*
 * precondition: itemList not empty.
 * result:
 *  - element pointer on resultStack if return value is EC_Normal
 *  - unmodified resultStack if return value is EC_TagNotFound
 * continue search: push pointer to sub-element onto resultStack and start sub-search
 */

OFCondition DcmSequenceOfItems::searchSubFromHere(const DcmTagKey &tag,
                                                  DcmStack &resultStack,
                                                  const OFBool searchIntoSub)
{
    DcmObject *dO;
    OFCondition l_error = EC_TagNotFound;
    if (!itemList->empty())
    {
        itemList->seek(ELP_first);
        do {
            dO = itemList->get();
            if (searchIntoSub)
            {
                resultStack.push(dO);
                if (tag == dO->getTag())
                    l_error = EC_Normal;
                else
                    l_error = dO->search(tag, resultStack, ESM_fromStackTop, OFTrue);
                if (l_error.bad())
                    resultStack.pop();
            } else {
                if (tag == dO->getTag())
                {
                    resultStack.push(dO);
                    l_error = EC_Normal;
                }
            }
        } while (l_error.bad() && itemList->seek(ELP_next));
    }
    return l_error;
}


// ********************************


OFCondition DcmSequenceOfItems::search(const DcmTagKey &tag,
                                       DcmStack &resultStack,
                                       E_SearchMode mode,
                                       OFBool searchIntoSub)
{
    DcmObject *dO = (DcmObject*)NULL;
    OFCondition l_error = EC_TagNotFound;
    if (mode == ESM_afterStackTop && resultStack.top() == this)
    {
        l_error = searchSubFromHere(tag, resultStack, searchIntoSub);
    }
    else if (!itemList->empty())
    {
        if (mode == ESM_fromHere || resultStack.empty())
        {
            resultStack.clear();
            l_error = searchSubFromHere(tag, resultStack, searchIntoSub);
        }
        else if (mode == ESM_fromStackTop)
        {
            dO = resultStack.top();
            if (dO == this)
                l_error = searchSubFromHere(tag, resultStack, searchIntoSub);
            else
            {   // continue directly in sub-tree
                l_error = dO->search(tag, resultStack, mode, searchIntoSub);
// The next two lines destroy the stack, delete them
//                if (l_error.bad())          // raeumt nur die oberste Stackebene
//                    resultStack.pop();      // ab; der Rest ist unveraendert
            }
        }
        else if (mode == ESM_afterStackTop && searchIntoSub)
        {
            // resultStack enthaelt Zielinformationen:
            // - stelle Zustand der letzen Suche in den einzelnen Suchroutinen
            //   wieder her
            // - finde Position von dO in Baum-Struktur
            //   1. suche eigenen Stack-Eintrag
            //      - bei Fehlschlag Suche beenden
            //   2. nehme naechsthoeheren Eintrag dnO
            //   3. stelle eigene Liste auf Position von dnO
            //   4. starte Suche ab dnO

            unsigned long i = resultStack.card();
            while (i > 0 && (dO = resultStack.elem(i-1)) != this)
            {
                i--;
            }
            if (dO != this && resultStack.card() > 0)
            {                            // highest level is never in resultStack
                i = resultStack.card()+1;// now points to highest level +1
                dO = this;               // match in highest level
            }
            if (dO == this)
            {
                if (i == 1)                 // resultStack.top() found
                    l_error = EC_TagNotFound; // don't mark as match, see above
                else
                {
                    E_SearchMode submode = mode;
                    OFBool searchNode = OFTrue;
                    DcmObject *dnO;
                    dnO = resultStack.elem(i-2); // node of next level
                    itemList->seek(ELP_first);
                    do {
                        dO = itemList->get();
                        searchNode = searchNode ? (dO != dnO) : OFFalse;
                        if (!searchNode)
                        {                               // continue search
                            if (submode == ESM_fromStackTop)
                                resultStack.push(dO);   // update stack
                            if (submode == ESM_fromStackTop && tag == dO->getTag())
                                l_error = EC_Normal;
                            else
                                l_error = dO->search(tag, resultStack, submode, OFTrue);
                            if (l_error.bad())
                                resultStack.pop();
                            else
                                break;
                            submode = ESM_fromStackTop; // normal search from here
                        }
                    } while (itemList->seek(ELP_next));
                }
            } else                              // probably never reached
                l_error = EC_IllegalCall;
        } // (mode == ESM_afterStackTop
        else
            l_error = EC_IllegalCall;
    }
    return l_error;
}


// ********************************


OFCondition DcmSequenceOfItems::searchErrors(DcmStack &resultStack)
{
    OFCondition l_error = errorFlag;
    DcmObject *dO = (DcmObject*)NULL;
    if (errorFlag.bad())
        resultStack.push(this);
    if (!itemList->empty())
    {
        itemList->seek(ELP_first);
        do {
            OFCondition err = EC_Normal;
            dO = itemList->get();
            if ((err = dO->searchErrors(resultStack)).bad())
                l_error = err;
        } while (itemList->seek(ELP_next));
    }
    return l_error;
}


// ********************************


OFCondition DcmSequenceOfItems::loadAllDataIntoMemory()
{
    OFCondition l_error = EC_Normal;
    if (!itemList->empty())
    {
        itemList->seek(ELP_first);
        do {
            OFCondition err = EC_Normal;
            DcmObject *dO = itemList->get();
            if ((err = dO->loadAllDataIntoMemory()).bad())
                l_error = err;
        } while (itemList->seek(ELP_next));
    }
    return l_error;
}


// ********************************


OFBool DcmSequenceOfItems::isSignable() const
{
    // a sequence is signable if the tag and VR doesn't prevent signing
    // and if none of the items contains a UN element
    return (DcmElement::isSignable() && (! containsUnknownVR()));
}


OFBool DcmSequenceOfItems::containsUnknownVR() const
{
    if (!itemList->empty())
    {
        itemList->seek(ELP_first);
        do {
            if (itemList->get()->containsUnknownVR())
                return OFTrue;
        } while (itemList->seek(ELP_next));
    }
    return OFFalse;
}


/*
** CVS/RCS Log:
** Log: dcsequen.cxx,v 
** Revision 1.2  2004/05/28 17:59:56  peter_vanroose
** typo corrected
**
** Revision 1.1  2004/01/14 04:01:10  amithaperera
** Add better DICOM support by wrapping DCMTK, and add a stripped down
** version of DCMTK to v3p. Add more DICOM test cases.
**
** Revision 1.51  2002/12/10 20:03:01  joergr
** Added curly brackets around debug() call to avoid compiler errors with gcc
** 2.9.5 in debug mode.
**
** Revision 1.50  2002/12/06 13:16:59  joergr
** Enhanced "print()" function by re-working the implementation and replacing
** the boolean "showFullData" parameter by a more general integer flag.
** Made source code formatting more consistent with other modules/files.
**
** Revision 1.49  2002/11/27 12:06:51  meichel
** Adapted module dcmdata to use of new header file ofstdinc.h
**
** Revision 1.48  2002/08/27 16:55:56  meichel
** Initial release of new DICOM I/O stream classes that add support for stream
**   compression (deflated little endian explicit VR transfer syntax)
**
** Revision 1.47  2002/07/08 14:44:41  meichel
** Improved dcmdata behaviour when reading odd tag length. Depending on the
**   global boolean flag dcmAcceptOddAttributeLength, the parser now either accepts
**   odd length attributes or implements the old behaviour, i.e. assumes a real
**   length larger by one.
**
** Revision 1.46  2002/04/25 10:26:10  joergr
** Added support for XML output of DICOM objects.
**
** Revision 1.45  2002/04/16 13:43:20  joergr
** Added configurable support for C++ ANSI standard includes (e.g. streams).
** Thanks to Andreas Barth <Andreas.Barth@bruker-biospin.de> for his
** contribution.
**
** Revision 1.44  2002/03/15 13:58:39  meichel
** Fixed incorrect debug message.
**
** Revision 1.43  2001/11/16 15:55:04  meichel
** Adapted digital signature code to final text of supplement 41.
**
** Revision 1.42  2001/09/28 14:21:06  joergr
** Added "#include <iomanip.h>" to keep gcc 3.0 quiet.
**
** Revision 1.41  2001/09/26 15:49:30  meichel
** Modified debug messages, required by OFCondition
**
** Revision 1.40  2001/09/25 17:19:53  meichel
** Adapted dcmdata to class OFCondition
**
** Revision 1.39  2001/06/01 15:49:08  meichel
** Updated copyright header
**
** Revision 1.38  2001/05/10 12:46:28  meichel
** Fixed memory leak that occurred when parsing of a sequence failed.
**
** Revision 1.37  2001/05/03 08:15:23  meichel
** Fixed bug in dcmdata sequence handling code that could lead to application
**   failure in rare cases during parsing of a correct DICOM dataset.
**
** Revision 1.36  2000/11/07 16:56:22  meichel
** Initial release of dcmsign module for DICOM Digital Signatures
**
** Revision 1.35  2000/04/14 15:55:07  meichel
** Dcmdata library code now consistently uses ofConsole for error output.
**
** Revision 1.34  2000/03/08 16:26:40  meichel
** Updated copyright header.
**
** Revision 1.33  2000/03/03 15:02:11  joergr
** Corrected bug related to padding of file and item size.
**
** Revision 1.32  2000/03/03 14:05:36  meichel
** Implemented library support for redirecting error messages into memory
**   instead of printing them to stdout/stderr for GUI applications.
**
** Revision 1.31  2000/02/23 15:12:01  meichel
** Corrected macro for Borland C++ Builder 4 workaround.
**
** Revision 1.30  2000/02/10 10:52:22  joergr
** Added new feature to dcmdump (enhanced print method of dcmdata): write
** pixel data/item value fields to raw files.
**
** Revision 1.29  2000/02/03 16:29:40  joergr
** Corrected bug that caused wrong calculation of group length for sequence
** of items (e.g. encapsulated pixel data).
**
** Revision 1.28  2000/02/01 10:12:10  meichel
** Avoiding to include <stdlib.h> as extern "C" on Borland C++ Builder 4,
**   workaround for bug in compiler header files.
**
** Revision 1.27  1999/03/31 09:25:38  meichel
** Updated copyright header in module dcmdata
**
** Revision 1.26  1998/11/12 16:48:20  meichel
** Implemented operator= for all classes derived from DcmObject.
**
** Revision 1.25  1998/07/15 15:52:06  joergr
** Removed several compiler warnings reported by gcc 2.8.1 with
** additional options, e.g. missing copy constructors and assignment
** operators, initialization of member variables in the body of a
** constructor instead of the member initialization list, hiding of
** methods by use of identical names, uninitialized member variables,
** missing const declaration of char pointers. Replaced tabs by spaces.
**
** Revision 1.24  1998/01/27 10:49:26  meichel
** Minor bug corrections (string too short, incorrect return value).
**   Thanks to Andreas Barth <anba@bruker.de> for the report.
**
** Revision 1.23  1997/09/12 13:44:54  meichel
** The algorithm introduced on 97.08.28 to detect incorrect odd-length
**   value fields falsely reported undefined length sequences and items
**   to be wrong. Corrected.
**
** Revision 1.22  1997/08/29 07:53:24  andreas
** - New error messages if length of an element is odd. Previously, no
**   error was reported. But the length is corrected by the method
**   newValueField and so it was impossible for a checking utility to find
**   such an error in DICOM objects.
**
** Revision 1.21  1997/07/21 08:25:29  andreas
** - Replace all boolean types (BOOLEAN, CTNBOOLEAN, DICOM_BOOL, BOOL)
**   with one unique boolean type OFBool.
**
** Revision 1.20  1997/07/07 07:46:20  andreas
** - Changed parameter type DcmTag & to DcmTagKey & in all search functions
**   in DcmItem, DcmSequenceOfItems, DcmDirectoryRecord and DcmObject
** - Enhanced (faster) byte swapping routine. swapIfNecessary moved from
**   a method in DcmObject to a general function.
**
** Revision 1.19  1997/07/03 15:10:04  andreas
** - removed debugging functions Bdebug() and Edebug() since
**   they write a static array and are not very useful at all.
**   Cdebug and Vdebug are merged since they have the same semantics.
**   The debugging functions in dcmdata changed their interfaces
**   (see dcmdata/include/dcdebug.h)
**
** Revision 1.18  1997/06/06 09:55:31  andreas
** - corrected error: canWriteXfer returns false if the old transfer syntax
**   was unknown, which causes several applications to prohibit the writing
**   of dataset.
**
** Revision 1.17  1997/05/30 06:45:45  andreas
** - fixed problem of inconsistent interfaces and implementation that the
**   syntax check of GNU C++ does not find.
**
** Revision 1.16  1997/05/27 13:49:02  andreas
** - Add method canWriteXfer to class DcmObject and all derived classes.
**   This method checks whether it is possible to convert the original
**   transfer syntax to an new transfer syntax. The check is used in the
**   dcmconv utility to prohibit the change of a compressed transfer
**   syntax to a uncompressed.
**
** Revision 1.15  1997/05/16 08:23:55  andreas
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
** Revision 1.14  1997/04/24 12:11:49  hewett
** Fixed DICOMDIR generation bug affecting ordering of
** patient/study/series/image records (item insertion into a sequence
** did produce the expected ordering).
**
** Revision 1.13  1997/04/18 08:08:54  andreas
** - Corrected debugging code
**
** Revision 1.12  1996/09/13 12:04:13  hewett
** Corrected missing () in function call (stack.card()) used in nextObject(...)
**
** Revision 1.11  1996/08/08 10:15:10  andreas
** Some more testing in nextObject
**
** Revision 1.10  1996/08/08 10:06:24  andreas
** Correct error for intoSub=OFFalse
**
** Revision 1.9  1996/08/05 08:46:17  andreas
** new print routine with additional parameters:
**         - print into files
**         - fix output length for elements
** corrected error in search routine with parameter ESM_fromStackTop
**
** Revision 1.8  1996/07/31 13:14:32  andreas
** - Minor corrections: error code for swapping to or from byteorder unknown
**                      correct read of dataset in fileformat
**
** Revision 1.7  1996/07/17 12:39:40  andreas
** new nextObject for DcmDataSet, DcmFileFormat, DcmItem, ...
**
** Revision 1.6  1996/06/19 13:54:10  andreas
** - correct error when reading big sequences with little buffers from net
**
** Revision 1.5  1996/01/29 13:38:29  andreas
** - new put method for every VR to put value as a string
** - better and unique print methods
**
** Revision 1.4  1996/01/09 11:06:48  andreas
** New Support for Visual C++
** Correct problems with inconsistent const declarations
** Correct error in reading Item Delimitation Elements
**
** Revision 1.3  1996/01/05 13:27:41  andreas
** - changed to support new streaming facilities
** - unique read/write methods for file and block transfer
** - more cleanups
**
*/

