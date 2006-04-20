/*
 *
 *  Copyright (C) 1996-2001, OFFIS
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
 *  Module:  dcmimgle
 *
 *  Author:  Joerg Riesmeier
 *
 *  Purpose: DicomMonochromeRotateTemplate (Header)
 *
 *  Last Update:      Author: peter_vanroose 
 *  Update Date:      Date: 2004/08/04 10:36:46 
 *  Source File:      Source: /cvsroot/vxl/vxl/v3p/dcmtk/dcmimgle/include/dimorot.h,v 
 *  CVS/RCS Revision: Revision: 1.2 
 *  Status:           State: Exp 
 *
 *  CVS/RCS Log at end of file
 *
 */


#ifndef __DIMOROT_H
#define __DIMOROT_H

#include "osconfig.h"
#include "dctypes.h"

#include "dimopxt.h"
#include "dirotat.h"


/*---------------------*
 *  class declaration  *
 *---------------------*/

/** Template class to rotate monochrome images (on pixel data level).
 *  by steps of 90 degrees
 */
template<class T>
class DiMonoRotateTemplate
  : public DiMonoPixelTemplate<T>,
    protected DiRotateTemplate<T>
{

 public:

    /** constructor
     *
     ** @param  pixel      pointer to intermediate pixel representation
     *  @param  src_cols   width of clipping area
     *  @param  src_rows   height of clipping area
     *  @param  dest_cols  width of destination image (scaled image)
     *  @param  dest_rows  height of destination image
     *  @param  frames     number of frames
     *  @param  degree     angle by which the pixel data should be rotated
     */
    DiMonoRotateTemplate(const DiMonoPixel *pixel,
                         const Uint16 src_cols,
                         const Uint16 src_rows,
                         const Uint16 dest_cols,
                         const Uint16 dest_rows,
                         const Uint32 frames,
                         const int degree)
      : DiMonoPixelTemplate<T>(pixel, (unsigned long)dest_cols * (unsigned long)dest_rows * frames),
        DiRotateTemplate<T>(1, src_cols, src_rows, dest_cols, dest_rows, frames)
    {
        if ((pixel != NULL) && (pixel->getCount() > 0))
        {
            if (pixel->getCount() == (unsigned long)src_cols * (unsigned long)src_rows * frames)
                rotate((const T *)pixel->getData(), degree);
            else {
                if (DicomImageClass::checkDebugLevel(DicomImageClass::DL_Warnings))
                {
                   ofConsole.lockCerr() << "WARNING: could not rotate image ... corrupted data." << endl;
                   ofConsole.unlockCerr();
                }
            }
        }
    }

    /** destructor
     */
    ~DiMonoRotateTemplate()
    {
    }


 private:

    /** rotate pixel data
     *
     ** @param  pixel   pointer to pixel data to be rotated
     *  @param  degree  angle by which the pixel data should be rotated
     */
    inline void rotate(const T *pixel,
                       const int degree)
    {
        if (pixel != NULL)
        {
            this->Data = new T[this->getCount()];
            if (this->Data != NULL)
            {
                if (degree == 90)
                    rotateRight(&pixel, &this->Data);
                else if (degree == 180)
                    rotateTopDown(&pixel, &this->Data);
                else if (degree == 270)
                    rotateLeft(&pixel, &this->Data);
            }
        }
    }
};


#endif


/*
 *
 * CVS/RCS Log:
 * Log: dimorot.h,v 
 * Revision 1.2  2004/08/04 10:36:46  peter_vanroose
 * fix for gcc 3.4 (missing "this->")
 *
 * Revision 1.1  2004/01/14 04:01:10  amithaperera
 * Add better DICOM support by wrapping DCMTK, and add a stripped down
 * version of DCMTK to v3p. Add more DICOM test cases.
 *
 * Revision 1.8  2001/06/01 15:49:48  meichel
 * Updated copyright header
 *
 * Revision 1.7  2000/09/12 10:04:45  joergr
 * Corrected bug: wrong parameter for attribute search routine led to crashes
 * when multiple pixel data attributes were contained in the dataset (e.g.
 * IconImageSequence). Added new checking routines to avoid crashes when
 * processing corrupted image data.
 *
 * Revision 1.6  2000/03/08 16:24:21  meichel
 * Updated copyright header.
 *
 * Revision 1.5  1999/09/17 12:43:23  joergr
 * Added/changed/completed DOC++ style comments in the header files.
 *
 * Revision 1.4  1999/03/24 17:20:17  joergr
 * Added/Modified comments and formatting.
 *
 * Revision 1.3  1999/02/11 16:40:46  joergr
 * Corrected some typos and formatting.
 *
 * Revision 1.2  1998/12/14 17:26:26  joergr
 * Corrected a typo.
 *
 * Revision 1.1  1998/11/27 14:57:48  joergr
 * Added copyright message.
 * Added methods and classes for flipping and rotating, changed for
 * scaling and clipping.
 *
 *
 */
