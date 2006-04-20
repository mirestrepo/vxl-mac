/*
 *
 *  Copyright (C) 1996-2002, OFFIS
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
 *  Purpose: DicomInputPixel (Header)
 *
 *  Last Update:      Author: amithaperera 
 *  Update Date:      Date: 2004/01/14 04:01:10 
 *  Source File:      Source: /cvsroot/vxl/vxl/v3p/dcmtk/dcmimgle/include/diinpx.h,v 
 *  CVS/RCS Revision: Revision: 1.1 
 *  Status:           State: Exp 
 *
 *   CVS/RCS Log at end of file
 *
 */


#ifndef __DIINPX_H
#define __DIINPX_H

#include "osconfig.h"
#include "ofconsol.h"
#include "dctypes.h"

#include "diutils.h"


/*---------------------*
 *  class declaration  *
 *---------------------*/

/** Abstract base class to convert DICOM pixel stream to intermediate representation
 */
class DiInputPixel
{

 public:

    /** constructor
     *
     ** @param  bits   number of bits stored for each pixel (depth)
     *  @param  start  start position of pixel data to be processed
     *  @param  count  number of pixels to be processed
     */
    DiInputPixel(const unsigned int bits,
                 const unsigned long start,
                 const unsigned long count);

    /** destructor
     */
    virtual ~DiInputPixel();

    /** determine minimum and maximum pixel value (abstract)
     *
     ** @return status, true if successful, false otherwise
     */
    virtual int determineMinMax() = 0;

    /** get pixel representation (abstract).
     *  Determine which integer type (size and signed/unsigned) is necessary to store
     *  the pixel data.
     *
     ** @return pixel representation
     */
    virtual EP_Representation getRepresentation() const = 0;

    /** get pointer to input pixel data (abstract)
     *
     ** @return pointer to input pixel data
     */
    virtual void *getData() const = 0;

    /** remove reference to (internally handled) pixel data (abstract)
     */
    virtual void removeDataReference() = 0;

    /** get minimum pixel value (abstract)
     *
     ** @param  idx  specifies whether to return the global minimum (0) or
     *               the minimum of the selected pixel range (1, see PixelStart/Range)
     *
     ** @return minimum pixel value
     */
    virtual double getMinValue(const int idx) const = 0;

    /** get maximum pixel value (abstract)
     *
     ** @param  idx  specifies whether to return the global maximum (0) or
     *               the maximum of the selected pixel range (1, see PixelStart/Range)
     *
     ** @return maximum pixel value
     */
    virtual double getMaxValue(const int idx) const = 0;

    /** get number of bits per pixel
     *
     ** @return number of bits per pixel
     */
    inline unsigned int getBits() const
    {
        return Bits;
    }

    /** get absolute minimum pixel value
     *
     ** @return absolute minimum pixel value
     */
    inline double getAbsMinimum() const
    {
        return AbsMinimum;
    }

    /** get absolute maximum pixel value
     *
     ** @return absolute maximum pixel value
     */
    inline double getAbsMaximum() const
    {
        return AbsMaximum;
    }

    /** get absolute pixel value range
     *
     ** @return absolute pixel value range
     */
    inline double getAbsMaxRange() const
    {
        return AbsMaximum - AbsMinimum + 1;
    }

    /** get number of pixels stored
     *
     ** @return number of pixels stored
     */
    inline unsigned long getCount() const
    {
        return Count;
    }

    /** get first pixel to be processed
     *
     ** @return first pixel to be processed
     */
    inline unsigned long getPixelStart() const
    {
        return PixelStart;
    }

    /** get number of pixels to be processed
     *
     ** @return number of pixels to be processed
     */
    inline unsigned long getPixelCount() const
    {
        return PixelCount;
    }

    /** get number of pixels computed from the image resolution
     *
     ** @return number of pixels computed
     */
    inline unsigned long getComputedCount() const
    {
        return ComputedCount;
    }


 protected:

    /// number of pixels stored
    unsigned long Count;
    /// bits per pixel/sample
    unsigned int Bits;

    /// first pixel to be processed
    unsigned long PixelStart;
    /// number of pixels to be processed
    unsigned long PixelCount;

    /// number of pixels computed from the image resolution
    unsigned long ComputedCount;

    /// absolute minimum (possible) pixel value
    double AbsMinimum;
    /// absolute maximum (possible) pixel value
    double AbsMaximum;
};


#endif


/*
 *
 * CVS/RCS Log:
 * Log: diinpx.h,v 
 * Revision 1.1  2004/01/14 04:01:10  amithaperera
 * Add better DICOM support by wrapping DCMTK, and add a stripped down
 * version of DCMTK to v3p. Add more DICOM test cases.
 *
 * Revision 1.14  2002/06/26 16:02:31  joergr
 * Enhanced handling of corrupted pixel data and/or length.
 *
 * Revision 1.13  2001/09/28 13:04:58  joergr
 * Enhanced algorithm to determine the min and max value.
 *
 * Revision 1.12  2001/06/01 15:49:42  meichel
 * Updated copyright header
 *
 * Revision 1.11  2000/04/27 13:08:38  joergr
 * Dcmimgle library code now consistently uses ofConsole for error output.
 *
 * Revision 1.10  2000/03/08 16:24:16  meichel
 * Updated copyright header.
 *
 * Revision 1.9  2000/03/03 14:09:11  meichel
 * Implemented library support for redirecting error messages into memory
 *   instead of printing them to stdout/stderr for GUI applications.
 *
 * Revision 1.8  1999/09/17 12:13:18  joergr
 * Added/changed/completed DOC++ style comments in the header files.
 *
 * Revision 1.7  1999/07/23 13:54:37  joergr
 * Optimized memory usage for converting input pixel data (reference instead
 * of copying where possible).
 *
 * Revision 1.6  1999/03/24 17:20:02  joergr
 * Added/Modified comments and formatting.
 *
 * Revision 1.5  1999/02/03 17:03:47  joergr
 * Added member variable and related methods to store number of bits used for
 * pixel data.
 *
 * Revision 1.4  1999/01/20 15:00:54  joergr
 * Added routine to calculate absolute range of pixel data.
 *
 * Revision 1.3  1998/12/22 14:18:40  joergr
 * Added implementation of methods to return member variables AbsMinimum/
 * Maximum.
 *
 * Revision 1.2  1998/12/16 16:30:34  joergr
 * Added methods to determine absolute minimum and maximum value for given
 * value representation.
 *
 * Revision 1.1  1998/11/27 15:06:38  joergr
 * Added copyright message.
 *
 * Revision 1.3  1998/05/11 14:53:17  joergr
 * Added CVS/RCS header to each file.
 *
 *
 */
