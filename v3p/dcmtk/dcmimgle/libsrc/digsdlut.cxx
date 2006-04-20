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
 *  Purpose: DicomGSDFLUT (Source)
 *
 *  Last Update:      Author: amithaperera 
 *  Update Date:      Date: 2004/01/14 04:01:11 
 *  Source File:      Source: /cvsroot/vxl/vxl/v3p/dcmtk/dcmimgle/libsrc/digsdlut.cxx,v 
 *  CVS/RCS Revision: Revision: 1.1 
 *  Status:           State: Exp 
 *
 *  CVS/RCS Log at end of file
 *
 */


#include "osconfig.h"

#include "ofconsol.h"
#include "digsdlut.h"
#include "displint.h"

#define INCLUDE_CMATH
#include "ofstdinc.h"


/*----------------*
 *  constructors  *
 *----------------*/

DiGSDFLUT::DiGSDFLUT(const unsigned long count,
                     const Uint16 max,
                     const Uint16 *ddl_tab,
                     const double *val_tab,
                     const unsigned long ddl_cnt,
                     const double *gsdf_tab,
                     const double *gsdf_spl,
                     const unsigned int gsdf_cnt,
                     const double jnd_min,
                     const double jnd_max,
                     const double amb,
                     const double illum,
                     const OFBool inverse,
                     ostream *stream,
                     const OFBool printMode)
  : DiDisplayLUT(count, max, amb, illum)
{
    if ((Count > 0) && (Bits > 0))
    {
#ifdef DEBUG
        if (DicomImageClass::checkDebugLevel(DicomImageClass::DL_Informationals))
        {
            ofConsole.lockCerr() << "INFO: new GSDF LUT with " << Bits << " bits output and "
                                 << Count << " entries created !" << endl;
            ofConsole.unlockCerr();
        }
#endif
        Valid = createLUT(ddl_tab, val_tab, ddl_cnt, gsdf_tab, gsdf_spl, gsdf_cnt, jnd_min, jnd_max,
                          inverse, stream, printMode);
    }
}


/*--------------*
 *  destructor  *
 *--------------*/

DiGSDFLUT::~DiGSDFLUT()
{
}


/********************************************************************/


int DiGSDFLUT::createLUT(const Uint16 *ddl_tab,
                         const double *val_tab,
                         const unsigned long ddl_cnt,
                         const double *gsdf_tab,
                         const double *gsdf_spl,
                         const unsigned int gsdf_cnt,
                         const double jnd_min,
                         const double jnd_max,
                         const OFBool inverse,
                         ostream *stream,
                         const OFBool printMode)
{
    if ((ddl_tab != NULL) && (val_tab != NULL) && (ddl_cnt > 0) && (gsdf_tab != NULL) && (gsdf_spl != NULL) && (gsdf_cnt > 0))
    {
        int status = 0;
        const unsigned long gin_ctn = (inverse) ? ddl_cnt : Count;      // number of points to be interpolated
        double *jidx = new double[gin_ctn];
        if (jidx != NULL)
        {
            const double dist = (jnd_max - jnd_min) / (gin_ctn - 1);    // distance between two entries
            register unsigned long i;
            register double *s = jidx;
            register double value = jnd_min;                            // first value is fixed !
            for (i = gin_ctn; i > 1; i--)                               // initialize scaled JND index array
            {
                *(s++) = value;
                value += dist;                                          // add step by step ...
            }
            *s = jnd_max;                                               // last value is fixed !
            double *jnd_idx = new double[gsdf_cnt];
            if (jnd_idx != NULL)
            {
                s = jnd_idx;
                for (i = 0; i < gsdf_cnt; i++)                          // initialize JND index array
                    *(s++) = i + 1;
                double *gsdf = new double[gin_ctn];                     // interpolated GSDF
                if (gsdf != NULL)
                {
                    if (DiCubicSpline<double, double>::Interpolation(jnd_idx, gsdf_tab, gsdf_spl, gsdf_cnt, jidx, gsdf, (unsigned int)gin_ctn))
                    {
                        DataBuffer = new Uint16[Count];
                        if (DataBuffer != NULL)
                        {
                            const double amb = getAmbientLightValue();
                            register Uint16 *q = DataBuffer;
                            register unsigned long j = 0;
                            if (inverse)
                            {
                                register double v;
                                const double factor = (double)(ddl_cnt - 1) / (double)(Count - 1);
                                /* convert DDL to P-Value */
                                for (i = 0; i < Count; i++)
                                {
                                    v = val_tab[(int)(i * factor)] + amb;                 // need to scale index to range of value table
                                    while ((j + 1 < ddl_cnt) && (gsdf[j] < v))            // search for closest index, assuming monotony
                                        j++;
                                    if ((j > 0) && (fabs(gsdf[j - 1] - v) < fabs(gsdf[j] - v)))
                                        j--;
                                    *(q++) = ddl_tab[j];
                                }
                            } else {
                                register const double *r = gsdf;
                                /* convert P-Value to DDL */
                                for (i = Count; i != 0; i--, r++)
                                {
                                    while ((j + 1 < ddl_cnt) && (val_tab[j] + amb < *r))  // search for closest index, assuming monotony
                                        j++;
                                    if ((j > 0) && (fabs(val_tab[j - 1] + amb - *r) < fabs(val_tab[j] + amb - *r)))
                                        j--;
                                    *(q++) = ddl_tab[j];
                                }
                            }
                            Data = DataBuffer;
                            if (stream != NULL)                         // write curve data to file
                            {
                                if (Count == ddl_cnt)                   // check whether GSDF LUT fits exactly to DISPLAY file
                                {
                                    for (i = 0; i < ddl_cnt; i++)
                                    {
                                        (*stream) << ddl_tab[i];                               // DDL
                                        stream->setf(ios::fixed, ios::floatfield);
                                        if (printMode)
                                            (*stream) << "\t" << val_tab[i] + amb;             // CC
                                        (*stream) << "\t" << gsdf[i];                          // GSDF
                                        if (printMode)
                                        {
                                            if (inverse)
                                                (*stream) << "\t" << gsdf[Data[i]];            // PSC'
                                            else
                                                (*stream) << "\t" << val_tab[Data[i]] + amb;   // PSC
                                        }
                                        (*stream) << endl;
                                    }
                                } else {
                                    if (DicomImageClass::checkDebugLevel(DicomImageClass::DL_Warnings))
                                    {
                                        ofConsole.lockCerr() << "WARNING: can't write curve data, "
                                                             << "wrong DISPLAY file or GSDF LUT !" << endl;
                                        ofConsole.unlockCerr();
                                    }
                                }
                            }
                            status = 1;
                        }
                    }
                }
                delete[] gsdf;
            }
            delete[] jnd_idx;
        }
        delete[] jidx;
        return status;
    }
    return 0;
}


/*
 *
 * CVS/RCS Log:
 * Log: digsdlut.cxx,v 
 * Revision 1.1  2004/01/14 04:01:11  amithaperera
 * Add better DICOM support by wrapping DCMTK, and add a stripped down
 * version of DCMTK to v3p. Add more DICOM test cases.
 *
 * Revision 1.14  2002/11/27 14:08:12  meichel
 * Adapted module dcmimgle to use of new header file ofstdinc.h
 *
 * Revision 1.13  2002/07/19 13:10:39  joergr
 * Enhanced handling of "inverse" calibration used for input devices.
 *
 * Revision 1.12  2002/07/18 12:35:26  joergr
 * Added support for hardcopy and softcopy input devices (camera and scanner).
 *
 * Revision 1.11  2002/07/03 13:51:00  joergr
 * Fixed inconsistencies regarding the handling of ambient light.
 *
 * Revision 1.10  2002/07/02 16:24:38  joergr
 * Added support for hardcopy devices to the calibrated output routines.
 *
 * Revision 1.9  2001/06/01 15:49:55  meichel
 * Updated copyright header
 *
 * Revision 1.8  2000/05/03 09:47:23  joergr
 * Removed most informational and some warning messages from release built
 * (#ifndef DEBUG).
 *
 * Revision 1.7  2000/04/28 12:33:43  joergr
 * DebugLevel - global for the module - now derived from OFGlobal (MF-safe).
 *
 * Revision 1.6  2000/04/27 13:10:27  joergr
 * Dcmimgle library code now consistently uses ofConsole for error output.
 *
 * Revision 1.5  2000/03/08 16:24:28  meichel
 * Updated copyright header.
 *
 * Revision 1.4  2000/03/03 14:09:18  meichel
 * Implemented library support for redirecting error messages into memory
 *   instead of printing them to stdout/stderr for GUI applications.
 *
 * Revision 1.3  1999/10/18 15:06:25  joergr
 * Enhanced command line tool dcmdspfn (added new options).
 *
 * Revision 1.2  1999/09/17 13:13:30  joergr
 * Enhanced efficiency of some "for" loops.
 *
 * Revision 1.1  1999/09/10 08:54:50  joergr
 * Added support for CIELAB display function. Restructured class hierarchy
 * for display functions.
 *
 *
 */
