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
 *  Purpose: DicomDisplayFunction (Source)
 *
 *  Last Update:      Author: peter_vanroose 
 *  Update Date:      Date: 2004/05/28 17:59:56 
 *  Source File:      Source: /cvsroot/vxl/vxl/v3p/dcmtk/dcmimgle/libsrc/didispfn.cxx,v 
 *  CVS/RCS Revision: Revision: 1.3 
 *  Status:           State: Exp 
 *
 *  CVS/RCS Log at end of file
 *
 */


#include "osconfig.h"
#include "ofconsol.h"
#include "ofbmanip.h"
#include "didispfn.h"
#include "displint.h"
#include "dicrvfit.h"
#include "ofstream.h"

#define INCLUDE_CCTYPE
#define INCLUDE_CMATH
#include "ofstdinc.h"

/*----------------------------*
 *  constant initializations  *
 *----------------------------*/

const int DiDisplayFunction::MinBits = 2;
const int DiDisplayFunction::MaxBits = 16;


/*----------------*
 *  constructors  *
 *----------------*/

DiDisplayFunction::DiDisplayFunction(const char *filename,
                                     const E_DeviceType deviceType,
                                     const signed int ord)
  : Valid(0),
    DeviceType(deviceType),
    ValueCount(0),
    MaxDDLValue(0),
    Order(0),
    AmbientLight(0),
    Illumination(0),
    DDLValue(NULL),
    LODValue(NULL),
    MinValue(0),
    MaxValue(0)
{
    OFBitmanipTemplate<DiDisplayLUT *>::zeroMem(LookupTable, MAX_NUMBER_OF_TABLES);
    if (readConfigFile(filename))
    {
        /* overwrite file setting for polynomial order */
        if (ord >= 0)
            Order = ord;
        Valid = createSortedTable(DDLValue, LODValue) && calculateMinMax() && interpolateValues();
    }
}


DiDisplayFunction::DiDisplayFunction(const double *val_tab,             // UNTESTED !!
                                     const unsigned long count,
                                     const Uint16 max,
                                     const E_DeviceType deviceType,
                                     const signed int ord)
  : Valid(0),
    DeviceType(deviceType),
    ValueCount(count),
    MaxDDLValue(max),
    Order(ord),
    AmbientLight(0),
    Illumination(0),
    DDLValue(NULL),
    LODValue(NULL),
    MinValue(0),
    MaxValue(0)
{
    OFBitmanipTemplate<DiDisplayLUT *>::zeroMem(LookupTable, MAX_NUMBER_OF_TABLES);
    /* check number of entries */
    if ((ValueCount > 0) && (ValueCount == (unsigned long)MaxDDLValue + 1))
    {
        /* copy value table */
        DDLValue = new Uint16[ValueCount];
        LODValue = new double[ValueCount];
        if ((DDLValue != NULL) && (LODValue != NULL))
        {
            register unsigned int i;
            for (i = 0; i <= MaxDDLValue; i++)
            {
                DDLValue[i] = (Uint16)i;                    // set DDL values
                LODValue[i] = val_tab[i];                   // copy table
            }
            Valid = calculateMinMax();
        }
    }
}


DiDisplayFunction::DiDisplayFunction(const Uint16 *ddl_tab,             // UNTESTED !!
                                     const double *val_tab,
                                     const unsigned long count,
                                     const Uint16 max,
                                     const E_DeviceType deviceType,
                                     const signed int ord)
  : Valid(0),
    DeviceType(deviceType),
    ValueCount(count),
    MaxDDLValue(max),
    Order(ord),
    AmbientLight(0),
    Illumination(0),
    DDLValue(NULL),
    LODValue(NULL),
    MinValue(0),
    MaxValue(0)
{
    OFBitmanipTemplate<DiDisplayLUT *>::zeroMem(LookupTable, MAX_NUMBER_OF_TABLES);
    /* check for maximum number of entries */
    if (ValueCount <= MAX_TABLE_ENTRY_COUNT)
        Valid = createSortedTable(ddl_tab, val_tab) && calculateMinMax() && interpolateValues();
}


DiDisplayFunction::DiDisplayFunction(const double val_min,
                                     const double val_max,
                                     const unsigned long count,
                                     const E_DeviceType deviceType,
                                     const signed int ord)
  : Valid(0),
    DeviceType(deviceType),
    ValueCount(count),
    MaxDDLValue(0),
    Order(ord),
    AmbientLight(0),
    Illumination(0),
    DDLValue(NULL),
    LODValue(NULL),
    MinValue(val_min),
    MaxValue(val_max)
{
    OFBitmanipTemplate<DiDisplayLUT *>::zeroMem(LookupTable, MAX_NUMBER_OF_TABLES);
    /* check parameters */
    if ((ValueCount > 1) && (ValueCount <= MAX_TABLE_ENTRY_COUNT) && (MinValue < MaxValue))
    {
        /* create value tables */
        MaxDDLValue = (Uint16)(count - 1);
        DDLValue = new Uint16[ValueCount];
        LODValue = new double[ValueCount];
        if ((DDLValue != NULL) && (LODValue != NULL))
        {
            register Uint16 i;
            const double val = (val_max - val_min) / (double)MaxDDLValue;
            DDLValue[0] = 0;
            LODValue[0] = val_min;
            for (i = 1; i < MaxDDLValue; i++)
            {
                DDLValue[i] = i;                            // set DDL values
                LODValue[i] = LODValue[i - 1] + val;        // compute luminance/OD value
            }
            DDLValue[MaxDDLValue] = MaxDDLValue;
            LODValue[MaxDDLValue] = val_max;
            Valid = 1;
        }
    }
}


/*--------------*
 *  destructor  *
 *--------------*/

DiDisplayFunction::~DiDisplayFunction()
{
    delete[] DDLValue;
    delete[] LODValue;
    register int i;
    for (i = 0; i < MAX_NUMBER_OF_TABLES; i++)
        delete LookupTable[i];
}


/********************************************************************/


const DiDisplayLUT *DiDisplayFunction::getLookupTable(const int bits,
                                                      unsigned long count)
{
    if (Valid && (bits >= MinBits) && (bits <= MaxBits))
    {
        const int idx = bits - MinBits;
        /* automatically compute number of entries */
        if (count == 0)
            count = DicomImageClass::maxval(bits, 0);
        /* check whether existing LUT is still valid */
        if ((LookupTable[idx] != NULL) && ((count != LookupTable[idx]->getCount()) ||
            (AmbientLight != LookupTable[idx]->getAmbientLightValue()) ||
            (Illumination != LookupTable[idx]->getIlluminationValue())))
        {
            delete LookupTable[idx];
            LookupTable[idx] = NULL;
        }
        if (LookupTable[idx] == NULL)                             // first calculation of this LUT
            LookupTable[idx] = getDisplayLUT(count);
        return LookupTable[idx];
    }
    return NULL;
}


int DiDisplayFunction::deleteLookupTable(const int bits)
{
    if (bits == 0)
    {
        /* delete all LUTs */
        register int i;
        for (i = 0; i < MAX_NUMBER_OF_TABLES; i++)
        {
            delete LookupTable[i];
            LookupTable[i] = NULL;
        }
        return 1;
    }
    else if ((bits >= MinBits) && (bits <= MaxBits))
    {
        /* delete the specified LUT */
        const int idx = bits - MinBits;
        if (LookupTable[idx] != NULL)
        {
            delete LookupTable[idx];
            LookupTable[idx] = NULL;
            return 1;
        }
        return 2;
    }
    return 0;
}


int DiDisplayFunction::setAmbientLightValue(const double value)
{
    if (value >= 0)
    {
        AmbientLight = value;
        return 1;
    }
    return 0;
}


int DiDisplayFunction::setIlluminationValue(const double value)
{
    if (value >= 0)
    {
        Illumination = value;
        return 1;
    }
    return 0;
}


/********************************************************************/


int DiDisplayFunction::readConfigFile(const char *filename)
{
    if ((filename != NULL) && (strlen(filename) > 0))
    {
#ifdef HAVE_IOS_NOCREATE
        ifstream file(filename, ios::in|ios::nocreate);
#else
        ifstream file(filename, ios::in);
#endif
        if (file)
        {
            char c;
            while (file.get(c))
            {
                if (c == '#')                                               // comment character
                {
                    while (file.get(c) && (c != '\n') && (c != '\r'));      // skip comments
                }
                else if (!isspace(c))                                       // skip whitespaces
                {
                    file.putback(c);
                    if (MaxDDLValue == 0)                                   // read maxvalue
                    {
                        char str[4];
                        file.get(str, sizeof(str));
                        if (strcmp(str, "max") == 0)                        // check for key word: max
                        {
                            file >> MaxDDLValue;
                            if (MaxDDLValue > 0)
                            {
                                DDLValue = new Uint16[(unsigned long)MaxDDLValue + 1];
                                LODValue = new double[(unsigned long)MaxDDLValue + 1];
                                if ((DDLValue == NULL) || (LODValue == NULL))
                                    return 0;
                            } else {
                                if (DicomImageClass::checkDebugLevel(DicomImageClass::DL_Errors))
                                {
                                    ofConsole.lockCerr() << "ERROR: invalid or missing value for maximum DDL value in DISPLAY file !" << endl;
                                    ofConsole.unlockCerr();
                                }
                                return 0;                                   // abort
                            }
                        } else {
                            if (DicomImageClass::checkDebugLevel(DicomImageClass::DL_Errors))
                            {
                                ofConsole.lockCerr() << "ERROR: missing keyword 'max' for maximum DDL value in DISPLAY file !" << endl;
                                ofConsole.unlockCerr();
                            }
                            return 0;                                       // abort
                        }
                    }
                    else if ((AmbientLight == 0.0) && (c == 'a'))           // read ambient light value (optional)
                    {
                        char str[4];
                        file.get(str, sizeof(str));
                        if (strcmp(str, "amb") == 0)                        // check for key word: amb
                        {
                            file >> AmbientLight;
                            if (AmbientLight < 0)
                            {
                                if (DicomImageClass::checkDebugLevel(DicomImageClass::DL_Warnings))
                                {
                                    ofConsole.lockCerr() << "WARNING: invalid value for ambient light in DISPLAY file ... ignoring !" << endl;
                                    ofConsole.unlockCerr();
                                }
                                AmbientLight = 0;
                            }
                        } else {
                            if (DicomImageClass::checkDebugLevel(DicomImageClass::DL_Errors))
                            {
                                ofConsole.lockCerr() << "ERROR: invalid DISPLAY file ... ignoring !" << endl;
                                ofConsole.unlockCerr();
                            }
                            return 0;                                       // abort
                        }
                    }
                    else if ((Illumination == 0.0) && (c == 'l'))           // read ambient light value (optional)
                    {
                        char str[4];
                        file.get(str, sizeof(str));
                        if (strcmp(str, "lum") == 0)                        // check for key word: lum
                        {
                            file >> Illumination;
                            if (Illumination < 0)
                            {
                                if (DicomImageClass::checkDebugLevel(DicomImageClass::DL_Warnings))
                                {
                                    ofConsole.lockCerr() << "WARNING: invalid value for illumination in DISPLAY file ... ignoring !" << endl;
                                    ofConsole.unlockCerr();
                                }
                                Illumination = 0;
                            }
                        } else {
                            if (DicomImageClass::checkDebugLevel(DicomImageClass::DL_Errors))
                            {
                                ofConsole.lockCerr() << "ERROR: invalid DISPLAY file ... ignoring !" << endl;
                                ofConsole.unlockCerr();
                            }
                            return 0;                                       // abort
                        }
                    }
                    else if ((Order == 0) && (c == 'o'))                    // read polynomial order (optional)
                    {
                        char str[4];
                        file.get(str, sizeof(str));
                        if (strcmp(str, "ord") == 0)                        // check for key word: ord
                        {
                            file >> Order;
                            if (Order < 0)
                            {
                                if (DicomImageClass::checkDebugLevel(DicomImageClass::DL_Warnings))
                                {
                                    ofConsole.lockCerr() << "WARNING: invalid value for polynomial order in DISPLAY file ... ignoring !" << endl;
                                    ofConsole.unlockCerr();
                                }
                                Order = 0;
                            }
                        } else {
                            if (DicomImageClass::checkDebugLevel(DicomImageClass::DL_Errors))
                            {
                                ofConsole.lockCerr() << "ERROR: invalid DISPLAY file ... ignoring !" << endl;
                                ofConsole.unlockCerr();
                            }
                            return 0;                                       // abort
                        }
                    } else {
                        if (ValueCount <= (unsigned long)MaxDDLValue)
                        {
                            file >> DDLValue[ValueCount];                   // read DDL value
                            file >> LODValue[ValueCount];                   // read luminance/OD value
                            if (file.fail())
                            {
                                if (DicomImageClass::checkDebugLevel(DicomImageClass::DL_Warnings))
                                {
                                    ofConsole.lockCerr() << "WARNING: missing luminance/OD value in DISPLAY file ... "
                                                         << "ignoring last entry !" << endl;
                                    ofConsole.unlockCerr();
                                }
                            }
                            else if (DDLValue[ValueCount] > MaxDDLValue)
                            {
                                if (DicomImageClass::checkDebugLevel(DicomImageClass::DL_Warnings))
                                {
                                    ofConsole.lockCerr() << "WARNING: DDL value (" << DDLValue[ValueCount]
                                                         << ") exceeds maximum value ("
                                                         << MaxDDLValue << ") in DISPLAY file ..." << endl
                                                         << "         ... ignoring value !" << endl;
                                    ofConsole.unlockCerr();
                                }
                            } else
                                ValueCount++;
                        } else {
                            if (DicomImageClass::checkDebugLevel(DicomImageClass::DL_Warnings))
                            {
                                ofConsole.lockCerr() << "WARNING: too many values in DISPLAY file ... "
                                                     << "ignoring last line(s) !" << endl;
                                ofConsole.unlockCerr();
                            }
                            return 2;
                        }
                    }
                }
            }
            if ((MaxDDLValue > 0) && (ValueCount > 0))
                return ((DDLValue != NULL) && (LODValue != NULL));
            else {
                if (DicomImageClass::checkDebugLevel(DicomImageClass::DL_Warnings))
                {
                    ofConsole.lockCerr() << "WARNING: invalid DISPLAY file ... ignoring !" << endl;
                    ofConsole.unlockCerr();
                }
            }
        } else {
            if (DicomImageClass::checkDebugLevel(DicomImageClass::DL_Warnings))
            {
                ofConsole.lockCerr() << "WARNING: can't open DISPLAY file ... ignoring !" << endl;
                ofConsole.unlockCerr();
            }
        }
    }
    return 0;
}


int DiDisplayFunction::createSortedTable(const Uint16 *ddl_tab,
                                         const double *val_tab)
{
    int status = 0;
    Uint16 *old_ddl = DDLValue;
    double *old_val = LODValue;
    if ((ValueCount > 0) && (ddl_tab != NULL) && (val_tab != NULL))
    {
        const unsigned long count = (unsigned long)MaxDDLValue + 1;
        DDLValue = new Uint16[ValueCount];
        LODValue = new double[ValueCount];
        Sint32 *sort_tab = new Sint32[count];                                       // auxiliary array (temporary)
        if ((DDLValue != NULL) && (LODValue != NULL) && (sort_tab != NULL))
        {
            OFBitmanipTemplate<Sint32>::setMem(sort_tab, -1, count);                // initialize array
            register unsigned long i;
            for (i = 0; i < ValueCount; i++)
            {
                if (ddl_tab[i] <= MaxDDLValue)                                      // calculate sort table
                    sort_tab[ddl_tab[i]] = i;
            }
            ValueCount = 0;
            for (i = 0; i <= MaxDDLValue; i++)                                      // sort ascending
            {
                if (sort_tab[i] >= 0)
                {
                    DDLValue[ValueCount] = ddl_tab[sort_tab[i]];
                    LODValue[ValueCount] = (val_tab[sort_tab[i]] > 0) ? val_tab[sort_tab[i]] : 0;
                    ValueCount++;                                                   // re-count to ignore values exceeding max
                }
            }
            i = 1;
            if ((DeviceType == EDT_Printer) || (DeviceType == EDT_Scanner))
            {
                /* hardcopy device: check for monotonous descending OD values */
                while ((i < ValueCount) && (LODValue[i - 1] >= LODValue[i]))
                    i++;
                if (i < ValueCount)
                {
                    if (DicomImageClass::checkDebugLevel(DicomImageClass::DL_Warnings))
                    {
                        ofConsole.lockCerr() << "WARNING: OD values (ordered by DDLs) don't descend monotonously !" << endl;
                        ofConsole.unlockCerr();
                    }
                }
            } else {
                /* softcopy device: check for monotonous ascending luminance values */
                while ((i < ValueCount) && (LODValue[i - 1] <= LODValue[i]))
                    i++;
                if (i < ValueCount)
                {
                    if (DicomImageClass::checkDebugLevel(DicomImageClass::DL_Warnings))
                    {
                        ofConsole.lockCerr() << "WARNING: luminance values (ordered by DDLs) don't ascend monotonously !" << endl;
                        ofConsole.unlockCerr();
                    }
                }
            }
            status = (ValueCount > 0);
        }
        delete[] sort_tab;
    }
    delete[] old_ddl;
    delete[] old_val;
    return status;
}


int DiDisplayFunction::interpolateValues()
{
    if (ValueCount <= (unsigned long)MaxDDLValue)                         // interpolation necessary ?
    {
        int status = 0;
        if (Order > 0)
        {
            /* use polynomial curve fitting */
            double *coeff = new double[Order + 1];
            /* compute coefficients */
            if ((coeff != NULL) &&
                DiCurveFitting<Uint16, double>::calculateCoefficients(DDLValue, LODValue, (unsigned int)ValueCount, Order, coeff))
            {
                /* delete old data arrays */
                delete[] DDLValue;
                delete[] LODValue;
                /* create new data arrays */
                ValueCount = (unsigned long)MaxDDLValue + 1;
                DDLValue = new Uint16[ValueCount];
                LODValue = new double[ValueCount];
                if ((DDLValue != NULL) && (LODValue != NULL))
                {
                    /* set x values linearly */
                    register unsigned int i;
                    for (i = 0; i <= MaxDDLValue; i++)
                        DDLValue[i] = (Uint16)i;
                    /* compute new y values */
                    status = DiCurveFitting<Uint16, double>::calculateValues(0, MaxDDLValue, LODValue, (unsigned int)ValueCount, Order, coeff);
                }
            }
            delete[] coeff;
        } else {
            /* use cubic spline interpolation */
            double *spline = new double[ValueCount];
            if ((spline != NULL) &&
                DiCubicSpline<Uint16, double>::Function(DDLValue, LODValue, (unsigned int)ValueCount, spline))
            {
                /* save old values */
                const unsigned long count = ValueCount;
                Uint16 *old_ddl = DDLValue;
                double *old_val = LODValue;
                /* create new data arrays */
                ValueCount = (unsigned long)MaxDDLValue + 1;
                DDLValue = new Uint16[ValueCount];
                LODValue = new double[ValueCount];
                if ((DDLValue != NULL) && (LODValue != NULL))
                {
                    /* set x values linearly */
                    register unsigned int i;
                    for (i = 0; i <= MaxDDLValue; i++)
                        DDLValue[i] = (Uint16)i;
                    /* compute new y values */
                    status = DiCubicSpline<Uint16, double>::Interpolation(old_ddl, old_val, spline, (unsigned int)count,
                                                                          DDLValue, LODValue, (unsigned int)ValueCount);
                }
                /* delete old data arrays */
                delete[] old_ddl;
                delete[] old_val;
            }
            delete[] spline;
        }
        return status;
    }
    return 2;
}


int DiDisplayFunction::calculateMinMax()
{
    if ((LODValue != NULL) && (ValueCount > 0))
    {
        MinValue = LODValue[0];
        MaxValue = LODValue[0];
        register unsigned long i;
        for (i = 1; i < ValueCount; i++)
        {
            if (LODValue[i] < MinValue)
                MinValue = LODValue[i];
            if (LODValue[i] > MaxValue)
                MaxValue = LODValue[i];
        }
        return 1;
    }
    return 0;
}


double *DiDisplayFunction::convertODtoLumTable(const double *od_tab,
                                               const unsigned long count,
                                               const OFBool useAmb)
{
    double *lum_tab = NULL;
    if ((od_tab != NULL) && (count > 0))
    {
        /* create a table for the luminance values */
        lum_tab = new double[count];
        if (lum_tab != NULL)
        {
            /* compute luminance values from optical density */
            register unsigned int i;
            if (useAmb)
            {
                for (i = 0; i < count; i++)
                    lum_tab[i] = AmbientLight + Illumination * pow(10.0, -od_tab[i]);
            } else {
                /* ambient light is added later */
                for (i = 0; i < count; i++)
                    lum_tab[i] = Illumination * pow(10.0, -od_tab[i]);
            }
        }
    }
    return lum_tab;
}


double DiDisplayFunction::convertODtoLum(const double value,
                                         const OFBool useAmb) const
{
    return (useAmb) ? convertODtoLum(value, AmbientLight, Illumination) :
                      convertODtoLum(value, 0, Illumination);
}


double DiDisplayFunction::convertODtoLum(const double value,
                                         const double ambient,
                                         const double illum)
{
    /* formula from DICOM PS3.14: L = La + L0 * 10^-D */
    return (value >= 0) && (ambient >= 0) && (illum >= 0) ? ambient + illum * pow(10.0, -value) : -1 /*invalid*/;
}


/*
 *
 * CVS/RCS Log:
 * Log: didispfn.cxx,v 
 * Revision 1.3  2004/05/28 17:59:56  peter_vanroose
 * typo corrected
 *
 * Revision 1.2  2004/01/20 14:38:26  amithaperera
 * Attempt to compensate for VC7.1 broken pow()
 *
 * Revision 1.1  2004/01/14 04:01:11  amithaperera
 * Add better DICOM support by wrapping DCMTK, and add a stripped down
 * version of DCMTK to v3p. Add more DICOM test cases.
 *
 * Revision 1.35  2002/11/27 14:08:11  meichel
 * Adapted module dcmimgle to use of new header file ofstdinc.h
 *
 * Revision 1.34  2002/07/19 13:10:15  joergr
 * Fixed bug which occurred for very large number of DDLs only (65536).
 *
 * Revision 1.33  2002/07/18 12:33:55  joergr
 * Added support for hardcopy and softcopy input devices (camera and scanner).
 * Added polynomial curve fitting algorithm as an alternate interpolation
 * method.
 *
 * Revision 1.32  2002/07/03 13:50:59  joergr
 * Fixed inconsistencies regarding the handling of ambient light.
 *
 * Revision 1.31  2002/07/02 16:52:40  joergr
 * Minor fixes to keep MSVC6 quiet.
 *
 * Revision 1.30  2002/07/02 16:24:37  joergr
 * Added support for hardcopy devices to the calibrated output routines.
 *
 * Revision 1.29  2002/04/16 13:53:31  joergr
 * Added configurable support for C++ ANSI standard includes (e.g. streams).
 * Thanks to Andreas Barth <Andreas.Barth@bruker-biospin.de> for his
 * contribution.
 *
 * Revision 1.28  2001/09/28 13:12:20  joergr
 * Added check whether ios::nocreate exists.
 *
 * Revision 1.27  2001/06/01 15:49:54  meichel
 * Updated copyright header
 *
 * Revision 1.26  2000/05/03 09:47:23  joergr
 * Removed most informational and some warning messages from release built
 * (#ifndef DEBUG).
 *
 * Revision 1.25  2000/04/28 12:33:42  joergr
 * DebugLevel - global for the module - now derived from OFGlobal (MF-safe).
 *
 * Revision 1.24  2000/04/27 13:10:25  joergr
 * Dcmimgle library code now consistently uses ofConsole for error output.
 *
 * Revision 1.23  2000/03/08 16:24:27  meichel
 * Updated copyright header.
 *
 * Revision 1.22  2000/03/07 16:15:46  joergr
 * Added explicit type casts to make Sun CC 2.0.1 happy.
 *
 * Revision 1.21  2000/03/06 18:20:35  joergr
 * Moved get-method to base class, renamed method and made method virtual to
 * avoid hiding of methods (reported by Sun CC 4.2).
 *
 * Revision 1.20  2000/03/03 14:09:17  meichel
 * Implemented library support for redirecting error messages into memory
 *   instead of printing them to stdout/stderr for GUI applications.
 *
 * Revision 1.19  2000/02/02 11:04:52  joergr
 * Removed space characters before preprocessor directives.
 *
 * Revision 1.18  1999/10/18 17:24:00  joergr
 * Added explicit type cast to avoid compiler warnings reported by MSVC.
 *
 * Revision 1.17  1999/10/18 15:06:24  joergr
 * Enhanced command line tool dcmdspfn (added new options).
 *
 * Revision 1.16  1999/10/18 10:14:27  joergr
 * Moved min/max value determination to display function base class. Now the
 * actual min/max values are also used for GSDFunction (instead of first and
 * last luminance value).
 *
 * Revision 1.15  1999/09/10 08:54:49  joergr
 * Added support for CIELAB display function. Restructured class hierarchy
 * for display functions.
 *
 * Revision 1.14  1999/07/23 13:34:08  joergr
 * Modified error reporting while reading calibration file.
 *
 * Revision 1.13  1999/05/03 11:05:29  joergr
 * Minor code purifications to keep Sun CC 2.0.1 quiet.
 *
 * Revision 1.12  1999/04/29 13:49:37  joergr
 * Renamed class CubicSpline to DiCubicSpline.
 *
 * Revision 1.11  1999/04/28 15:01:44  joergr
 * Introduced new scheme for the debug level variable: now each level can be
 * set separately (there is no "include" relationship).
 *
 * Revision 1.10  1999/03/24 17:22:38  joergr
 * Added support for Barten transformation from 2 to 7 bits input (now: 2-16).
 *
 * Revision 1.9  1999/03/22 08:54:10  joergr
 * Added/Changed comments.
 *
 * Revision 1.7  1999/03/04 09:43:28  joergr
 * Barten LUT is now be re-created when ambient light value has changed.
 *
 * Revision 1.6  1999/03/03 12:06:24  joergr
 * Added support to specify ambient light value (re: Barten transformation).
 *
 * Revision 1.5  1999/02/23 16:56:06  joergr
 * Added tool to export display curves to a text file.
 *
 * Revision 1.4  1999/02/11 16:50:34  joergr
 * Removed unused parameter / member variable.
 * Renamed file to indicate the use of templates. Moved global functions for
 * cubic spline interpolation to static methods of a separate template class.
 * Added mode ios::nocreate when opening file streams for reading to avoid
 * implicit creation of non-existing files.
 *
 * Revision 1.3  1999/02/09 14:22:31  meichel
 * Removed explicit template parameters from template function calls,
 *   required for Sun CC 4.2
 *
 * Revision 1.2  1999/02/08 13:09:06  joergr
 * Added (debug) warning message when using invalid DISPLAY file names.
 *
 * Revision 1.1  1999/02/03 17:48:37  joergr
 * Added support for calibration according to Barten transformation (incl.
 * a DISPLAY file describing the monitor characteristic).
 *
 *
 */
