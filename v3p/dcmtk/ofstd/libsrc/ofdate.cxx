/*
 *
 *  Copyright (C) 2002, OFFIS
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
 *  Module:  ofstd
 *
 *  Author:  Joerg Riesmeier
 *
 *  Purpose: Class for date functions (Source)
 *
 *  Last Update:      Author: peter_vanroose 
 *  Update Date:      Date: 2004/08/06 14:11:49 
 *  Source File:      Source: /cvsroot/vxl/vxl/v3p/dcmtk/ofstd/libsrc/ofdate.cxx,v 
 *  CVS/RCS Revision: Revision: 1.2 
 *  Status:           State: Exp 
 *
 *  CVS/RCS Log at end of file
 *
 */


#include "osconfig.h"
#include "ofdate.h"

#define INCLUDE_CSTDIO
#define INCLUDE_CTIME
#include "ofstdinc.h"

/*------------------*
 *  implementation  *
 *------------------*/

OFDate::OFDate()
  : Year(0),
    Month(0),
    Day(0)
{
}


OFDate::OFDate(const OFDate &dateVal)
  : Year(dateVal.Year),
    Month(dateVal.Month),
    Day(dateVal.Day)
{
}


OFDate::OFDate(const unsigned int year,
               const unsigned int month,
               const unsigned int day)
  : Year(year),
    Month(month),
    Day(day)
{
}


OFDate::~OFDate()
{
}


OFDate &OFDate::operator=(const OFDate &dateVal)
{
    Year = dateVal.Year;
    Month = dateVal.Month;
    Day = dateVal.Day;
    return *this;
}


OFBool OFDate::operator==(const OFDate &dateVal)
{
    return (Year == dateVal.Year) && (Month == dateVal.Month) && (Day == dateVal.Day);
}


OFBool OFDate::operator!=(const OFDate &dateVal)
{
    return (Year != dateVal.Year) || (Month != dateVal.Month) || (Day != dateVal.Day);
}


OFBool OFDate::operator<(const OFDate &dateVal)
{
    return (Year < dateVal.Year) || ((Year == dateVal.Year) && ((Month < dateVal.Month) || ((Month == dateVal.Month) && (Day < dateVal.Day))));
}


OFBool OFDate::operator<=(const OFDate &dateVal)
{
    return (Year < dateVal.Year) || ((Year == dateVal.Year) && ((Month < dateVal.Month) || ((Month == dateVal.Month) && (Day <= dateVal.Day))));
}


OFBool OFDate::operator>=(const OFDate &dateVal)
{
    return (Year > dateVal.Year) || ((Year == dateVal.Year) && ((Month > dateVal.Month) || ((Month == dateVal.Month) && (Day >= dateVal.Day))));
}


OFBool OFDate::operator>(const OFDate &dateVal)
{
    return (Year > dateVal.Year) || ((Year == dateVal.Year) && ((Month > dateVal.Month) || ((Month == dateVal.Month) && (Day > dateVal.Day))));
}


void OFDate::clear()
{
    Year = 0;
    Month = 0;
    Day = 0;
}


OFBool OFDate::isValid() const
{
    /* check current date settings */
    return isDateValid(Year, Month, Day);
}


OFBool OFDate::isDateValid(const unsigned int /*year*/,
                           const unsigned int month,
                           const unsigned int day)
{
    /* this very simple validity check might be enhanced in the future */
    return (month >= 1) && (month <= 12) && (day >= 1) && (day <= 31);
}


OFBool OFDate::setDate(const unsigned int year,
                       const unsigned int month,
                       const unsigned int day)
{
    OFBool status = OFFalse;
    /* only change if the new date is valid */
    if (isDateValid(year, month, day))
    {
        Year = year;
        Month = month;
        Day = day;
        /* report that a new date has been set */
        status = OFTrue;
    }
    return status;
}


OFBool OFDate::setYear(const unsigned int year)
{
    OFBool status = OFFalse;
    /* only change if the new year is valid */
    if (isDateValid(year, Month, Day))
    {
        Year = year;
        /* report that a new year has been set */
        status = OFTrue;
    }
    return status;
}


OFBool OFDate::setMonth(const unsigned int month)
{
    OFBool status = OFFalse;
    /* only change if the new month is valid */
    if (isDateValid(Year, month, Day))
    {
        Month = month;
        /* report that a new month has been set */
        status = OFTrue;
    }
    return status;
}


OFBool OFDate::setDay(const unsigned int day)
{
    OFBool status = OFFalse;
    /* only change if the new day is valid */
    if (isDateValid(Year, Month, day))
    {
        Day = day;
        /* report that a new day has been set */
        status = OFTrue;
    }
    return status;
}


OFBool OFDate::setCurrentDate()
{
    /* get the current system date and call the "real" function */
    return setCurrentDate(time(NULL));
}


OFBool OFDate::setCurrentDate(const time_t &tt)
{
    OFBool status = OFFalse;
#if defined(_REENTRANT) && !defined(_WIN32) && !defined(__CYGWIN__) && !defined(__hpux)
    // use localtime_r instead of localtime
    struct tm ltBuf;
    struct tm *lt = &ltBuf;
    localtime_r(&tt, lt);
#else
    struct tm *lt = localtime(&tt);
#endif
    if (lt != NULL)
    {
        /* store retrieved date */
        Year = 1900 + lt->tm_year;
        Month = lt->tm_mon + 1;
        Day = lt->tm_mday;
        /* report that current system date has been set */
        status = OFTrue;
    }
    return status;
}


unsigned int OFDate::getYear() const
{
    return Year;
}


unsigned int OFDate::getMonth() const
{
    return Month;
}


unsigned int OFDate::getDay() const
{
    return Day;
}


OFBool OFDate::getISOFormattedDate(OFString &formattedDate,
                                   const OFBool showDelimiter) const
{
    OFBool status = OFFalse;
    /* check for valid date first */
    if (isValid())
    {
        char buf[32];
        /* format: YYYY-MM-DD */
        if (showDelimiter)
            sprintf(buf, "%04u-%02u-%02u", Year, Month, Day);
        /* format: YYYYMMDD */
        else
            sprintf(buf, "%04u%02u%02u", Year, Month, Day);
        formattedDate = buf;
        status = OFTrue;
    }
    return status;
}


OFDate OFDate::getCurrentDate()
{
    /* create a date object with the current system date set */
    OFDate dateVal;
    /* this call might fail! */
    dateVal.setCurrentDate();
    /* return by-value */
    return dateVal;
}


ostream& operator<<(ostream& stream, const OFDate &dateVal)
{
    OFString string;
    /* print the given date in ISO format to the stream */
    if (dateVal.getISOFormattedDate(string))
        stream << string;
    return stream;
}


/*
 *
 * CVS/RCS Log:
 * Log: ofdate.cxx,v 
 * Revision 1.2  2004/08/06 14:11:49  peter_vanroose
 * fix for platform HPUX
 *
 * Revision 1.1  2004/01/14 04:01:11  amithaperera
 * Add better DICOM support by wrapping DCMTK, and add a stripped down
 * version of DCMTK to v3p. Add more DICOM test cases.
 *
 * Revision 1.4  2002/11/27 11:23:10  meichel
 * Adapted module ofstd to use of new header file ofstdinc.h
 *
 * Revision 1.3  2002/05/24 09:44:26  joergr
 * Renamed some parameters/variables to avoid ambiguities.
 *
 * Revision 1.2  2002/04/15 09:40:47  joergr
 * Removed "include <sys/types.h>" from implementation file.
 *
 * Revision 1.1  2002/04/11 12:14:33  joergr
 * Introduced new standard classes providing date and time functions.
 *
 *
 */
