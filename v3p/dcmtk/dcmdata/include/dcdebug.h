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
 *  Purpose: Print debug information
 *
 *  Last Update:      Author: amithaperera 
 *  Update Date:      Date: 2004/01/14 04:01:09 
 *  Source File:      Source: /cvsroot/vxl/vxl/v3p/dcmtk/dcmdata/include/dcdebug.h,v 
 *  CVS/RCS Revision: Revision: 1.1 
 *  Status:           State: Exp 
 *
 *  CVS/RCS Log at end of file
 *
 */

#ifndef DCDEBUG_H
#define DCDEBUG_H

#include "osconfig.h"    /* make sure OS specific configuration is included first */
#include "ofstream.h"
#include "ofglobal.h"

extern OFGlobal<int> DcmDebugLevel; /* default 0 */

#ifdef DEBUG

void debug_print(const char* text, ... );

// Set the debug level
#define SetDebugLevel(level) DcmDebugLevel.set(level);

// debug prints a debug message in param if lev <= DcmDebugLevel. param has the
// format of the printf parameters (with round brackets)!
#define debug(lev, param) \
  { \
    if ((lev) <= DcmDebugLevel.get()) \
    { \
      ofConsole.lockCerr() << __FILE__ << ", LINE " << __LINE__ << ":"; \
      debug_print param ; \
      ofConsole.unlockCerr(); \
    } \
  }

// Cdebug does the same as debug but only if a condition cond is OFTrue
#define Cdebug(lev, cond, param) \
  { \
    if ((lev) <= DcmDebugLevel.get() && (cond)) \
    { \
      ofConsole.lockCerr() << __FILE__ << ", LINE " << __LINE__ << ":"; \
      debug_print param ; \
      ofConsole.unlockCerr(); \
    } \
  }

#else  // DEBUG

#define SetDebugLevel(param)
#define debug(lev, param)
#define Cdebug(lev, cond, param)

#endif // DEBUG

#endif // DCDEBUG_H

/*
 * CVS/RCS Log:
 * Log: dcdebug.h,v 
 * Revision 1.1  2004/01/14 04:01:09  amithaperera
 * Add better DICOM support by wrapping DCMTK, and add a stripped down
 * version of DCMTK to v3p. Add more DICOM test cases.
 *
 * Revision 1.10  2002/04/16 13:41:43  joergr
 * Added configurable support for C++ ANSI standard includes (e.g. streams).
 * Thanks to Andreas Barth <Andreas.Barth@bruker-biospin.de> for his
 * contribution.
 *
 * Revision 1.9  2001/06/01 15:48:35  meichel
 * Updated copyright header
 *
 * Revision 1.8  2000/04/14 15:45:30  meichel
 * Dcmdata debug facility now uses ofConsole for output.
 *
 * Revision 1.7  2000/03/08 16:26:12  meichel
 * Updated copyright header.
 *
 * Revision 1.6  2000/03/03 14:05:22  meichel
 * Implemented library support for redirecting error messages into memory
 *   instead of printing them to stdout/stderr for GUI applications.
 *
 * Revision 1.5  1999/03/31 09:24:33  meichel
 * Updated copyright header in module dcmdata
 *
 *
 */
