//
// Copyright (C) 1991 Texas Instruments Incorporated.
//
// Permission is granted to any individual or institution to use, copy, modify,
// and distribute this software, provided that this complete copyright and
// permission notice is maintained, intact, in all copies and supporting
// documentation.
//
// Texas Instruments Incorporated provides this software "as is" without
// express or implied warranty.
//

// Created: BMK 07/14/89  Initial design and implementation
// Updated: LGO 09/23/89  Conform to COOL coding style
// Updated: AFM 12/31/89  OS/2 port
// Updated: DLS 03/22/91  New lite version
// Updated: VDN 10/14/93  ANSI C does not have user/system time.
//
// The Timer class provides timing code  for performance evaluation.  This code
// was originally written by Joe Rahmeh at UT Austin.
//
//  User time:
//    time cpu spends in user mode on behalf of the program.
//  System time:
//    time cpu spends in system mode on behalf of the program.
//  Real time:
//    what you get from a stop watch timer.
//

#include "vbl_timer.h"

#include <vcl_sys/time.h>
#include <vcl_ctime.h>

struct vbl_timer_data
{
#ifndef WIN32
  tms usage0;                    // usage mark.
  struct timeval real0;          // wall clock mark.
#else
 clock_t usage0;
 struct _timeb real0;
#endif
};

#include <vxl_config.h> // VXL_TWO_ARG_GETTIME

#include <vcl_climits.h>   // for CLK_TCK
#include <vcl_ctime.h>
#include <vcl_iostream.h>
#include <vcl_sys/time.h>


//#define CLK_TCK _sysconf(3) in <limits.h> has error

#ifdef WIN32
#include <direct.h> // for sysconf()
#else
#include <unistd.h>
#endif
#undef CLK_TCK
#define CLK_TCK sysconf(_SC_CLK_TCK)

vbl_timer::vbl_timer()
  : data(new vbl_timer_data)
{
  mark();
}

vbl_timer::~vbl_timer()
{
  delete data;
  data = 0;
}

// -- Sets the reference time to now.

void vbl_timer::mark()
{
#ifndef WIN32
  times(&data->usage0);  // user/system time
#ifndef SYSV
  struct timezone tz;
  gettimeofday(&data->real0, &tz);  // wall clock time
#else
#if VXL_TWO_ARG_GETTIME
  gettimeofday(&data->real0, (void*)0);
#else
  gettimeofday(&data->real0);
#endif
#endif
#else
  // Win32 section
  data->usage0 = clock();
  _ftime(&data->real0);
#endif
}

// -- Returns the number of milliseconds of wall clock time, since last mark().

long vbl_timer::real()
{
 long s;

#ifndef WIN32
 struct timeval  real;    // new real time
#ifndef SYSV
 struct timezone tz;
 gettimeofday(&real, &tz);  // wall clock time
#else
#if VXL_TWO_ARG_GETTIME
  gettimeofday(&real, (void*)0);
#else
  gettimeofday(&real);
#endif
#endif
 s  = real.tv_sec    - data->real0.tv_sec;
 long us = real.tv_usec - data->real0.tv_usec;

 if(us < 0)
   {us += 1000000;
    s--;
   }
 return long(1000.0*s + us / 1000.0 + 0.5);

#else
 // Win32 section
 struct _timeb real;
 _ftime(&real);
 s = real.time - data->real0.time;
 long ms = real.millitm - data->real0.millitm;

 if(ms < 0) {
   ms += 1000;
   s--;
 }
 return 1000*s + ms;
#endif
}

// --

long vbl_timer::user()
{
#ifndef WIN32
  tms usage;
  times(&usage);  // new user/system time
  return (usage.tms_utime - data->usage0.tms_utime) * 1000 / CLK_TCK;
#else
  clock_t usage = clock();
  return (usage - data->usage0) / (CLOCKS_PER_SEC/1000);
#endif
}

// -- Returns the number of milliseconds spent in user-process or
// operating system respectively, since last mark().

long vbl_timer::system()
{
#ifndef WIN32
  tms usage;
  times(&usage);  // new user/system time
  return (usage.tms_stime - data->usage0.tms_stime) * 1000 / CLK_TCK;
#else
  return 0L;
#endif
}

// Returns the number of milliseconds spent in user-process AND
// operating system, since last mark().

long vbl_timer::all()
{
#ifndef WIN32
  tms usage;
  times(&usage);  // new user/system time
  return (usage.tms_utime + usage.tms_stime -
          data->usage0.tms_utime - data->usage0.tms_stime)  * 1000 / CLK_TCK;
#else
  clock_t usage = clock();
  return (usage - data->usage0) / (CLOCKS_PER_SEC/1000);
#endif
}

// -- Display user and real time since the last mark.
void vbl_timer::print(vcl_ostream& s)
{
  s << "Time: user " << user() / 1000.0 << ", real " << real() / 1000.0 << vcl_endl;
}
