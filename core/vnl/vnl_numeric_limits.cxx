// This is core/vnl/vnl_numeric_limits.cxx
#ifdef VCL_NEEDS_PRAGMA_INTERFACE
#pragma implementation
#endif
//
// numeric_limits
// Author: Andrew W. Fitzgibbon, Oxford RRG
// Created: 28 Aug 96
//
//-----------------------------------------------------------------------------

#include "vnl_numeric_limits.h"
#include <vxl_config.h> // for VXL_BIG_ENDIAN

// ----------------------------------------------------------------------
// Constants for int

const bool vnl_numeric_limits<int>::is_specialized VCL_STATIC_CONST_INIT_INT_DEFN(true);
const int vnl_numeric_limits<int>::digits   VCL_STATIC_CONST_INIT_INT_DEFN(31);
const int vnl_numeric_limits<int>::digits10 VCL_STATIC_CONST_INIT_INT_DEFN(9);
const bool vnl_numeric_limits<int>::is_signed  VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<int>::is_integer VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<int>::is_exact   VCL_STATIC_CONST_INIT_INT_DEFN(true);
const int vnl_numeric_limits<int>::radix VCL_STATIC_CONST_INIT_INT_DEFN(2);
const int vnl_numeric_limits<int>::min_exponent   VCL_STATIC_CONST_INIT_INT_DEFN(-31);
const int vnl_numeric_limits<int>::min_exponent10 VCL_STATIC_CONST_INIT_INT_DEFN(-9);
const int vnl_numeric_limits<int>::max_exponent   VCL_STATIC_CONST_INIT_INT_DEFN(31);
const int vnl_numeric_limits<int>::max_exponent10 VCL_STATIC_CONST_INIT_INT_DEFN(9);
const bool vnl_numeric_limits<int>::has_infinity      VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<int>::has_quiet_NaN     VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<int>::has_signaling_NaN VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<int>::has_denorm        VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<int>::is_iec559  VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<int>::is_bounded VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<int>::is_modulo  VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<int>::traps      VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<int>::tinyness_before VCL_STATIC_CONST_INIT_INT_DEFN(false);
const vnl_float_round_style vnl_numeric_limits<int>::round_style VCL_STATIC_CONST_INIT_INT_DEFN(vnl_round_toward_zero);


// ----------------------------------------------------------------------
// Constants for long

const bool vnl_numeric_limits<long>::is_specialized VCL_STATIC_CONST_INIT_INT_DEFN(true);
const int vnl_numeric_limits<long>::digits   VCL_STATIC_CONST_INIT_INT_DEFN(31);
const int vnl_numeric_limits<long>::digits10 VCL_STATIC_CONST_INIT_INT_DEFN(9);
const bool vnl_numeric_limits<long>::is_signed  VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<long>::is_integer VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<long>::is_exact   VCL_STATIC_CONST_INIT_INT_DEFN(true);
const int vnl_numeric_limits<long>::radix VCL_STATIC_CONST_INIT_INT_DEFN(2);
const int vnl_numeric_limits<long>::min_exponent   VCL_STATIC_CONST_INIT_INT_DEFN(-31);
const int vnl_numeric_limits<long>::min_exponent10 VCL_STATIC_CONST_INIT_INT_DEFN(-9);
const int vnl_numeric_limits<long>::max_exponent   VCL_STATIC_CONST_INIT_INT_DEFN(31);
const int vnl_numeric_limits<long>::max_exponent10 VCL_STATIC_CONST_INIT_INT_DEFN(9);
const bool vnl_numeric_limits<long>::has_infinity      VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<long>::has_quiet_NaN     VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<long>::has_signaling_NaN VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<long>::has_denorm        VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<long>::is_iec559  VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<long>::is_bounded VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<long>::is_modulo  VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<long>::traps      VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<long>::tinyness_before VCL_STATIC_CONST_INIT_INT_DEFN(false);
const vnl_float_round_style vnl_numeric_limits<long>::round_style VCL_STATIC_CONST_INIT_INT_DEFN(vnl_round_toward_zero);


// ----------------------------------------------------------------------
// Constants for unsigned long

const bool vnl_numeric_limits<unsigned long>::is_specialized VCL_STATIC_CONST_INIT_INT_DEFN(true);
const int vnl_numeric_limits<unsigned long>::digits   VCL_STATIC_CONST_INIT_INT_DEFN(sizeof(unsigned long) * 8 );
const int vnl_numeric_limits<unsigned long>::digits10 VCL_STATIC_CONST_INIT_INT_DEFN( (digits * 301) / 1000 );
const bool vnl_numeric_limits<unsigned long>::is_signed  VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<unsigned long>::is_integer VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<unsigned long>::is_exact   VCL_STATIC_CONST_INIT_INT_DEFN(true);
const int vnl_numeric_limits<unsigned long>::radix VCL_STATIC_CONST_INIT_INT_DEFN(2);
const int vnl_numeric_limits<unsigned long>::min_exponent   VCL_STATIC_CONST_INIT_INT_DEFN(-31);
const int vnl_numeric_limits<unsigned long>::min_exponent10 VCL_STATIC_CONST_INIT_INT_DEFN(-9);
const int vnl_numeric_limits<unsigned long>::max_exponent   VCL_STATIC_CONST_INIT_INT_DEFN(31);
const int vnl_numeric_limits<unsigned long>::max_exponent10 VCL_STATIC_CONST_INIT_INT_DEFN(9);
const bool vnl_numeric_limits<unsigned long>::has_infinity      VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<unsigned long>::has_quiet_NaN     VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<unsigned long>::has_signaling_NaN VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<unsigned long>::has_denorm        VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<unsigned long>::is_iec559  VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<unsigned long>::is_bounded VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<unsigned long>::is_modulo  VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<unsigned long>::traps      VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<unsigned long>::tinyness_before VCL_STATIC_CONST_INIT_INT_DEFN(false);
const vnl_float_round_style vnl_numeric_limits<unsigned long>::round_style VCL_STATIC_CONST_INIT_INT_DEFN(vnl_round_toward_zero);


// ----------------------------------------------------------------------
// Constants for unsigned short

const bool vnl_numeric_limits<unsigned short>::is_specialized VCL_STATIC_CONST_INIT_INT_DEFN(true);
const int vnl_numeric_limits<unsigned short>::digits   VCL_STATIC_CONST_INIT_INT_DEFN(sizeof(unsigned short) * 8 );
const int vnl_numeric_limits<unsigned short>::digits10 VCL_STATIC_CONST_INIT_INT_DEFN( (digits * 301) / 1000 );
const bool vnl_numeric_limits<unsigned short>::is_signed  VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<unsigned short>::is_integer VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<unsigned short>::is_exact   VCL_STATIC_CONST_INIT_INT_DEFN(true);
const int vnl_numeric_limits<unsigned short>::radix VCL_STATIC_CONST_INIT_INT_DEFN(2);
const int vnl_numeric_limits<unsigned short>::min_exponent   VCL_STATIC_CONST_INIT_INT_DEFN(-31);
const int vnl_numeric_limits<unsigned short>::min_exponent10 VCL_STATIC_CONST_INIT_INT_DEFN(-9);
const int vnl_numeric_limits<unsigned short>::max_exponent   VCL_STATIC_CONST_INIT_INT_DEFN(31);
const int vnl_numeric_limits<unsigned short>::max_exponent10 VCL_STATIC_CONST_INIT_INT_DEFN(9);
const bool vnl_numeric_limits<unsigned short>::has_infinity      VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<unsigned short>::has_quiet_NaN     VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<unsigned short>::has_signaling_NaN VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<unsigned short>::has_denorm        VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<unsigned short>::is_iec559  VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<unsigned short>::is_bounded VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<unsigned short>::is_modulo  VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<unsigned short>::traps      VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<unsigned short>::tinyness_before VCL_STATIC_CONST_INIT_INT_DEFN(false);
const vnl_float_round_style vnl_numeric_limits<unsigned short>::round_style VCL_STATIC_CONST_INIT_INT_DEFN(vnl_round_toward_zero);


// ----------------------------------------------------------------------
// Constants for short

const bool vnl_numeric_limits<short>::is_specialized VCL_STATIC_CONST_INIT_INT_DEFN(true);
const int vnl_numeric_limits<short>::digits   VCL_STATIC_CONST_INIT_INT_DEFN(15);
const int vnl_numeric_limits<short>::digits10 VCL_STATIC_CONST_INIT_INT_DEFN(5);
const bool vnl_numeric_limits<short>::is_signed  VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<short>::is_integer VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<short>::is_exact   VCL_STATIC_CONST_INIT_INT_DEFN(true);
const int vnl_numeric_limits<short>::radix VCL_STATIC_CONST_INIT_INT_DEFN(2);
const int vnl_numeric_limits<short>::min_exponent   VCL_STATIC_CONST_INIT_INT_DEFN(-15);
const int vnl_numeric_limits<short>::min_exponent10 VCL_STATIC_CONST_INIT_INT_DEFN(-5);
const int vnl_numeric_limits<short>::max_exponent   VCL_STATIC_CONST_INIT_INT_DEFN(15);
const int vnl_numeric_limits<short>::max_exponent10 VCL_STATIC_CONST_INIT_INT_DEFN(5);
const bool vnl_numeric_limits<short>::has_infinity      VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<short>::has_quiet_NaN     VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<short>::has_signaling_NaN VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<short>::has_denorm        VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<short>::is_iec559  VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<short>::is_bounded VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<short>::is_modulo  VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<short>::traps      VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<short>::tinyness_before VCL_STATIC_CONST_INIT_INT_DEFN(false);
const vnl_float_round_style vnl_numeric_limits<short>::round_style VCL_STATIC_CONST_INIT_INT_DEFN(vnl_round_toward_zero);

// ----------------------------------------------------------------------
// Constants and functions for double

union vnl_numeric_limits_double_nan {
  double nan;
  unsigned char x[8];

  vnl_numeric_limits_double_nan() {
#if VXL_BIG_ENDIAN
    x[0] = 0x7f;
    x[1] = x[2] = x[3] = x[4] = x[5] = x[6] = x[7] = 0xff;
#else
    x[7] = 0x7f;
    x[0] = x[1] = x[2] = x[3] = x[4] = x[5] = x[6] = 0xff;
#endif
  }
};
static vnl_numeric_limits_double_nan dnan;

union vnl_numeric_limits_double_inf {
  double inf;
  unsigned char x[8];

  vnl_numeric_limits_double_inf() {
#if VXL_BIG_ENDIAN
    x[0] = 0x7f; x[1] = 0xf0;
    x[2] = x[3] = x[4] = x[5] = x[6] = x[7] = 0x00;
#else
    x[7] = 0x7f; x[6] = 0xf0;
    x[0] = x[1] = x[2] = x[3] = x[4] = x[5] = 0x00;
#endif
  }
};
static vnl_numeric_limits_double_inf dinf;

double vnl_numeric_limits<double>::infinity()
{
  return dinf.inf;
}

double vnl_numeric_limits<double>::quiet_NaN()
{
  return dnan.nan;
}

double vnl_numeric_limits<double>::signaling_NaN()
{
  return quiet_NaN();
}

const bool vnl_numeric_limits<double>::is_specialized VCL_STATIC_CONST_INIT_INT_DEFN(true);
const int  vnl_numeric_limits<double>::digits   VCL_STATIC_CONST_INIT_INT_DEFN(53);
const int  vnl_numeric_limits<double>::digits10 VCL_STATIC_CONST_INIT_INT_DEFN( 15);
const bool vnl_numeric_limits<double>::is_signed  VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<double>::is_integer VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<double>::is_exact   VCL_STATIC_CONST_INIT_INT_DEFN(false);
const int  vnl_numeric_limits<double>::radix VCL_STATIC_CONST_INIT_INT_DEFN(2);
const int  vnl_numeric_limits<double>::min_exponent   VCL_STATIC_CONST_INIT_INT_DEFN(-1021);
const int  vnl_numeric_limits<double>::min_exponent10 VCL_STATIC_CONST_INIT_INT_DEFN(-307);
const int  vnl_numeric_limits<double>::max_exponent   VCL_STATIC_CONST_INIT_INT_DEFN(1024);
const int  vnl_numeric_limits<double>::max_exponent10 VCL_STATIC_CONST_INIT_INT_DEFN(308);
const bool vnl_numeric_limits<double>::has_infinity      VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<double>::has_quiet_NaN     VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<double>::has_signaling_NaN VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<double>::has_denorm        VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<double>::is_iec559  VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<double>::is_bounded VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<double>::is_modulo  VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<double>::traps      VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<double>::tinyness_before VCL_STATIC_CONST_INIT_INT_DEFN(true);
const vnl_float_round_style vnl_numeric_limits<double>::round_style VCL_STATIC_CONST_INIT_INT_DEFN(vnl_round_to_nearest);


// ----------------------------------------------------------------------
// Constants and functions for float

union vnl_numeric_limits_float_nan {
  float nan;
  unsigned char x[4];

  vnl_numeric_limits_float_nan() {
#if VXL_BIG_ENDIAN
    x[0] = 0x7f; x[1] = x[2] = x[3] = 0xff;
#else
    x[3] = 0x7f; x[0] = x[1] = x[2] = 0xff;
#endif
  }
};
static vnl_numeric_limits_float_nan fnan;

union vnl_numeric_limits_float_inf {
  float inf;
  unsigned char x[4];

  vnl_numeric_limits_float_inf() {
#if VXL_BIG_ENDIAN
    x[0] = 0x7f; x[1] = 0x80; x[2] = x[3] = 0x00;
#else
    x[3] = 0x7f; x[2] = 0x80; x[0] = x[1] = 0x00;
#endif
  }
};
static vnl_numeric_limits_float_inf finf;

float vnl_numeric_limits<float>::infinity()
{
  return finf.inf;
}

float vnl_numeric_limits<float>::quiet_NaN()
{
  return fnan.nan;
}

float vnl_numeric_limits<float>::signaling_NaN()
{
  return quiet_NaN();
}

const bool vnl_numeric_limits<float>::is_specialized VCL_STATIC_CONST_INIT_INT_DEFN(true);
const int  vnl_numeric_limits<float>::digits   VCL_STATIC_CONST_INIT_INT_DEFN(24);
const int  vnl_numeric_limits<float>::digits10 VCL_STATIC_CONST_INIT_INT_DEFN( 6);
const bool vnl_numeric_limits<float>::is_signed  VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<float>::is_integer VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<float>::is_exact   VCL_STATIC_CONST_INIT_INT_DEFN(false);
const int  vnl_numeric_limits<float>::radix VCL_STATIC_CONST_INIT_INT_DEFN(2);
const int  vnl_numeric_limits<float>::min_exponent   VCL_STATIC_CONST_INIT_INT_DEFN(-125);
const int  vnl_numeric_limits<float>::min_exponent10 VCL_STATIC_CONST_INIT_INT_DEFN(-37);
const int  vnl_numeric_limits<float>::max_exponent   VCL_STATIC_CONST_INIT_INT_DEFN(128);
const int  vnl_numeric_limits<float>::max_exponent10 VCL_STATIC_CONST_INIT_INT_DEFN(38);
const bool vnl_numeric_limits<float>::has_infinity      VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<float>::has_quiet_NaN     VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<float>::has_signaling_NaN VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<float>::has_denorm        VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<float>::is_iec559  VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<float>::is_bounded VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<float>::is_modulo  VCL_STATIC_CONST_INIT_INT_DEFN(false);
const bool vnl_numeric_limits<float>::traps      VCL_STATIC_CONST_INIT_INT_DEFN(true);
const bool vnl_numeric_limits<float>::tinyness_before VCL_STATIC_CONST_INIT_INT_DEFN(true);
const vnl_float_round_style vnl_numeric_limits<float>::round_style VCL_STATIC_CONST_INIT_INT_DEFN(vnl_round_to_nearest);
