#ifndef v3p_netlib_mangle_h
#define v3p_netlib_mangle_h

/*

This header file mangles all symbols exported from the v3p_netlib
library.  It is included in all files while building the library.

The following command was used to obtain the symbol list:

nm libv3p_netlib.a |grep " [TR] "

*/

/* Functions */
#define caxpy_ v3p_netlib_caxpy_
#define cdotc_ v3p_netlib_cdotc_
#define cscal_ v3p_netlib_cscal_
#define csrot_ v3p_netlib_csrot_
#define csvdc_ v3p_netlib_csvdc_
#define cswap_ v3p_netlib_cswap_
#define daxpy_ v3p_netlib_daxpy_
#define dcabs1_ v3p_netlib_dcabs1_
#define ddot_ v3p_netlib_ddot_
#define dnrm2_ v3p_netlib_dnrm2_
#define drot_ v3p_netlib_drot_
#define drotg_ v3p_netlib_drotg_
#define dscal_ v3p_netlib_dscal_
#define dsvdc_ v3p_netlib_dsvdc_
#define dswap_ v3p_netlib_dswap_
#define dznrm2_ v3p_netlib_dznrm2_
#define saxpy_ v3p_netlib_saxpy_
#define scnrm2_ v3p_netlib_scnrm2_
#define sdot_ v3p_netlib_sdot_
#define snrm2_ v3p_netlib_snrm2_
#define srot_ v3p_netlib_srot_
#define srotg_ v3p_netlib_srotg_
#define sscal_ v3p_netlib_sscal_
#define ssvdc_ v3p_netlib_ssvdc_
#define sswap_ v3p_netlib_sswap_
#define zaxpy_ v3p_netlib_zaxpy_
#define zdotc_ v3p_netlib_zdotc_
#define zdrot_ v3p_netlib_zdrot_
#define zscal_ v3p_netlib_zscal_
#define zsvdc_ v3p_netlib_zsvdc_
#define zswap_ v3p_netlib_zswap_

#endif
