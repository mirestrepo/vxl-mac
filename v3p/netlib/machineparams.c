#include "f2c.h"

doublereal dpmpar_(i)
integer *i;
{
/*     function dpmpar */

/*     This function provides double precision machine parameters */
/*     when the appropriate set of data statements is activated (by */
/*     removing the c from column 1) and all other data statements are */
/*     rendered inactive. Most of the parameter values were obtained */
/*     from the corresponding Bell Laboratories Port Library function. */

/*     The function statement is */

/*       double precision function dpmpar(i) */

/*     where */

/*       i is an integer input variable set to 1, 2, or 3 which */
/*         selects the desired machine parameter. If the machine has */
/*         t base b digits and its smallest and largest exponents are */
/*         emin and emax, respectively, then these parameters are */

/*         dpmpar(1) = b**(1 - t), the machine precision, */

/*         dpmpar(2) = b**(emin - 1), the smallest magnitude, */

/*         dpmpar(3) = b**emax*(1 - b**(-t)), the largest magnitude. */

/*     Argonne National Laboratory. MINPACK Project. June 1983. */
/*     Burton S. Garbow, Kenneth E. Hillstrom, Jorge J. More */

/*     Machine constants for IEEE double */

  double dmach[3] = {
    2.2204460492503130808472633361816406250000e-16,
    1.797693134862316e+308,
    2.225073858507202e-308
  };
  
  return dmach[*i - 1];
}
