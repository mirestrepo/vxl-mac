/* snlaso.f -- translated by f2c (version 19990326).
   You must link the resulting object file with the libraries:
        -lf2c -lm   (in that order)
*/

#include "f2c.h"

/* Table of constant values */

static integer c__0 = 0;
static integer c__1 = 1;
static doublereal c_b122 = 10.;
static real c_b195 = 0.f;


/* *********************************************************************** */

/* Subroutine */ void slabax_(n, nband, a, x, y)
integer *n, *nband;
real *a, *x, *y;
{
    /* System generated locals */
    integer a_dim1, a_offset;

    /* Local variables */
    static real zero[1];
    static integer i__, k, l, m;
    extern /* Subroutine */ void scopy_();

/*  THIS SUBROUTINE SETS Y = A*X */
/*  WHERE X AND Y ARE VECTORS OF LENGTH N */
/*  AND A IS AN  N X NBAND  SYMMETRIC BAND MATRIX */

/*  FORMAL PARAMETERS */

/*  LOCAL VARIABLES */

/*  FUNCTIONS CALLED */

/*  SUBROUTINES CALLED */

/*     SCOPY */

    /* Parameter adjustments */
    a_dim1 = *nband;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --x;
    --y;

    /* Function Body */
    zero[0] = 0.f;
    scopy_(n, zero, &c__0, &y[1], &c__1);
    for (k = 1; k <= *n; ++k) {
        y[k] += a[k * a_dim1 + 1] * x[k];
/* Computing MIN */
        m = min(*n-k+1,*nband);
        if (m < 2) {
            goto L20;
        }
        for (i__ = 2; i__ <= m; ++i__) {
            l = k + i__ - 1;
            y[l] += a[i__ + k * a_dim1] * x[k];
            y[k] += a[i__ + k * a_dim1] * x[l];
        }
L20:
        ;
    }
} /* slabax_ */


/* *********************************************************************** */

/* Subroutine */ void slabcm_(n, nband, nl, nr, a, eigval, lde, eigvec, atol,
        artol, bound, atemp, d__, vtemp)
integer *n, *nband, *nl, *nr;
real *a, *eigval;
integer *lde;
real *eigvec, *atol, *artol, *bound, *atemp, *d__, *vtemp;
{
    /* System generated locals */
    integer a_dim1, a_offset, eigvec_dim1, eigvec_offset, i__1;
    real r__1, r__2;

    /* Local variables */
    static logical flag__;
    static real errb;
    static integer nval;
    extern doublereal sdot_();
    static integer numl;
    extern doublereal snrm2_();
    static integer i__, j, l, m;
    static real sigma;
    extern /* Subroutine */ void sscal_();
    static real resid;
    extern /* Subroutine */ void scopy_();
    static real vnorm;
    extern /* Subroutine */ void saxpy_(), slabfc_();
    static real rq;
    extern /* Subroutine */ void slabax_(), slaran_();
    static integer numvec;
    static real gap;

/*  THIS SUBROUTINE ORGANIZES THE CALCULATION OF THE EIGENVALUES */
/*  FOR THE BNDEIG PACKAGE.  EIGENVALUES ARE COMPUTED BY */
/*  A MODIFIED RAYLEIGH QUOTIENT ITERATION.  THE EIGENVALUE COUNT */
/*  OBTAINED BY EACH FACTORIZATION IS USED TO OCCASIONALLY OVERRIDE */
/*  THE COMPUTED RAYLEIGH QUOTIENT WITH A DIFFERENT SHIFT TO */
/*  INSURE CONVERGENCE TO THE DESIRED EIGENVALUES. */

/*  FORMAL PARAMETERS. */

/*  LOCAL VARIABLES */

/*  FUNCTIONS CALLED */

/*  SUBROUTINES CALLED */

/*     SLABAX, SLABFC, SLARAN, SAXPY, SCOPY, SSCAL */

/*  REPLACE ZERO VECTORS BY RANDOM */

    /* Parameter adjustments */
    a_dim1 = *nband;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --eigval;
    eigvec_dim1 = *lde;
    eigvec_offset = 1 + eigvec_dim1 * 1;
    eigvec -= eigvec_offset;
    bound -= 3;
    --atemp;
    --d__;
    --vtemp;

    /* Function Body */
    nval = *nr - *nl + 1;
    flag__ = FALSE_;
    for (i__ = 1; i__ <= nval; ++i__) {
        if (sdot_(n, &eigvec[i__ * eigvec_dim1 + 1], &c__1, &eigvec[i__ *
                eigvec_dim1 + 1], &c__1) == 0.f) {
            slaran_(n, &eigvec[i__ * eigvec_dim1 + 1]);
        }
    }

/*  LOOP OVER EIGENVALUES */

    sigma = bound[(nval << 1) + 4];
    for (j = 1; j <= nval; ++j) {
        numl = j;

/*  PREPARE TO COMPUTE FIRST RAYLEIGH QUOTIENT */

L10:
        slabax_(n, nband, &a[a_offset], &eigvec[j * eigvec_dim1 + 1], &vtemp[
                1]);
        vnorm = snrm2_(n, &vtemp[1], &c__1);
        if (vnorm == 0.f) {
            goto L20;
        }
        r__1 = 1.f / vnorm;
        sscal_(n, &r__1, &vtemp[1], &c__1);
        r__1 = 1.f / vnorm;
        sscal_(n, &r__1, &eigvec[j * eigvec_dim1 + 1], &c__1);
        r__1 = -sigma;
        saxpy_(n, &r__1, &eigvec[j * eigvec_dim1 + 1], &c__1, &vtemp[1], &
                c__1);

/*  LOOP OVER SHIFTS */

/*  COMPUTE RAYLEIGH QUOTIENT, RESIDUAL NORM, AND CURRENT TOLERANCE */

L20:
        vnorm = snrm2_(n, &eigvec[j * eigvec_dim1 + 1], &c__1);
        if (vnorm != 0.f) {
            goto L30;
        }
        slaran_(n, &eigvec[j * eigvec_dim1 + 1]);
        goto L10;

L30:
        rq = sigma + sdot_(n, &eigvec[j * eigvec_dim1 + 1], &c__1, &vtemp[1],
                &c__1) / vnorm / vnorm;
        r__1 = sigma - rq;
        saxpy_(n, &r__1, &eigvec[j * eigvec_dim1 + 1], &c__1, &vtemp[1], &
                c__1);
/* Computing MAX */
        r__1 = *atol, r__2 = snrm2_(n, &vtemp[1], &c__1) / vnorm;
        resid = dmax(r__1,r__2);
        r__1 = 1.f / vnorm;
        sscal_(n, &r__1, &eigvec[j * eigvec_dim1 + 1], &c__1);

/*  ACCEPT EIGENVALUE IF THE INTERVAL IS SMALL ENOUGH */

        if (bound[(j << 1) + 4] - bound[(j << 1) + 3] < *atol * ( float)3.) {
            goto L300;
        }

/*  COMPUTE MINIMAL ERROR BOUND */

        errb = resid;
/* Computing MIN */
        r__1 = bound[(j << 1) + 5] - rq, r__2 = rq - bound[(j << 1) + 2];
        gap = dmin(r__1,r__2);
        if (gap > resid) {
/* Computing MAX */
            r__1 = *atol, r__2 = resid * resid / gap;
            errb = dmax(r__1,r__2);
        }

/*  TENTATIVE NEW SHIFT */

        sigma = (bound[(j << 1) + 3] + bound[(j << 1) + 4]) * .5f;

/*  CHECK FOR TERMINALTION */

        if (resid > *atol * 2.f) {
            goto L40;
        }
        if (rq - errb > bound[(j << 1) + 2] && rq + errb < bound[(j << 1) + 5]) {
            goto L310;
        }

/*  RQ IS TO THE LEFT OF THE INTERVAL */

L40:
        if (rq >= bound[(j << 1) + 3]) {
            goto L50;
        }
        if (rq - errb > bound[(j << 1) + 2]) {
            goto L100;
        }
        if (rq + errb < bound[(j << 1) + 3]) {
            slaran_(n, &eigvec[j * eigvec_dim1 + 1]);
        }
        goto L200;

/*  RQ IS TO THE RIGHT OF THE INTERVAL */

L50:
        if (rq <= bound[(j << 1) + 4]) {
            goto L100;
        }
        if (rq + errb < bound[(j << 1) + 5]) {
            goto L100;
        }

/*  SAVE THE REJECTED VECTOR IF INDICATED */

        if (rq - errb <= bound[(j << 1) + 4]) {
            goto L200;
        }
        for (i__ = j; i__ <= nval; ++i__) {
            if (bound[(i__ << 1) + 4] > rq) {
                goto L70;
            }
        }
        goto L80;

L70:
        scopy_(n, &eigvec[j * eigvec_dim1 + 1], &c__1, &eigvec[i__ *
                eigvec_dim1 + 1], &c__1);

L80:
        slaran_(n, &eigvec[j * eigvec_dim1 + 1]);
        goto L200;

/*  PERTURB RQ TOWARD THE MIDDLE */

L100:
        if (sigma < rq-errb) {
            sigma = rq-errb;
        }
        if (sigma > rq+errb) {
            sigma = rq+errb;
        }

/*  FACTOR AND SOLVE */

L200:
        for (i__ = j; i__ <= nval; ++i__) {
            if (sigma < bound[(i__ << 1) + 3]) {
                goto L220;
            }
        }
        i__ = nval + 1;
L220:
        numvec = i__ - j;
        numvec = min(numvec,*nband+2);
        if (resid < *artol) {
            numvec = min(1,numvec);
        }
        scopy_(n, &eigvec[j * eigvec_dim1 + 1], &c__1, &vtemp[1], &c__1);
        i__1 = (*nband << 1) - 1;
        slabfc_(n, nband, &a[a_offset], &sigma, &numvec, lde, &eigvec[j *
                eigvec_dim1 + 1], &numl, &i__1, &atemp[1], &d__[1], atol);

/*  PARTIALLY SCALE EXTRA VECTORS TO PREVENT UNDERFLOW OR OVERFLOW */

        if (numvec == 1) {
            goto L227;
        }
        l = numvec - 1;
        for (i__ = 1; i__ <= l; ++i__) {
            m = j + i__;
            r__1 = 1.f / vnorm;
            sscal_(n, &r__1, &eigvec[m * eigvec_dim1 + 1], &c__1);
        }

/*  UPDATE INTERVALS */

L227:
        numl = numl - *nl + 1;
        if (numl >= 0) {
            bound[4] = dmin(bound[4],sigma);
        }
        for (i__ = j; i__ <= nval; ++i__) {
            if (sigma < bound[(i__ << 1) + 3]) {
                goto L20;
            }
            if (numl < i__) {
                bound[(i__ << 1) + 3] = sigma;
            }
            if (numl >= i__) {
                bound[(i__ << 1) + 4] = sigma;
            }
        }
        if (numl < nval + 1) {
/* Computing MAX */
            r__1 = sigma, r__2 = bound[(nval << 1) + 5];
            bound[(nval << 1) + 5] = dmax(r__1,r__2);
        }
        goto L20;

/*  ACCEPT AN EIGENPAIR */

L300:
        slaran_(n, &eigvec[j * eigvec_dim1 + 1]);
        flag__ = TRUE_;
        goto L310;

L305:
        flag__ = FALSE_;
        rq = (bound[(j << 1) + 3] + bound[(j << 1) + 4]) * .5f;
        i__1 = (*nband << 1) - 1;
        slabfc_(n, nband, &a[a_offset], &rq, &numvec, lde, &eigvec[j * eigvec_dim1 + 1], &numl, &i__1, &atemp[1], &d__[1], atol);
        vnorm = snrm2_(n, &eigvec[j * eigvec_dim1 + 1], &c__1);
        if (vnorm != 0.f) {
            r__1 = 1.f / vnorm;
            sscal_(n, &r__1, &eigvec[j * eigvec_dim1 + 1], &c__1);
        }

/*  ORTHOGONALIZE THE NEW EIGENVECTOR AGAINST THE OLD ONES */

L310:
        eigval[j] = rq;
        if (j == 1) {
            goto L330;
        }
        m = j - 1;
        for (i__ = 1; i__ <= m; ++i__) {
            r__1 = -sdot_(n, &eigvec[i__ * eigvec_dim1 + 1], &c__1, &eigvec[j
                    * eigvec_dim1 + 1], &c__1);
            saxpy_(n, &r__1, &eigvec[i__ * eigvec_dim1 + 1], &c__1, &eigvec[j
                    * eigvec_dim1 + 1], &c__1);
        }
L330:
        vnorm = snrm2_(n, &eigvec[j * eigvec_dim1 + 1], &c__1);
        if (vnorm == 0.f) {
            goto L305;
        }
        r__1 = 1.f / vnorm;
        sscal_(n, &r__1, &eigvec[j * eigvec_dim1 + 1], &c__1);

/*   ORTHOGONALIZE LATER VECTORS AGAINST THE CONVERGED ONE */

        if (flag__) {
            goto L305;
        }
        if (j == nval) {
            return;
        }
        m = j + 1;
        for (i__ = m; i__ <= nval; ++i__) {
            r__1 = -sdot_(n, &eigvec[j * eigvec_dim1 + 1], &c__1, &eigvec[i__
                    * eigvec_dim1 + 1], &c__1);
            saxpy_(n, &r__1, &eigvec[j * eigvec_dim1 + 1], &c__1, &eigvec[i__
                    * eigvec_dim1 + 1], &c__1);
        }
    }
} /* slabcm_ */


/* *********************************************************************** */

/* Subroutine */ void slabfc_(n, nband, a, sigma, number, lde, eigvec, numl,
        ldad, atemp, d__, atol)
integer *n, *nband;
real *a, *sigma;
integer *number, *lde;
real *eigvec;
integer *numl, *ldad;
real *atemp, *d__, *atol;
{
    /* System generated locals */
    integer a_dim1, a_offset, eigvec_dim1, eigvec_offset, atemp_dim1,
            atemp_offset, d_dim1, d_offset, i__1;
    real r__1, r__2;

    /* Builtin functions */
    double r_sign();

    /* Local variables */
    static real zero[1];
    static integer i__, j, k, l, m;
    extern /* Subroutine */ void scopy_(), sswap_(), saxpy_();
    static integer la, ld, kk, nb1, lpm;

/*  THIS SUBROUTINE FACTORS (A-SIGMA*I) WHERE A IS A GIVEN BAND */
/*  MATRIX AND SIGMA IS AN INPUT PARAMETER.  IT ALSO SOLVES ZERO */
/*  OR MORE SYSTEMS OF LINEAR EQUATIONS.  IT RETURNS THE NUMBER */
/*  OF EIGENVALUES OF A LESS THAN SIGMA BY COUNTING THE STURM */
/*  SEQUENCE DURING THE FACTORIZATION.  TO OBTAIN THE STURM */
/*  SEQUENCE COUNT WHILE ALLOWING NON-SYMMETRIC PIVOTING FOR */
/*  STABILITY, THE CODE USES A GUPTA'S MULTIPLE PIVOTING */
/*  ALGORITHM. */

/*  FORMAL PARAMETERS */

/*  LOCAL VARIABLES */

/*  FUNCTIONS CALLED */

/*  SUBROUTINES CALLED */

/*     SAXPY, SCOPY, SSWAP */

/*  INITIALIZE */

    /* Parameter adjustments */
    a_dim1 = *nband;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    eigvec_dim1 = *lde;
    eigvec_offset = 1 + eigvec_dim1 * 1;
    eigvec -= eigvec_offset;
    d_dim1 = *ldad;
    d_offset = 1 + d_dim1 * 1;
    d__ -= d_offset;
    atemp_dim1 = *ldad;
    atemp_offset = 1 + atemp_dim1 * 1;
    atemp -= atemp_offset;

    /* Function Body */
    zero[0] = 0.f;
    nb1 = *nband - 1;
    *numl = 0;
    i__1 = *ldad * *nband;
    scopy_(&i__1, zero, &c__0, &d__[d_offset], &c__1);

/*   LOOP OVER COLUMNS OF A */

    for (k = 1; k <= *n; ++k) {

/*   ADD A COLUMN OF A TO D */

        d__[*nband + *nband * d_dim1] = a[k * a_dim1 + 1] - *sigma;
        m = min(k,*nband) - 1;
        if (m == 0) {
            goto L20;
        }
        for (i__ = 1; i__ <= m; ++i__) {
            la = k - i__;
            ld = *nband - i__;
            d__[ld + *nband * d_dim1] = a[i__ + 1 + la * a_dim1];
        }

L20:
/* Computing MIN */
        m = min(*n-k,nb1);
        if (m == 0) {
            goto L40;
        }
        for (i__ = 1; i__ <= m; ++i__) {
            ld = *nband + i__;
            d__[ld + *nband * d_dim1] = a[i__ + 1 + k * a_dim1];
        }

/*   TERMINATE */

L40:
        lpm = 1;
        if (nb1 == 0) {
            goto L70;
        }
        for (i__ = 1; i__ <= nb1; ++i__) {
            l = k - *nband + i__;
            if (d__[i__ + *nband * d_dim1] == 0.f) {
                goto L60;
            }
            if (dabs(d__[i__ + i__ * d_dim1]) >= dabs(d__[i__ + *nband * d_dim1])) {
                goto L50;
            }
            if (d__[i__ + *nband * d_dim1] < 0.f && d__[i__ + i__ *
                    d_dim1] < 0.f || d__[i__ + *nband * d_dim1] > (
                    float)0. && d__[i__ + i__ * d_dim1] >= 0.f) {
                lpm = -lpm;
            }
            i__1 = *ldad - i__ + 1;
            sswap_(&i__1, &d__[i__ + i__ * d_dim1], &c__1, &d__[i__ + *nband * d_dim1], &c__1);
            sswap_(number, &eigvec[l + eigvec_dim1], lde, &eigvec[k + eigvec_dim1], lde);
L50:
            i__1 = *ldad - i__;
            r__1 = -d__[i__ + *nband * d_dim1] / d__[i__ + i__ * d_dim1];
            saxpy_(&i__1, &r__1, &d__[i__ + 1 + i__ * d_dim1], &c__1, &d__[i__ + 1 + *nband * d_dim1], &c__1);
            r__1 = -d__[i__ + *nband * d_dim1] / d__[i__ + i__ * d_dim1];
            saxpy_(number, &r__1, &eigvec[l + eigvec_dim1], lde, &eigvec[k + eigvec_dim1], lde);
L60:
            ;
        }

/*  UPDATE STURM SEQUENCE COUNT */

L70:
        if (d__[*nband + *nband * d_dim1] < 0.f) {
            lpm = -lpm;
        }
        if (lpm < 0) {
            ++(*numl);
        }
        if (k == *n) {
            goto L110;
        }

/*   COPY FIRST COLUMN OF D INTO ATEMP */
        if (k < *nband) {
            goto L80;
        }
        l = k - nb1;
        scopy_(ldad, &d__[d_offset], &c__1, &atemp[l * atemp_dim1 + 1], &c__1)
                ;

/*   SHIFT THE COLUMNS OF D OVER AND UP */

        if (nb1 == 0) {
            goto L100;
        }
L80:
        for (i__ = 1; i__ <= nb1; ++i__) {
            i__1 = *ldad - i__;
            scopy_(&i__1, &d__[i__ + 1 + (i__ + 1) * d_dim1], &c__1, &d__[i__ + i__ * d_dim1], &c__1);
            d__[*ldad + i__ * d_dim1] = 0.f;
        }
L100:
        ;
    }

/*  TRANSFER D TO ATEMP */

L110:
    for (i__ = 1; i__ <= *nband; ++i__) {
        l = *n - *nband + i__;
        i__1 = *nband - i__ + 1;
        scopy_(&i__1, &d__[i__ + i__ * d_dim1], &c__1, &atemp[l * atemp_dim1 + 1], &c__1);
    }

/*   BACK SUBSTITUTION */

    if (*number == 0) {
        return;
    }
    for (kk = 1; kk <= *n; ++kk) {
        k = *n - kk + 1;
        if (dabs(atemp[k * atemp_dim1 + 1]) <= *atol) {
            atemp[k * atemp_dim1 + 1] = r_sign(atol, &atemp[k * atemp_dim1 +
                    1]);
        }

        for (i__ = 1; i__ <= *number; ++i__) {
            eigvec[k + i__ * eigvec_dim1] /= atemp[k * atemp_dim1 + 1];
            m = min(*ldad,k) - 1;
            if (m == 0) {
                goto L150;
            }
            for (j = 1; j <= m; ++j) {
                l = k - j;
                eigvec[l + i__ * eigvec_dim1] -= atemp[j + 1 + l * atemp_dim1]
                         * eigvec[k + i__ * eigvec_dim1];
            }
L150:
            ;
        }
    }
} /* slabfc_ */

/* Subroutine */ void slaeig_(n, nband, nl, nr, a, eigval, lde, eigvec, bound,
        atemp, d__, vtemp, eps, tmin, tmax)
integer *n, *nband, *nl, *nr;
real *a, *eigval;
integer *lde;
real *eigvec, *bound, *atemp, *d__, *vtemp, *eps, *tmin, *tmax;
{
    /* System generated locals */
    integer a_dim1, a_offset, eigvec_dim1, eigvec_offset;
    real r__1, r__2;

    /* Builtin functions */
    double sqrt();

    /* Local variables */
    static real atol;
    static integer nval, i__, m;
    static real artol;
    extern /* Subroutine */ void slabcm_();

/*  THIS IS A SPECIALIZED VERSION OF THE SUBROUTINE BNDEIG TAILORED */
/*  SPECIFICALLY FOR USE BY THE LASO PACKAGE. */

/*  LOCAL VARIABLES */

/*  FUNCTIONS CALLED */

/*  SUBROUTINES CALLED */

/*     SLABCM, SLABFC, SLAGER, SCOPY */

/*  SET PARAMETERS */

    /* Parameter adjustments */
    a_dim1 = *nband;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;
    --eigval;
    eigvec_dim1 = *lde;
    eigvec_offset = 1 + eigvec_dim1 * 1;
    eigvec -= eigvec_offset;
    bound -= 3;
    --atemp;
    --d__;
    --vtemp;

    /* Function Body */
/* Computing MAX */
    r__1 = *tmax, r__2 = -(*tmin);
    atol = (real) (*n) * *eps * dmax(r__1,r__2);
    artol = atol / sqrt(*eps);
    nval = *nr - *nl + 1;

/*   CHECK FOR SPECIAL CASE OF N = 1 */

    if (*n != 1) {
        goto L30;
    }
    eigval[1] = a[a_dim1 + 1];
    eigvec[eigvec_dim1 + 1] = 1.f;
    return;

/*   SET UP INITIAL EIGENVALUE BOUNDS */

L30:
    m = nval + 1;
    for (i__ = 2; i__ <= m; ++i__) {
        bound[(i__ << 1) + 1] = *tmin;
        bound[(i__ << 1) + 2] = *tmax;
    }
    bound[4] = *tmax;
    bound[(nval << 1) + 5] = *tmin;
    if (*nl == 1) {
        bound[4] = *tmin;
    }
    if (*nr == *n) {
        bound[(nval << 1) + 5] = *tmax;
    }

    slabcm_(n, nband, nl, nr, &a[a_offset], &eigval[1], lde, &eigvec[eigvec_offset],
            &atol, &artol, &bound[3], &atemp[1], &d__[1], &vtemp[1]);
} /* slaeig_ */


/* *********************************************************************** */

/* Subroutine */ void slager_(n, nband, nstart, a, tmin, tmax)
integer *n, *nband, *nstart;
real *a, *tmin, *tmax;
{
    /* System generated locals */
    integer a_dim1, a_offset;
    real r__1, r__2;

    /* Local variables */
    static real temp;
    static integer i__, k, l, m;

/*  THIS SUBROUTINE COMPUTES BOUNDS ON THE SPECTRUM OF A BY */
/*  EXAMINING THE GERSCHGORIN CIRCLES. ONLY THE NEWLY CREATED */
/*  CIRCLES ARE EXAMINED */

/*  FORMAL PARAMETERS */

/*  LOCAL VARIABLES */

/*  FUNCTIONS CALLED */

    /* Parameter adjustments */
    a_dim1 = *nband;
    a_offset = 1 + a_dim1 * 1;
    a -= a_offset;

    /* Function Body */
    for (k = *nstart; k <= *n; ++k) {
        temp = 0.f;
        for (i__ = 2; i__ <= *nband; ++i__) {
            temp += dabs(a[i__ + k * a_dim1]);
        }
        l = min(k,*nband);
        if (l == 1) {
            goto L40;
        }
        for (i__ = 2; i__ <= l; ++i__) {
            m = k - i__ + 1;
            temp += dabs(a[i__ + m * a_dim1]);
        }
L40:
/* Computing MIN */
        r__1 = *tmin, r__2 = a[k * a_dim1 + 1] - temp;
        *tmin = dmin(r__1,r__2);
/* Computing MAX */
        r__1 = *tmax, r__2 = a[k * a_dim1 + 1] + temp;
        *tmax = dmax(r__1,r__2);
    }
} /* slager_ */


/* *********************************************************************** */

/* Subroutine */ void slaran_(n, x)
integer *n;
real *x;
{
    /* Initialized data */

    static integer iurand = 0;

    /* Local variables */
    static integer i__;
    extern doublereal urand_();

/*  THIS SUBROUTINE SETS THE VECTOR X TO RANDOM NUMBERS */

/*  FORMAL PARAMETERS */

/*  LOCAL VARIABLES */

/*  FUNCTIONS CALLED */

/*  SUBROUTINES CALLED */

/*     NONE */

/*  INITIALIZE SEED */

    /* Parameter adjustments */
    --x;

    /* Function Body */

    for (i__ = 1; i__ <= *n; ++i__) {
        x[i__] = urand_(&iurand) - .5f;
    }
} /* slaran_ */


/* ------------------------------------------------------------------ */

/* Subroutine */ void smvpc_(nblock, bet, maxj, j, s, number, resnrm, orthcf,
        rv)
integer *nblock;
real *bet;
integer *maxj, *j;
real *s;
integer *number;
real *resnrm, *orthcf, *rv;
{
    /* System generated locals */
    integer bet_dim1, bet_offset, s_dim1, s_offset;
    real r__1, r__2, r__3;

    /* Local variables */
    extern doublereal sdot_(), snrm2_();
    static integer i__, k, m;

/* THIS SUBROUTINE COMPUTES THE NORM AND THE SMALLEST ELEMENT */
/* (IN ABSOLUTE VALUE) OF THE VECTOR BET*SJI, WHERE SJI */
/* IS AN NBLOCK VECTOR OF THE LAST NBLOCK ELEMENTS OF THE ITH */
/* EIGENVECTOR OF T.  THESE QUANTITIES ARE THE RESIDUAL NORM */
/* AND THE ORTHOGONALITY COEFFICIENT RESPECTIVELY FOR THE */
/* CORRESPONDING RITZ PAIR.  THE ORTHOGONALITY COEFFICIENT IS */
/* NORMALIZED TO ACCOUNT FOR THE LOCAL REORTHOGONALIZATION. */

    /* Parameter adjustments */
    bet_dim1 = *nblock;
    bet_offset = 1 + bet_dim1 * 1;
    bet -= bet_offset;
    s_dim1 = *maxj;
    s_offset = 1 + s_dim1 * 1;
    s -= s_offset;
    --resnrm;
    --orthcf;
    --rv;

    /* Function Body */
    m = *j - *nblock + 1;
    for (i__ = 1; i__ <= *number; ++i__) {
        for (k = 1; k <= *nblock; ++k) {
            rv[k] = sdot_(nblock, &s[m + i__ * s_dim1], &c__1, &bet[k +
                    bet_dim1], nblock);
            if (k == 1) {
                orthcf[i__] = dabs(rv[k]);
            }
/* Computing MIN */
            r__2 = orthcf[i__], r__3 = dabs(rv[k]);
            orthcf[i__] = dmin(r__2,r__3);
        }
        resnrm[i__] = snrm2_(nblock, &rv[1], &c__1);
    }
} /* smvpc_ */

/*   VERSION 2    DOES NOT USE EISPACK */

/* ------------------------------------------------------------------ */

/* Subroutine */ void snlaso_(op, iovect, n, nval, nfig, nperm, nmval, val,
        nmvec, vec, nblock, maxop, maxj, work, ind, ierr)
/* Subroutine */ void (*op) (), (*iovect) ();
integer *n, *nval, *nfig, *nperm, *nmval;
real *val;
integer *nmvec;
real *vec;
integer *nblock, *maxop, *maxj;
real *work;
integer *ind, *ierr;
{
    /* System generated locals */
    integer vec_dim1, vec_offset, val_dim1, val_offset, i__1, i__2;
    real r__1;

    /* Local variables */
    static real temp, tarr[1];
    extern doublereal snrm2_();
    static integer i__, m, nband;
    static real delta;
    static logical small;
    extern /* Subroutine */ void snwla_();
    static integer i1, i2, i3, i4, i5, i6, i7, i8, i9;
    extern /* Subroutine */ void scopy_();
    static integer i10, i11, i12, i13, nv;
    extern /* Subroutine */ void snppla_();
    static logical raritz;
    extern /* Subroutine */ void sortqr_(), svsort_();
    static real eps;
    static integer nop;

/* AUTHOR/IMPLEMENTER D.S.SCOTT-B.N.PARLETT/D.S.SCOTT */

/* COMPUTER SCIENCES DEPARTMENT */
/* UNIVERSITY OF TEXAS AT AUSTIN */
/* AUSTIN, TX 78712 */

/* VERSION 2 ORIGINATED APRIL 1982 */

/* CURRENT VERSION  JUNE 1983 */

/* SNLASO FINDS A FEW EIGENVALUES AND EIGENVECTORS AT EITHER END OF */
/* THE SPECTRUM OF A LARGE SPARSE SYMMETRIC MATRIX.  THE SUBROUTINE */
/* SNLASO IS PRIMARILY A DRIVER FOR SUBROUTINE SNWLA WHICH IMPLEMENTS */
/* THE LANCZOS ALGORITHM WITH SELECTIVE ORTHOGONALIZATION AND */
/* SUBROUTINE SNPPLA WHICH POST PROCESSES THE OUTPUT OF SNWLA. */
/* HOWEVER SNLASO DOES CHECK FOR INCONSISTENCIES IN THE CALLING */
/* PARAMETERS AND DOES PREPROCESS ANY USER SUPPLIED EIGENPAIRS. */
/* SNLASO ALWAYS LOOKS FOR THE SMALLEST (LEFTMOST) EIGENVALUES.  IF */
/* THE LARGEST EIGENVALUES ARE DESIRED SNLASO IMPLICITLY USES THE */
/* NEGATIVE OF THE MATRIX. */


/* ON INPUT */


/*   OP   A USER SUPPLIED SUBROUTINE WITH CALLING SEQUENCE */
/*     OP(N,M,P,Q).  P AND Q ARE N X M MATRICES AND Q IS */
/*     RETURNED AS THE MATRIX TIMES P. */

/*   IOVECT   A USER SUPPLIED SUBROUTINE WITH CALLING SEQUENCE */
/*     IOVECT(N,M,Q,J,K).  Q IS AN N X M MATRIX.  IF K = 0 */
/*     THE COLUMNS OF Q ARE STORED AS THE (J-M+1)TH THROUGH */
/*     THE JTH LANCZOS VECTORS.  IF K = 1 THEN Q IS RETURNED */
/*     AS THE (J-M+1)TH THROUGH THE JTH LANCZOS VECTORS.  SEE */
/*     DOCUMENTATION FOR FURTHER DETAILS AND EXAMPLES. */

/*   N   THE ORDER OF THE MATRIX. */

/*   NVAL   NVAL SPECIFIES THE EIGENVALUES TO BE FOUND. */
/*     ABS(NVAL)  IS THE NUMBER OF EIGENVALUES DESIRED. */
/*     IF NVAL < 0 THE ALGEBRAICALLY SMALLEST (LEFTMOST) */
/*     EIGENVALUES ARE FOUND.  IF NVAL > 0 THE ALGEBRAICALLY */
/*     LARGEST (RIGHTMOST) EIGENVALUES ARE FOUND.  NVAL MUST NOT */
/*     BE ZERO.  ABS(NVAL) MUST BE LESS THAN  MAXJ/2. */

/*   NFIG   THE NUMBER OF DECIMAL DIGITS OF ACCURACY DESIRED IN THE */
/*     EIGENVALUES.  NFIG MUST BE GREATER THAN OR EQUAL TO 1. */

/*   NPERM   AN INTEGER VARIABLE WHICH SPECIFIES THE NUMBER OF USER */
/*     SUPPLIED EIGENPAIRS.  IN MOST CASES NPERM WILL BE ZERO.  SEE */
/*     DOCUMENTAION FOR FURTHER DETAILS OF USING NPERM GREATER */
/*     THAN ZERO.  NPERM MUST NOT BE LESS THAN ZERO. */

/*   NMVAL   THE ROW DIMENSION OF THE ARRAY VAL.  NMVAL MUST BE GREATER */
/*     THAN OR EQUAL TO ABS(NVAL). */

/*   VAL   A TWO DIMENSIONAL REAL ARRAY OF ROW */
/*     DIMENSION NMVAL AND COLUMN DIMENSION AT LEAST 4.  IF NPERM */
/*     IS GREATER THAN ZERO THEN CERTAIN INFORMATION MUST BE STORED */
/*     IN VAL.  SEE DOCUMENTATION FOR DETAILS. */

/*   NMVEC   THE ROW DIMENSION OF THE ARRAY VEC.  NMVEC MUST BE GREATER */
/*     THAN OR EQUAL TO N. */

/*   VEC   A TWO DIMENSIONAL REAL ARRAY OF ROW */
/*     DIMENSION NMVEC AND COLUMN DIMENSION AT LEAST ABS(NVAL).  IF */
/*     NPERM > 0 THEN THE FIRST NPERM COLUMNS OF VEC MUST */
/*     CONTAIN THE USER SUPPLIED EIGENVECTORS. */

/*   NBLOCK   THE BLOCK SIZE.  SEE DOCUMENTATION FOR CHOOSING */
/*     AN APPROPRIATE VALUE FOR NBLOCK.  NBLOCK MUST BE GREATER */
/*     THAN ZERO AND LESS THAN  MAXJ/6. */

/*   MAXOP   AN UPPER BOUND ON THE NUMBER OF CALLS TO THE SUBROUTINE */
/*     OP.  SNLASO TERMINATES WHEN MAXOP IS EXCEEDED.  SEE */
/*     DOCUMENTATION FOR GUIDELINES IN CHOOSING A VALUE FOR MAXOP. */

/*   MAXJ   AN INDICATION OF THE AVAILABLE STORAGE (SEE WORK AND */
/*     DOCUMENTATION ON IOVECT).  FOR THE FASTEST CONVERGENCE MAXJ */
/*     SHOULD BE AS LARGE AS POSSIBLE, ALTHOUGH IT IS USELESS TO HAVE */
/*     MAXJ LARGER THAN MAXOP*NBLOCK. */

/*   WORK   A REAL ARRAY OF DIMENSION AT LEAST AS */
/*     LARGE AS */

/*         2*N*NBLOCK + MAXJ*(NBLOCK+NV+2) + 2*NBLOCK*NBLOCK + 3*NV */

/*            + THE MAXIMUM OF */
/*                 N*NBLOCK */
/*                   AND */
/*         MAXJ*(2*NBLOCK+3) + 2*NV + 6 + (2*NBLOCK+2)*(NBLOCK+1) */

/*     WHERE NV = ABS(NVAL) */

/*     THE FIRST N*NBLOCK ELEMENTS OF WORK MUST CONTAIN THE DESIRED */
/*     STARTING VECTORS.  SEE DOCUMENTATION FOR GUIDELINES IN */
/*     CHOOSING STARTING VECTORS. */

/*   IND   AN INTEGER ARRAY OF DIMENSION AT LEAST ABS(NVAL). */

/*   IERR   AN INTEGER VARIABLE. */


/* ON OUTPUT */


/*   NPERM   THE NUMBER OF EIGENPAIRS NOW KNOWN. */

/*   VEC   THE FIRST NPERM COLUMNS OF VEC CONTAIN THE EIGENVECTORS. */

/*   VAL   THE FIRST COLUMN OF VAL CONTAINS THE CORRESPONDING */
/*     EIGENVALUES.  THE SECOND COLUMN CONTAINS THE RESIDUAL NORMS OF */
/*     THE EIGENPAIRS WHICH ARE BOUNDS ON THE ACCURACY OF THE EIGEN- */
/*     VALUES.  THE THIRD COLUMN CONTAINS MORE REALISTIC ESTIMATES */
/*     OF THE ACCURACY OF THE EIGENVALUES.  THE FOURTH COLUMN CONTAINS */
/*     ESTIMATES OF THE ACCURACY OF THE EIGENVECTORS.  SEE */
/*     DOCUMENTATION FOR FURTHER INFORMATION ON THESE QUANTITIES. */

/*   WORK   IF WORK IS TERMINATED BEFORE COMPLETION (IERR = -2) */
/*     THE FIRST N*NBLOCK ELEMENTS OF WORK CONTAIN THE BEST VECTORS */
/*     FOR RESTARTING THE ALGORITHM AND SNLASO CAN BE IMMEDIATELY */
/*     RECALLED TO CONTINUE WORKING ON THE PROBLEM. */

/*   IND   IND(1)  CONTAINS THE ACTUAL NUMBER OF CALLS TO OP.  ON SOME */
/*     OCCASIONS THE NUMBER OF CALLS TO OP MAY BE SLIGHTLY LARGER */
/*     THAN MAXOP. */

/*   IERR   AN ERROR COMPLETION CODE.  THE NORMAL COMPLETION CODE IS */
/*     ZERO.  SEE THE DOCUMENTATION FOR INTERPRETATIONS OF NON-ZERO */
/*     COMPLETION CODES. */


/* INTERNAL VARIABLES. */



/* NOP   RETURNED FROM SNWLA AS THE NUMBER OF CALLS TO THE */
/*   SUBROUTINE OP. */

/* NV   SET EQUAL TO ABS(NVAL), THE NUMBER OF EIGENVALUES DESIRED, */
/*   AND PASSED TO SNWLA. */

/* SMALL   SET TO .TRUE. IF THE SMALLEST EIGENVALUES ARE DESIRED. */

/* RARITZ   RETURNED FROM SNWLA AND PASSED TO SNPPLA.  RARITZ IS .TRUE. */
/*   IF A FINAL RAYLEIGH-RITZ PROCEDURE IS NEEDED. */

/* DELTA   RETURNED FROM SNWLA AS THE EIGENVALUE OF THE MATRIX */
/*   WHICH IS CLOSEST TO THE DESIRED EIGENVALUES. */

/* SNPPLA   A SUBROUTINE FOR POST-PROCESSING THE EIGENVECTORS COMPUTED */
/*   BY SNWLA. */

/* SNWLA   A SUBROUTINE FOR IMPLEMENTING THE LANCZOS ALGORITHM */
/*   WITH SELECTIVE ORTHOGONALIZATION. */

/* SMVPC   A SUBROUTINE FOR COMPUTING THE RESIDUAL NORM AND */
/*   ORTHOGONALITY COEFFICIENT OF GIVEN RITZ VECTORS. */

/* SORTQR   A SUBROUTINE FOR ORTHONORMALIZING A BLOCK OF VECTORS */
/*   USING HOUSEHOLDER REFLECTIONS. */

/* SAXPY,SCOPY,SDOT,SNRM2,SSCAL,SSWAP   A SUBSET OF THE BASIC LINEAR */
/*   ALGEBRA SUBPROGRAMS USED FOR VECTOR MANIPULATION. */

/* SLARAN   A SUBROUTINE TO GENERATE RANDOM VECTORS */

/* SLAEIG, SLAGER, SLABCM, SLABFC   SUBROUTINES FOR BAND EIGENVALUE */
/*   CALCULATIONS. */

/* ------------------------------------------------------------------ */

/* THIS SECTION CHECKS FOR INCONSISTENCY IN THE INPUT PARAMETERS. */

    /* Parameter adjustments */
    val_dim1 = *nmval;
    val_offset = 1 + val_dim1 * 1;
    val -= val_offset;
    vec_dim1 = *nmvec;
    vec_offset = 1 + vec_dim1 * 1;
    vec -= vec_offset;
    --work;
    --ind;

    /* Function Body */
    nv = abs(*nval);
    ind[1] = 0;
    *ierr = 0;
    if (*n < *nblock * 6) {
        *ierr = 1;
    }
    if (*nfig <= 0) {
        *ierr += 2;
    }
    if (*nmvec < *n) {
        *ierr += 4;
    }
    if (*nperm < 0) {
        *ierr += 8;
    }
    if (*maxj < *nblock * 6) {
        *ierr += 16;
    }
    if (nv < max(1,*nperm)) {
        *ierr += 32;
    }
    if (nv > *nmval) {
        *ierr += 64;
    }
    if (nv > *maxop) {
        *ierr += 128;
    }
    if (nv >= *maxj / 2) {
        *ierr += 256;
    }
    if (*nblock < 1) {
        *ierr += 512;
    }
    if (*ierr != 0) {
        return;
    }

    small = *nval < 0;

/* ------------------------------------------------------------------ */

/* THIS SECTION SORTS AND ORTHONORMALIZES THE USER SUPPLIED VECTORS. */
/* IF A USER SUPPLIED VECTOR IS ZERO OR IF SIGNIFICANT CANCELLATION */
/* OCCURS IN THE ORTHOGONALIZATION PROCESS THEN IERR IS SET TO  -1 */
/* AND SNLASO TERMINATES. */

    if (*nperm == 0) {
        goto L110;
    }

/* THIS NEGATES THE USER SUPPLIED EIGENVALUES WHEN THE LARGEST */
/* EIGENVALUES ARE DESIRED, SINCE SNWLA WILL IMPLICITLY USE THE */
/* NEGATIVE OF THE MATRIX. */

    if (small) {
        goto L20;
    }
    for (i__ = 1; i__ <= *nperm; ++i__) {
        val[i__ + val_dim1] = -val[i__ + val_dim1];
    }

/* THIS SORTS THE USER SUPPLIED VALUES AND VECTORS. */

L20:
    svsort_(nperm, &val[val_offset], &val[(val_dim1 << 1) + 1], &c__0, tarr,
            nmvec, n, &vec[vec_offset]);

/* THIS STORES THE NORMS OF THE VECTORS FOR LATER COMPARISON. */
/* IT ALSO INSURES THAT THE RESIDUAL NORMS ARE POSITIVE. */

    for (i__ = 1; i__ <= *nperm; ++i__) {
        val[i__ + (val_dim1 << 1)] = dabs(val[i__ + (val_dim1 << 1)]);
        val[i__ + val_dim1 * 3] = snrm2_(n, &vec[i__ * vec_dim1 + 1], &c__1);
    }

/* THIS PERFORMS THE ORTHONORMALIZATION. */

    m = *n * *nblock + 1;
    sortqr_(nmvec, n, nperm, &vec[vec_offset], &work[m]);
    m = *n * *nblock - *nperm;
    for (i__ = 1; i__ <= *nperm; ++i__) {
        m = m + *nperm + 1;
        if (dabs(work[m]) > val[i__ + val_dim1 * 3] * .9f) {
            goto L70;
        }
        *ierr = -1;
        return;
L70:
        ;
    }

/* THIS COPIES THE RESIDUAL NORMS INTO THE CORRECT LOCATIONS IN */
/* THE ARRAY WORK FOR LATER REFERENCE IN SNWLA. */

    m = (*n << 1) * *nblock + 1;
    scopy_(nperm, &val[(val_dim1 << 1) + 1], &c__1, &work[m], &c__1);

/* THIS SETS EPS TO AN APPROXIMATION OF THE RELATIVE MACHINE */
/* PRECISION */

/* ***THIS SHOULD BE REPLACED BY AN ASSIGNMENT STATEMENT */
/* ***IN A PRODUCTION CODE */

L110:
    eps = 1.f;
    for (i__ = 1; i__ <= 1000; ++i__) {
        eps *= .5f;
        temp = eps + 1.f;
        if (temp == 1.f) {
            goto L130;
        }
    }

/* ------------------------------------------------------------------ */

/* THIS SECTION CALLS SNWLA WHICH IMPLEMENTS THE LANCZOS ALGORITHM */
/* WITH SELECTIVE ORTHOGONALIZATION. */

L130:
    nband = *nblock + 1;
    i1 = *n * *nblock + 1;
    i2 = i1 + *n * *nblock;
    i3 = i2 + nv;
    i4 = i3 + nv;
    i5 = i4 + nv;
    i6 = i5 + *maxj * nband;
    i7 = i6 + *nblock * *nblock;
    i8 = i7 + *nblock * *nblock;
    i9 = i8 + *maxj * (nv + 1);
    i10 = i9 + *nblock;
    i11 = i10 + (nv << 1) + 6;
    i12 = i11 + *maxj * ((*nblock << 1) + 1);
    i13 = i12 + *maxj;
    snwla_(op, iovect, n, &nband, &nv, nfig, nperm, &val[val_offset], nmvec, &
            vec[vec_offset], nblock, maxop, maxj, &nop, &work[1], &work[i1], &
            work[i2], &work[i3], &work[i4], &work[i5], &work[i6], &work[i7], &
            work[i8], &work[i9], &work[i10], &work[i11], &work[i12], &work[
            i13], &ind[1], &small, &raritz, &delta, &eps, ierr);

/* ------------------------------------------------------------------ */

/* THIS SECTION CALLS SNPPLA (THE POST PROCESSOR). */

    if (*nperm == 0) {
        goto L140;
    }
    i1 = *n * *nblock + 1;
    i2 = i1 + *nperm * *nperm;
    i3 = i2 + *nperm * *nperm;
/* Computing MAX */
    i__1 = *n * *nblock, i__2 = (*nperm << 1) * *nperm;
    i4 = i3 + max(i__1,i__2);
    i5 = i4 + *n * *nblock;
    i6 = i5 + (*nperm << 1) + 4;
    snppla_(op, iovect, n, nperm, &nop, nmval, &val[val_offset], nmvec, &vec[
            vec_offset], nblock, &work[i1], &work[i2], &work[i3], &work[i4], &
            work[i5], &work[i6], &delta, &small, &raritz, &eps);

L140:
    ind[1] = nop;
    return;
} /* snlaso_ */


/* ------------------------------------------------------------------ */

/* Subroutine */ void snwla_(op, iovect, n, nband, nval, nfig, nperm, val,
        nmvec, vec, nblock, maxop, maxj, nop, p1, p0, res, tau, otau, t, alp,
        bet, s, p2, bound, atemp, vtemp, d__, ind, small, raritz, delta, eps,
        ierr)
/* Subroutine */ void (*op) (), (*iovect) ();
integer *n, *nband, *nval, *nfig, *nperm;
real *val;
integer *nmvec;
real *vec;
integer *nblock, *maxop, *maxj, *nop;
real *p1, *p0, *res, *tau, *otau, *t, *alp, *bet, *s, *p2, *bound, *atemp, *
        vtemp, *d__;
integer *ind;
logical *small, *raritz;
real *delta, *eps;
integer *ierr;
{
    /* System generated locals */
    integer vec_dim1, vec_offset, p0_dim1, p0_offset, p1_dim1, p1_offset,
            p2_dim1, p2_offset, t_dim1, t_offset, alp_dim1, alp_offset,
            bet_dim1, bet_offset, s_dim1, s_offset, i__1;
    real r__1, r__2, r__3;
    doublereal d__1;

    /* Builtin functions */
    double sqrt(), pow_dd();

    /* Local variables */
    static real tola, temp, tolg, tmin, tmax;
    extern doublereal sdot_();
    static real tarr[1];
    static logical test;
    static real zero[1], utol;
    extern doublereal snrm2_();
    static integer i__, j, k, l, m;
    extern /* Subroutine */ void sscal_();
    static integer ngood, nleft;
    static real anorm;
    static integer mtemp;
    extern /* Subroutine */ void smvpc_();
    static integer i1;
    static real pnorm, epsrt, rnorm;
    extern /* Subroutine */ void scopy_(), saxpy_();
    static integer ii, ng;
    extern /* Subroutine */ void slaeig_(), slager_();
    static real betmin, alpmin, betmax, alpmax;
    static integer ntheta;
    extern /* Subroutine */ void slaran_();
    static logical enough;
    static integer number, nstart;
    extern /* Subroutine */ void sortqr_(), svsort_();

/* SNWLA IMPLEMENTS THE LANCZOS ALGORITHM WITH SELECTIVE */
/* ORTHOGONALIZATION. */

/*   NBAND  NBLOCK + 1  THE BAND WIDTH OF T. */

/*   NVAL   THE NUMBER OF DESIRED EIGENVALUES. */

/*   NPERM   THE NUMBER OF PERMANENT VECTORS (THOSE EIGENVECTORS */
/*     INPUT BY THE USER OR THOSE EIGENVECTORS SAVED WHEN THE */
/*     ALGORITHM IS ITERATED).  PERMANENT VECTORS ARE ORTHOGONAL */
/*     TO THE CURRENT KRYLOV SUBSPACE. */

/*   NOP   THE NUMBER OF CALLS TO OP. */

/*   P0, P1, AND P2   THE CURRENT BLOCKS OF LANCZOS VECTORS. */

/*   RES   THE (APPROXIMATE) RESIDUAL NORMS OF THE PERMANENT VECTORS. */

/*   TAU AND OTAU   USED TO MONITOR THE NEED FOR ORTHOGONALIZATION. */

/*   T   THE BAND MATRIX. */

/*   ALP   THE CURRENT DIAGONAL BLOCK. */

/*   BET   THE CURRENT OFF DIAGONAL BLOCK. */

/*   BOUND, ATEMP, VTEMP, D  TEMPORARY STORAGE USED BY THE BAND */
/*      EIGENVALUE SOLVER SLAEIG. */

/*   S   EIGENVECTORS OF T. */

/*   SMALL   .TRUE.  IF THE SMALL EIGENVALUES ARE DESIRED. */

/*   RARITZ  RETURNED AS  .TRUE.  IF A FINAL RAYLEIGH-RITZ PROCEDURE */
/*     IS TO BE DONE. */

/*   DELTA   RETURNED AS THE VALUE OF THE (NVAL+1)TH EIGENVALUE */
/*     OF THE MATRIX.  USED IN ESTIMATING THE ACCURACY OF THE */
/*     COMPUTED EIGENVALUES. */


/* INTERNAL VARIABLES USED */


/* J   THE CURRENT DIMENSION OF T.  (THE DIMENSION OF THE CURRENT */
/*    KRYLOV SUBSPACE. */

/* NGOOD   THE NUMBER OF GOOD RITZ VECTORS (GOOD VECTORS */
/*    LIE IN THE CURRENT KRYLOV SUBSPACE). */

/* NLEFT   THE NUMBER OF VALUES WHICH REMAIN TO BE DETERMINED, */
/*    I.E.  NLEFT = NVAL - NPERM. */

/* NUMBER = NPERM + NGOOD. */

/* ANORM   AN ESTIMATE OF THE NORM OF THE MATRIX. */

/* EPS   THE RELATIVE MACHINE PRECISION. */

/* UTOL   THE USER TOLERANCE. */

/* TARR  AN ARRAY OF LENGTH ONE USED TO INSURE TYPE CONSISTENCY IN CALLS TO */
/*       SLAEIG */

/* ZERO  AN ARRAY OF LENGTH ONE CONTAINING ZERO, USED TO INSURE TYPE CONSISTENCY */
/*       IN CALLS TO SCOPY */

    /* Parameter adjustments */
    p2_dim1 = *n;
    p2_offset = 1 + p2_dim1 * 1;
    p2 -= p2_offset;
    p0_dim1 = *n;
    p0_offset = 1 + p0_dim1 * 1;
    p0 -= p0_offset;
    p1_dim1 = *n;
    p1_offset = 1 + p1_dim1 * 1;
    p1 -= p1_offset;
    t_dim1 = *nband;
    t_offset = 1 + t_dim1 * 1;
    t -= t_offset;
    --val;
    vec_dim1 = *nmvec;
    vec_offset = 1 + vec_dim1 * 1;
    vec -= vec_offset;
    bet_dim1 = *nblock;
    bet_offset = 1 + bet_dim1 * 1;
    bet -= bet_offset;
    alp_dim1 = *nblock;
    alp_offset = 1 + alp_dim1 * 1;
    alp -= alp_offset;
    s_dim1 = *maxj;
    s_offset = 1 + s_dim1 * 1;
    s -= s_offset;
    --res;
    --tau;
    --otau;
    --bound;
    --atemp;
    --vtemp;
    --d__;
    --ind;

    /* Function Body */
    zero[0] = 0.f;
    rnorm = 0.f;
    if (*nperm != 0) {
/* Computing MAX */
        r__1 = -val[1], r__2 = val[*nperm];
        rnorm = dmax(r__1,r__2);
    }
    pnorm = rnorm;
    *delta = (float)1e31;
    epsrt = sqrt(*eps);
    nleft = *nval - *nperm;
    *nop = 0;
    number = *nperm;
    *raritz = FALSE_;
/* Computing MAX */
    d__1 = (doublereal) (-((real) (*nfig)));
    r__1 = (real) (*n) * *eps, r__2 = pow_dd(&c_b122, &d__1);
    utol = dmax(r__1,r__2);
    j = *maxj;

/* ------------------------------------------------------------------ */

/* ANY ITERATION OF THE ALGORITHM BEGINS HERE. */

L30:
    for (i__ = 1; i__ <= *nblock; ++i__) {
        temp = snrm2_(n, &p1[i__ * p1_dim1 + 1], &c__1);
        if (temp == 0.f) {
            slaran_(n, &p1[i__ * p1_dim1 + 1]);
        }
    }
    if (*nperm == 0) {
        goto L70;
    }
    for (i__ = 1; i__ <= *nperm; ++i__) {
        tau[i__] = 1.f;
        otau[i__] = 0.f;
    }
L70:
    i__1 = *n * *nblock;
    scopy_(&i__1, zero, &c__0, &p0[p0_offset], &c__1);
    i__1 = *nblock * *nblock;
    scopy_(&i__1, zero, &c__0, &bet[bet_offset], &c__1);
    i__1 = j * *nband;
    scopy_(&i__1, zero, &c__0, &t[t_offset], &c__1);
    mtemp = *nval + 1;
    for (i__ = 1; i__ <= mtemp; ++i__) {
        scopy_(&j, zero, &c__0, &s[i__ * s_dim1 + 1], &c__1);
    }
    ngood = 0;
    tmin = (float)1e30;
    tmax = (float)-1e30;
    test = TRUE_;
    enough = FALSE_;
    betmax = 0.f;
    j = 0;

/* ------------------------------------------------------------------ */

/* THIS SECTION TAKES A SINGLE BLOCK LANCZOS STEP. */

L80:
    j += *nblock;

/* THIS IS THE SELECTIVE ORTHOGONALIZATION. */

    if (number == 0) {
        goto L110;
    }
    for (i__ = 1; i__ <= number; ++i__) {
        if (tau[i__] < epsrt) {
            goto L100;
        }
        test = TRUE_;
        tau[i__] = 0.f;
        if (otau[i__] != 0.f) {
            otau[i__] = 1.f;
        }
        for (k = 1; k <= *nblock; ++k) {
            temp = -sdot_(n, &vec[i__ * vec_dim1 + 1], &c__1, &p1[k * p1_dim1
                    + 1], &c__1);
            saxpy_(n, &temp, &vec[i__ * vec_dim1 + 1], &c__1, &p1[k * p1_dim1
                    + 1], &c__1);

/* THIS CHECKS FOR TOO GREAT A LOSS OF ORTHOGONALITY BETWEEN A */
/* NEW LANCZOS VECTOR AND A GOOD RITZ VECTOR.  THE ALGORITHM IS */
/* TERMINATED IF TOO MUCH ORTHOGONALITY IS LOST. */

            if (dabs(temp * bet[k + k * bet_dim1]) > (*n) * epsrt * anorm && i__ > *nperm) {
                goto L380;
            }
        }
L100:
        ;
    }

/* IF NECESSARY, THIS REORTHONORMALIZES P1 AND UPDATES BET. */

L110:
    if (! test) {
        goto L160;
    }
    sortqr_(n, n, nblock, &p1[p1_offset], &alp[alp_offset]);
    test = FALSE_;
    if (j == *nblock) {
        goto L160;
    }
    for (i__ = 1; i__ <= *nblock; ++i__) {
        if (alp[i__ + i__ * alp_dim1] > 0.f) {
            goto L130;
        }
        m = j - (*nblock << 1) + i__;
        l = *nblock + 1;
        for (k = i__; k <= *nblock; ++k) {
            bet[i__ + k * bet_dim1] = -bet[i__ + k * bet_dim1];
            t[l + m * t_dim1] = -t[l + m * t_dim1];
            --l;
            ++m;
        }
L130:
        ;
    }

/* THIS IS THE LANCZOS STEP. */

L160:
    (*op)(n, nblock, &p1[p1_offset], &p2[p2_offset]);
    ++(*nop);
    (*iovect)(n, nblock, &p1[p1_offset], &j, &c__0);

/* THIS COMPUTES P2=P2-P0*BET(TRANSPOSE) */

    for (i__ = 1; i__ <= *nblock; ++i__) {
        for (k = i__; k <= *nblock; ++k) {
            r__1 = -bet[i__ + k * bet_dim1];
            saxpy_(n, &r__1, &p0[k * p0_dim1 + 1], &c__1, &p2[i__ * p2_dim1 +
                    1], &c__1);
        }
    }

/* THIS COMPUTES ALP AND P2=P2-P1*ALP. */

    for (i__ = 1; i__ <= *nblock; ++i__) {
        for (k = 1; k <= i__; ++k) {
            ii = i__ - k + 1;
            alp[ii + k * alp_dim1] = sdot_(n, &p1[i__ * p1_dim1 + 1], &c__1, &
                    p2[k * p2_dim1 + 1], &c__1);
            r__1 = -alp[ii + k * alp_dim1];
            saxpy_(n, &r__1, &p1[i__ * p1_dim1 + 1], &c__1, &p2[k * p2_dim1 +
                    1], &c__1);
            if (k != i__) {
                r__1 = -alp[ii + k * alp_dim1];
                saxpy_(n, &r__1, &p1[k * p1_dim1 + 1], &c__1, &p2[i__ *
                        p2_dim1 + 1], &c__1);
            }
        }
    }

/*  REORTHOGONALIZATION OF THE SECOND BLOCK */

    if (j != *nblock) {
        goto L220;
    }
    for (i__ = 1; i__ <= *nblock; ++i__) {
        for (k = 1; k <= i__; ++k) {
            temp = sdot_(n, &p1[i__ * p1_dim1 + 1], &c__1, &p2[k * p2_dim1 +
                    1], &c__1);
            r__1 = -temp;
            saxpy_(n, &r__1, &p1[i__ * p1_dim1 + 1], &c__1, &p2[k * p2_dim1 +
                    1], &c__1);
            if (k != i__) {
                r__1 = -temp;
                saxpy_(n, &r__1, &p1[k * p1_dim1 + 1], &c__1, &p2[i__ *
                        p2_dim1 + 1], &c__1);
            }
            ii = i__ - k + 1;
            alp[ii + k * alp_dim1] += temp;
        }
    }

/* THIS ORTHONORMALIZES THE NEXT BLOCK */

L220:
    sortqr_(n, n, nblock, &p2[p2_offset], &bet[bet_offset]);

/* THIS STORES ALP AND BET IN T. */

    for (i__ = 1; i__ <= *nblock; ++i__) {
        m = j - *nblock + i__;
        for (k = i__; k <= *nblock; ++k) {
            l = k - i__ + 1;
            t[l + m * t_dim1] = alp[l + i__ * alp_dim1];
        }
        for (k = 1; k <= i__; ++k) {
            l = *nblock - i__ + k + 1;
            t[l + m * t_dim1] = bet[k + i__ * bet_dim1];
        }
    }

/* THIS NEGATES T IF SMALL IS FALSE. */

    if (*small) {
        goto L280;
    }
    m = j - *nblock + 1;
    for (i__ = m; i__ <= j; ++i__) {
        for (k = 1; k <= l; ++k) {
            t[k + i__ * t_dim1] = -t[k + i__ * t_dim1];
        }
    }

/* THIS SHIFTS THE LANCZOS VECTORS */

L280:
    i__1 = *nblock * *n;
    scopy_(&i__1, &p1[p1_offset], &c__1, &p0[p0_offset], &c__1);
    i__1 = *nblock * *n;
    scopy_(&i__1, &p2[p2_offset], &c__1, &p1[p1_offset], &c__1);
    i__1 = j - *nblock + 1;
    slager_(&j, nband, &i__1, &t[t_offset], &tmin, &tmax);
/* Computing MAX */
    r__1 = max(rnorm,tmax), r__2 = -tmin;
    anorm = dmax(r__1,r__2);
    if (number == 0) {
        goto L305;
    }

/* THIS COMPUTES THE EXTREME EIGENVALUES OF ALP. */

    scopy_(nblock, zero, &c__0, &p2[p2_offset], &c__1);
    slaeig_(nblock, nblock, &c__1, &c__1, &alp[alp_offset], tarr, nblock, &p2[
            p2_offset], &bound[1], &atemp[1], &d__[1], &vtemp[1], eps, &tmin,
            &tmax);
    alpmin = tarr[0];
    scopy_(nblock, zero, &c__0, &p2[p2_offset], &c__1);
    slaeig_(nblock, nblock, nblock, nblock, &alp[alp_offset], tarr, nblock, &
            p2[p2_offset], &bound[1], &atemp[1], &d__[1], &vtemp[1], eps, &
            tmin, &tmax);
    alpmax = tarr[0];

/* THIS COMPUTES ALP = BET(TRANSPOSE)*BET. */

L305:
    for (i__ = 1; i__ <= *nblock; ++i__) {
        for (k = 1; k <= i__; ++k) {
            l = i__ - k + 1;
            i__1 = *nblock - i__ + 1;
            alp[l + k * alp_dim1] = sdot_(&i__1, &bet[i__ + i__ * bet_dim1],
                    nblock, &bet[k + i__ * bet_dim1], nblock);
        }
    }
    if (number == 0) {
        goto L330;
    }

/* THIS COMPUTES THE SMALLEST SINGULAR VALUE OF BET. */

    scopy_(nblock, zero, &c__0, &p2[p2_offset], &c__1);
    r__1 = anorm * anorm;
    slaeig_(nblock, nblock, &c__1, &c__1, &alp[alp_offset], tarr, nblock, &p2[
            p2_offset], &bound[1], &atemp[1], &d__[1], &vtemp[1], eps, &
            c_b195, &r__1);
    betmin = sqrt(tarr[0]);

/* THIS UPDATES TAU AND OTAU. */

    for (i__ = 1; i__ <= number; ++i__) {
/* Computing MAX */
        r__1 = alpmax - val[i__], r__2 = val[i__] - alpmin;
        temp = (tau[i__] * dmax(r__1,r__2) + otau[i__] * betmax + *eps *
                anorm) / betmin;
        if (i__ <= *nperm) {
            temp += res[i__] / betmin;
        }
        otau[i__] = tau[i__];
        tau[i__] = temp;
    }

/* THIS COMPUTES THE LARGEST SINGULAR VALUE OF BET. */

L330:
    scopy_(nblock, zero, &c__0, &p2[p2_offset], &c__1);
    r__1 = anorm * anorm;
    slaeig_(nblock, nblock, nblock, nblock, &alp[alp_offset], tarr, nblock, &
            p2[p2_offset], &bound[1], &atemp[1], &d__[1], &vtemp[1], eps, &
            c_b195, &r__1);
    betmax = sqrt(tarr[0]);
    if (j <= *nblock << 1) {
        goto L80;
    }

/* ------------------------------------------------------------------ */

/* THIS SECTION COMPUTES AND EXAMINES THE SMALLEST NONGOOD AND */
/* LARGEST DESIRED EIGENVALUES OF T TO SEE IF A CLOSER LOOK */
/* IS JUSTIFIED. */

    tolg = epsrt * anorm;
    tola = utol * rnorm;
    if (*maxj - j < *nblock || *nop >= *maxop && nleft != 0) {
        goto L390;
    }
    goto L400;

/* ------------------------------------------------------------------ */

/* THIS SECTION COMPUTES SOME EIGENVALUES AND EIGENVECTORS OF T TO */
/* SEE IF FURTHER ACTION IS INDICATED, ENTRY IS AT 380 OR 390 IF AN */
/* ITERATION (OR TERMINATION) IS KNOWN TO BE NEEDED, OTHERWISE ENTRY */
/* IS AT 400. */

L380:
    j -= *nblock;
    *ierr = -8;
L390:
    if (nleft == 0) {
        return;
    }
    test = TRUE_;
L400:
/* Computing MIN */
    ntheta = min(j/2,nleft+1);
    slaeig_(&j, nband, &c__1, &ntheta, &t[t_offset], &val[number + 1], maxj, &
            s[s_offset], &bound[1], &atemp[1], &d__[1], &vtemp[1], eps, &tmin,
             &tmax);
    smvpc_(nblock, &bet[bet_offset], maxj, &j, &s[s_offset], &ntheta, &atemp[
            1], &vtemp[1], &d__[1]);

/* THIS CHECKS FOR TERMINATION OF A CHECK RUN */

    if (nleft != 0 || j < *nblock * 6) {
        goto L410;
    }
    if (val[number + 1] - atemp[1] > val[*nperm] - tola) {
        goto L790;
    }

/* THIS UPDATES NLEFT BY EXAMINING THE COMPUTED EIGENVALUES OF T */
/* TO DETERMINE IF SOME PERMANENT VALUES ARE NO LONGER DESIRED. */

L410:
    if (ntheta <= nleft) {
        goto L470;
    }
    if (*nperm == 0) {
        goto L430;
    }
    m = number + nleft + 1;
    if (val[m] >= val[*nperm]) {
        goto L430;
    }
    --(*nperm);
    ngood = 0;
    number = *nperm;
    ++nleft;
    goto L400;

/* THIS UPDATES DELTA. */

L430:
    m = number + nleft + 1;
/* Computing MIN */
    r__1 = *delta, r__2 = val[m];
    *delta = dmin(r__1,r__2);
    enough = TRUE_;
    if (nleft == 0) {
        goto L80;
    }
    ntheta = nleft;
    vtemp[ntheta + 1] = 1.f;

/* ------------------------------------------------------------------ */

/* THIS SECTION EXAMINES THE COMPUTED EIGENPAIRS IN DETAIL. */

/* THIS CHECKS FOR ENOUGH ACCEPTABLE VALUES. */

    if (! (test || enough)) {
        goto L470;
    }
    *delta = dmin(*delta,anorm);
/* Computing MAX */
/* Computing MAX */
    r__3 = -val[number + 1];
    r__1 = rnorm, r__2 = dmax(r__3,*delta);
    pnorm = dmax(r__1,r__2);
    tola = utol * pnorm;
    nstart = 0;
    for (i__ = 1; i__ <= ntheta; ++i__) {
        m = number + i__;
/* Computing MIN */
        r__1 = atemp[i__] * atemp[i__] / (*delta - val[m]), r__2 = atemp[i__];
        if (dmin(r__1,r__2) > tola) {
            goto L450;
        }
        ind[i__] = -1;
        goto L460;

L450:
        enough = FALSE_;
        if (! test) {
            goto L470;
        }
        ind[i__] = 1;
        ++nstart;
L460:
        ;
    }

/*  COPY VALUES OF IND INTO VTEMP */

    for (i__ = 1; i__ <= ntheta; ++i__) {
        vtemp[i__] = (real) ind[i__];
    }
    goto L500;

/* THIS CHECKS FOR NEW GOOD VECTORS. */

L470:
    ng = 0;
    for (i__ = 1; i__ <= ntheta; ++i__) {
        if (vtemp[i__] > tolg) {
            goto L480;
        }
        ++ng;
        vtemp[i__] = -1.f;
        goto L490;

L480:
        vtemp[i__] = 1.f;
L490:
        ;
    }

    if (ng <= ngood) {
        goto L80;
    }
    nstart = ntheta - ng;

/* ------------------------------------------------------------------ */

/* THIS SECTION COMPUTES AND NORMALIZES THE INDICATED RITZ VECTORS. */
/* IF NEEDED (TEST = .TRUE.), NEW STARTING VECTORS ARE COMPUTED. */

L500:
    test = test && ! enough;
    ngood = ntheta - nstart;
    ++nstart;
    ++ntheta;

/* THIS ALIGNS THE DESIRED (ACCEPTABLE OR GOOD) EIGENVALUES AND */
/* EIGENVECTORS OF T.  THE OTHER EIGENVECTORS ARE SAVED FOR */
/* FORMING STARTING VECTORS, IF NECESSARY.  IT ALSO SHIFTS THE */
/* EIGENVALUES TO OVERWRITE THE GOOD VALUES FROM THE PREVIOUS */
/* PAUSE. */

    scopy_(&ntheta, &val[number + 1], &c__1, &val[*nperm + 1], &c__1);
    if (nstart == 0) {
        goto L580;
    }
    if (nstart == ntheta) {
        goto L530;
    }
    svsort_(&ntheta, &vtemp[1], &atemp[1], &c__1, &val[*nperm + 1], maxj, &j,
            &s[s_offset]);

/* THES ACCUMULATES THE J-VECTORS USED TO FORM THE STARTING */
/* VECTORS. */

L530:
    if (! test) {
        nstart = 0;
    }
    if (! test) {
        goto L580;
    }

/*  FIND MINIMUM ATEMP VALUE TO AVOID POSSIBLE OVERFLOW */

    temp = atemp[1];
    for (i__ = 1; i__ <= nstart; ++i__) {
/* Computing MIN */
        r__1 = temp, r__2 = atemp[i__];
        temp = dmin(r__1,r__2);
    }
    m = ngood + 1;
    l = ngood + min(nstart,*nblock);
    for (i__ = m; i__ <= l; ++i__) {
        r__1 = temp / atemp[i__];
        sscal_(&j, &r__1, &s[i__ * s_dim1 + 1], &c__1);
    }
    m = (nstart - 1) / *nblock;
    if (m == 0) {
        goto L570;
    }
    l = ngood + *nblock;
    for (i__ = 1; i__ <= m; ++i__) {
        for (k = 1; k <= *nblock; ++k) {
            ++l;
            if (l > ntheta) {
                goto L570;
            }
            i1 = ngood + k;
            r__1 = temp / atemp[l];
            saxpy_(&j, &r__1, &s[l * s_dim1 + 1], &c__1, &s[i1 * s_dim1 + 1],
                    &c__1);
        }
    }
L570:
    nstart = min(nstart,*nblock);

/* THIS STORES THE RESIDUAL NORMS OF THE NEW PERMANENT VECTORS. */

L580:
    if (ngood == 0 || ! (test || enough)) {
        goto L600;
    }
    for (i__ = 1; i__ <= ngood; ++i__) {
        m = *nperm + i__;
        res[m] = atemp[i__];
    }

/* THIS COMPUTES THE RITZ VECTORS BY SEQUENTIALLY RECALLING THE */
/* LANCZOS VECTORS. */

L600:
    number = *nperm + ngood;
    if (test || enough) {
        i__1 = *n * *nblock;
        scopy_(&i__1, zero, &c__0, &p1[p1_offset], &c__1);
    }
    if (ngood == 0) {
        goto L620;
    }
    m = *nperm + 1;
    for (i__ = m; i__ <= number; ++i__) {
        scopy_(n, zero, &c__0, &vec[i__ * vec_dim1 + 1], &c__1);
    }
L620:
    for (i__ = *nblock; *nblock < 0 ? i__ >= j : i__ <= j; i__ += *nblock) {
        (*iovect)(n, nblock, &p2[p2_offset], &i__, &c__1);
        for (k = 1; k <= *nblock; ++k) {
            m = i__ - *nblock + k;
            if (nstart == 0) {
                goto L640;
            }
            for (l = 1; l <= nstart; ++l) {
                i1 = ngood + l;
                saxpy_(n, &s[m + i1 * s_dim1], &p2[k * p2_dim1 + 1], &c__1, &
                        p1[l * p1_dim1 + 1], &c__1);
            }
L640:
            if (ngood == 0) {
                goto L660;
            }
            for (l = 1; l <= ngood; ++l) {
                i1 = l + *nperm;
                saxpy_(n, &s[m + l * s_dim1], &p2[k * p2_dim1 + 1], &c__1, &
                        vec[i1 * vec_dim1 + 1], &c__1);
            }
L660:
            ;
        }
    }
    if (test || enough) {
        goto L690;
    }

/* THIS NORMALIZES THE RITZ VECTORS AND INITIALIZES THE */
/* TAU RECURRENCE. */

    m = *nperm + 1;
    for (i__ = m; i__ <= number; ++i__) {
        temp = 1.f / snrm2_(n, &vec[i__ * vec_dim1 + 1], &c__1);
        sscal_(n, &temp, &vec[i__ * vec_dim1 + 1], &c__1);
        tau[i__] = 1.f;
        otau[i__] = 1.f;
    }

/*  SHIFT S VECTORS TO ALIGN FOR LATER CALL TO SLAEIG */

    scopy_(&ntheta, &val[*nperm + 1], &c__1, &vtemp[1], &c__1);
    svsort_(&ntheta, &vtemp[1], &atemp[1], &c__0, tarr, maxj, &j, &s[s_offset]
            );
    goto L80;

/* ------------------------------------------------------------------ */

/* THIS SECTION PREPARES TO ITERATE THE ALGORITHM BY SORTING THE */
/* PERMANENT VALUES, RESETTING SOME PARAMETERS, AND ORTHONORMALIZING */
/* THE PERMANENT VECTORS. */

L690:
    if (ngood == 0 && *nop >= *maxop) {
        goto L810;
    }
    if (ngood == 0) {
        goto L30;
    }

/* THIS ORTHONORMALIZES THE VECTORS */

    i__1 = *nperm + ngood;
    sortqr_(nmvec, n, &i__1, &vec[vec_offset], &s[s_offset]);

/* THIS SORTS THE VALUES AND VECTORS. */

    if (*nperm != 0) {
        i__1 = *nperm + ngood;
        svsort_(&i__1, &val[1], &res[1], &c__0, &temp, nmvec, n, &vec[vec_offset]);
    }
    *nperm += ngood;
    nleft -= ngood;
/* Computing MAX */
    r__1 = -val[1], r__2 = val[*nperm];
    rnorm = dmax(r__1,r__2);

/* THIS DECIDES WHERE TO GO NEXT. */

    if (*nop >= *maxop && nleft != 0) {
        goto L810;
    }
    if (nleft != 0) {
        goto L30;
    }
    if (val[*nval] - val[1] < tola) {
        goto L790;
    }

/* THIS DOES A CLUSTER TEST TO SEE IF A CHECK RUN IS NEEDED */
/* TO LOOK FOR UNDISCLOSED MULTIPLICITIES. */

    m = *nperm - *nblock + 1;
    if (m <= 0) {
        return;
    }
    for (i__ = 1; i__ <= m; ++i__) {
        l = i__ + *nblock - 1;
        if (val[l] - val[i__] < tola) {
            goto L30;
        }
    }

/* THIS DOES A CLUSTER TEST TO SEE IF A FINAL RAYLEIGH-RITZ */
/* PROCEDURE IS NEEDED. */

L790:
    m = *nperm - *nblock;
    if (m <= 0) {
        return;
    }
    for (i__ = 1; i__ <= m; ++i__) {
        l = i__ + *nblock;
        if (val[l] - val[i__] >= tola) {
            goto L800;
        }
        *raritz = TRUE_;
        return;
L800:
        ;
    }
    return;

/* ------------------------------------------------------------------ */

/* THIS REPORTS THAT MAXOP WAS EXCEEDED. */

L810:
    *ierr = -2;
    goto L790;

} /* snwla_ */


/* ------------------------------------------------------------------ */

/* Subroutine */ void snppla_(op, iovect, n, nperm, nop, nmval, val, nmvec,
        vec, nblock, h__, hv, p, q, bound, d__, delta, small, raritz, eps)
/* Subroutine */ void (*op) (), (*iovect) ();
integer *n, *nperm, *nop, *nmval;
real *val;
integer *nmvec;
real *vec;
integer *nblock;
real *h__, *hv, *p, *q, *bound, *d__, *delta;
logical *small, *raritz;
real *eps;
{
    /* System generated locals */
    integer val_dim1, val_offset, vec_dim1, vec_offset, h_dim1, h_offset,
            hv_dim1, hv_offset, p_dim1, p_offset, q_dim1, q_offset, i__1;
    real r__1;

    /* Local variables */
    static real hmin, hmax, temp;
    extern doublereal sdot_();
    static real zero[1];
    extern doublereal snrm2_();
    static integer i__, j, k, l, m;
    extern /* Subroutine */ void scopy_(), saxpy_();
    static integer jj, kk;
    extern /* Subroutine */ void slaeig_(), slager_();

/* THIS SUBROUTINE POST PROCESSES THE EIGENVECTORS.  BLOCK MATRIX */
/* VECTOR PRODUCTS ARE USED TO MINIMIZED THE NUMBER OF CALLS TO OP. */

/* IF RARITZ IS .TRUE.  A FINAL RAYLEIGH-RITZ PROCEDURE IS APPLIED */
/* TO THE EIGENVECTORS. */

    /* Parameter adjustments */
    q_dim1 = *n;
    q_offset = 1 + q_dim1 * 1;
    q -= q_offset;
    p_dim1 = *n;
    p_offset = 1 + p_dim1 * 1;
    p -= p_offset;
    hv_dim1 = *nperm;
    hv_offset = 1 + hv_dim1 * 1;
    hv -= hv_offset;
    h_dim1 = *nperm;
    h_offset = 1 + h_dim1 * 1;
    h__ -= h_offset;
    val_dim1 = *nmval;
    val_offset = 1 + val_dim1 * 1;
    val -= val_offset;
    vec_dim1 = *nmvec;
    vec_offset = 1 + vec_dim1 * 1;
    vec -= vec_offset;
    --bound;
    --d__;

    /* Function Body */
    zero[0] = 0.f;
    if (! (*raritz)) {
        goto L190;
    }

/* ------------------------------------------------------------------ */

/* THIS CONSTRUCTS H=Q*AQ, WHERE THE COLUMNS OF Q ARE THE */
/* APPROXIMATE EIGENVECTORS.  TEMP = -1 IS USED WHEN SMALL IS */
/* FALSE TO AVOID HAVING TO RESORT THE EIGENVALUES AND EIGENVECTORS */
/* COMPUTED BY SLAEIG. */

    i__1 = *nperm * *nperm;
    scopy_(&i__1, zero, &c__0, &h__[h_offset], &c__1);
    temp = -1.f;
    if (*small) {
        temp = 1.f;
    }
    m = *nperm % *nblock;
    if (m == 0) {
        goto L40;
    }
    for (i__ = 1; i__ <= m; ++i__) {
        scopy_(n, &vec[i__ * vec_dim1 + 1], &c__1, &p[i__ * p_dim1 + 1], &
                c__1);
    }
    (*iovect)(n, &m, &p[p_offset], &m, &c__0);
    (*op)(n, &m, &p[p_offset], &q[q_offset]);
    ++(*nop);
    for (i__ = 1; i__ <= m; ++i__) {
        for (j = i__; j <= *nperm; ++j) {
            jj = j - i__ + 1;
            h__[jj + i__ * h_dim1] = temp * sdot_(n, &vec[j * vec_dim1 + 1], &
                    c__1, &q[i__ * q_dim1 + 1], &c__1);
        }
    }
    if (*nperm < *nblock) {
        goto L90;
    }
L40:
    m += *nblock;
    for (i__ = m; *nblock < 0 ? i__ >= *nperm : i__ <= *nperm; i__ += *nblock) {
        for (j = 1; j <= *nblock; ++j) {
            l = i__ - *nblock + j;
            scopy_(n, &vec[l * vec_dim1 + 1], &c__1, &p[j * p_dim1 + 1], &
                    c__1);
        }
        (*iovect)(n, nblock, &p[p_offset], &i__, &c__0);
        (*op)(n, nblock, &p[p_offset], &q[q_offset]);
        ++(*nop);
        for (j = 1; j <= *nblock; ++j) {
            l = i__ - *nblock + j;
            for (k = l; k <= *nperm; ++k) {
                kk = k - l + 1;
                h__[kk + l * h_dim1] = temp * sdot_(n, &vec[k * vec_dim1 + 1],
                         &c__1, &q[j * q_dim1 + 1], &c__1);
            }
        }
    }

/* THIS COMPUTES THE SPECTRAL DECOMPOSITION OF H. */

L90:
    hmin = h__[h_dim1 + 1];
    hmax = h__[h_dim1 + 1];
    slager_(nperm, nperm, &c__1, &h__[h_offset], &hmin, &hmax);
    slaeig_(nperm, nperm, &c__1, nperm, &h__[h_offset], &val[val_offset],
            nperm, &hv[hv_offset], &bound[1], &p[p_offset], &d__[1], &q[
            q_offset], eps, &hmin, &hmax);

/* THIS COMPUTES THE RITZ VECTORS--THE COLUMNS OF */
/* Y = QS WHERE S IS THE MATRIX OF EIGENVECTORS OF H. */

    for (i__ = 1; i__ <= *nperm; ++i__) {
        scopy_(n, zero, &c__0, &vec[i__ * vec_dim1 + 1], &c__1);
    }
    m = *nperm % *nblock;
    if (m == 0) {
        goto L150;
    }
    (*iovect)(n, &m, &p[p_offset], &m, &c__1);
    for (i__ = 1; i__ <= m; ++i__) {
        for (j = 1; j <= *nperm; ++j) {
            saxpy_(n, &hv[i__ + j * hv_dim1], &p[i__ * p_dim1 + 1], &c__1, &
                    vec[j * vec_dim1 + 1], &c__1);
        }
    }
    if (*nperm < *nblock) {
        goto L190;
    }
L150:
    m += *nblock;
    for (i__ = m; *nblock < 0 ? i__ >= *nperm : i__ <= *nperm; i__ += *nblock) {
        (*iovect)(n, nblock, &p[p_offset], &i__, &c__1);
        for (j = 1; j <= *nblock; ++j) {
            l = i__ - *nblock + j;
            for (k = 1; k <= *nperm; ++k) {
                saxpy_(n, &hv[l + k * hv_dim1], &p[j * p_dim1 + 1], &c__1, &
                        vec[k * vec_dim1 + 1], &c__1);
            }
        }
    }

/* ------------------------------------------------------------------ */

/* THIS SECTION COMPUTES THE RAYLEIGH QUOTIENTS (IN VAL(*,1)) */
/* AND RESIDUAL NORMS (IN VAL(*,2)) OF THE EIGENVECTORS. */

L190:
    if (! (*small)) {
        *delta = -(*delta);
    }
    m = *nperm % *nblock;
    if (m == 0) {
        goto L220;
    }
    for (i__ = 1; i__ <= m; ++i__) {
        scopy_(n, &vec[i__ * vec_dim1 + 1], &c__1, &p[i__ * p_dim1 + 1], &c__1);
    }
    (*op)(n, &m, &p[p_offset], &q[q_offset]);
    ++(*nop);
    for (i__ = 1; i__ <= m; ++i__) {
        val[i__ + val_dim1] = sdot_(n, &p[i__ * p_dim1 + 1], &c__1, &q[i__ * q_dim1 + 1], &c__1);
        r__1 = -val[i__ + val_dim1];
        saxpy_(n, &r__1, &p[i__ * p_dim1 + 1], &c__1, &q[i__ * q_dim1 + 1], &c__1);
        val[i__ + (val_dim1 << 1)] = snrm2_(n, &q[i__ * q_dim1 + 1], &c__1);
    }
    if (*nperm < *nblock) {
        goto L260;
    }
L220:
    ++m;
    for (i__ = m; *nblock < 0 ? i__ >= *nperm : i__ <= *nperm; i__ += *nblock) {
        for (j = 1; j <= *nblock; ++j) {
            l = i__ - 1 + j;
            scopy_(n, &vec[l * vec_dim1 + 1], &c__1, &p[j * p_dim1 + 1], &
                    c__1);
        }
        (*op)(n, nblock, &p[p_offset], &q[q_offset]);
        ++(*nop);
        for (j = 1; j <= *nblock; ++j) {
            l = i__ - 1 + j;
            val[l + val_dim1] = sdot_(n, &p[j * p_dim1 + 1], &c__1, &q[j *
                    q_dim1 + 1], &c__1);
            r__1 = -val[l + val_dim1];
            saxpy_(n, &r__1, &p[j * p_dim1 + 1], &c__1, &q[j * q_dim1 + 1], &
                    c__1);
            val[l + (val_dim1 << 1)] = snrm2_(n, &q[j * q_dim1 + 1], &c__1);
        }
    }

/* THIS COMPUTES THE ACCURACY ESTIMATES.  FOR CONSISTENCY WITH SILASO */
/* A DO LOOP IS NOT USED. */

L260:
    i__ = 0;
L270:
    ++i__;
    if (i__ > *nperm) {
        return;
    }
    temp = *delta - val[i__ + val_dim1];
    if (! (*small)) {
        temp = -temp;
    }
    val[i__ + (val_dim1 << 2)] = 0.f;
    if (temp > 0.f) {
        val[i__ + (val_dim1 << 2)] = val[i__ + (val_dim1 << 1)] / temp;
    }
    val[i__ + val_dim1 * 3] = val[i__ + (val_dim1 << 2)] * val[i__ + (
            val_dim1 << 1)];
    goto L270;

} /* snppla_ */


/* ------------------------------------------------------------------ */

/* Subroutine */ void sortqr_(nz, n, nblock, z__, b)
integer *nz, *n, *nblock;
real *z__, *b;
{
    /* System generated locals */
    integer z_dim1, z_offset, b_dim1, b_offset;
    real r__1;

    /* Builtin functions */
    double r_sign();

    /* Local variables */
    static real temp;
    extern doublereal sdot_(), snrm2_();
    static integer i__, j, k, m;
    static real sigma;
    extern /* Subroutine */ void sscal_(), saxpy_();
    static integer length;
    static real tau;

/* THIS SUBROUTINE COMPUTES THE QR FACTORIZATION OF THE N X NBLOCK */
/* MATRIX Z.  Q IS FORMED IN PLACE AND RETURNED IN Z.  R IS */
/* RETURNED IN B. */

/* THIS SECTION REDUCES Z TO TRIANGULAR FORM. */

    /* Parameter adjustments */
    z_dim1 = *nz;
    z_offset = 1 + z_dim1 * 1;
    z__ -= z_offset;
    b_dim1 = *nblock;
    b_offset = 1 + b_dim1 * 1;
    b -= b_offset;

    /* Function Body */
    for (i__ = 1; i__ <= *nblock; ++i__) {

/* THIS FORMS THE ITH REFLECTION. */

        length = *n - i__ + 1;
        r__1 = snrm2_(&length, &z__[i__ + i__ * z_dim1], &c__1);
        sigma = r_sign(&r__1, &z__[i__ + i__ * z_dim1]);
        b[i__ + i__ * b_dim1] = -sigma;
        z__[i__ + i__ * z_dim1] += sigma;
        tau = sigma * z__[i__ + i__ * z_dim1];
        if (i__ == *nblock) {
            goto L30;
        }
        j = i__ + 1;

/* THIS APPLIES THE ROTATION TO THE REST OF THE COLUMNS. */

        for (k = j; k <= *nblock; ++k) {
            if (tau == 0.f) {
                goto L10;
            }
            temp = -sdot_(&length, &z__[i__ + i__ * z_dim1], &c__1, &z__[i__
                    + k * z_dim1], &c__1) / tau;
            saxpy_(&length, &temp, &z__[i__ + i__ * z_dim1], &c__1, &z__[i__
                    + k * z_dim1], &c__1);
L10:
            b[i__ + k * b_dim1] = z__[i__ + k * z_dim1];
            z__[i__ + k * z_dim1] = 0.f;
        }
L30:
        ;
    }

/* THIS ACCUMULATES THE REFLECTIONS IN REVERSE ORDER. */

    for (m = 1; m <= *nblock; ++m) {

/* THIS RECREATES THE ITH = NBLOCK-M+1)TH REFLECTION. */

        i__ = *nblock + 1 - m;
        sigma = -b[i__ + i__ * b_dim1];
        tau = z__[i__ + i__ * z_dim1] * sigma;
        if (tau == 0.f) {
            goto L60;
        }
        length = *n - *nblock + m;
        if (i__ == *nblock) {
            goto L50;
        }
        j = i__ + 1;

/* THIS APPLIES IT TO THE LATER COLUMNS. */

        for (k = j; k <= *nblock; ++k) {
            temp = -sdot_(&length, &z__[i__ + i__ * z_dim1], &c__1, &z__[i__
                    + k * z_dim1], &c__1) / tau;
            saxpy_(&length, &temp, &z__[i__ + i__ * z_dim1], &c__1, &z__[i__
                    + k * z_dim1], &c__1);
        }
L50:
        r__1 = -1.f / sigma;
        sscal_(&length, &r__1, &z__[i__ + i__ * z_dim1], &c__1);
L60:
        z__[i__ + i__ * z_dim1] += 1.f;
    }
    return;
} /* sortqr_ */


/* ------------------------------------------------------------------- */

/* Subroutine */ void svsort_(num, val, res, iflag, v, nmvec, n, vec)
integer *num;
real *val, *res;
integer *iflag;
real *v;
integer *nmvec, *n;
real *vec;
{
    /* System generated locals */
    integer vec_dim1, vec_offset;

    /* Local variables */
    static real temp;
    static integer i__, k, m;
    extern /* Subroutine */ void sswap_();

/*  THIS SUBROUTINE SORTS THE EIGENVALUES (VAL) IN ASCENDING ORDER */
/*  WHILE CONCURRENTLY SWAPPING THE RESIDUALS AND VECTORS. */
    /* Parameter adjustments */
    --val;
    --res;
    --v;
    vec_dim1 = *nmvec;
    vec_offset = 1 + vec_dim1 * 1;
    vec -= vec_offset;

    /* Function Body */
    if (*num <= 1) {
        return;
    }
    for (i__ = 2; i__ <= *num; ++i__) {
        m = *num - i__ + 1;
        for (k = 1; k <= m; ++k) {
            if (val[k] <= val[k + 1]) {
                goto L10;
            }
            temp = val[k];
            val[k] = val[k + 1];
            val[k + 1] = temp;
            temp = res[k];
            res[k] = res[k + 1];
            res[k + 1] = temp;
            sswap_(n, &vec[k * vec_dim1 + 1], &c__1, &vec[(k + 1) * vec_dim1
                    + 1], &c__1);
            if (*iflag == 0) {
                goto L10;
            }
            temp = v[k];
            v[k] = v[k + 1];
            v[k + 1] = temp;
L10:
            ;
        }
    }
} /* svsort_ */

