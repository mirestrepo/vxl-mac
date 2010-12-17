/* Include prototype headers. */
#include "blas/caxpy.h"
#include "blas/ccopy.h"
#include "blas/cdotc.h"
#include "blas/cscal.h"
#include "blas/csrot.h"
#include "blas/cswap.h"
#include "blas/dasum.h"
#include "blas/daxpy.h"
#include "blas/dcabs1.h"
#include "blas/dcopy.h"
#include "blas/ddot.h"
#include "blas/dgemm.h"
#include "blas/dgemv.h"
#include "blas/dger.h"
#include "blas/dlamch.h"
#include "blas/dnrm2.h"
#include "blas/drot.h"
#include "blas/drotg.h"
#include "blas/dscal.h"
#include "blas/dswap.h"
#include "blas/dtrmm.h"
#include "blas/dtrmv.h"
#include "blas/dtrsv.h"
#include "blas/dzasum.h"
#include "blas/dznrm2.h"
#include "blas/idamax.h"
#include "blas/isamax.h"
#include "blas/izamax.h"
#include "blas/saxpy.h"
#include "blas/scnrm2.h"
#include "blas/scopy.h"
#include "blas/sdot.h"
#include "blas/sgemv.h"
#include "blas/sger.h"
#include "blas/slamch.h"
#include "blas/snrm2.h"
#include "blas/srot.h"
#include "blas/srotg.h"
#include "blas/sscal.h"
#include "blas/sswap.h"
#include "blas/xerbla.h"
#include "blas/zaxpy.h"
#include "blas/zcopy.h"
#include "blas/zdotc.h"
#include "blas/zdotu.h"
#include "blas/zdrot.h"
#include "blas/zdscal.h"
#include "blas/zgemm.h"
#include "blas/zgemv.h"
#include "blas/zgerc.h"
#include "blas/zscal.h"
#include "blas/zswap.h"
#include "blas/ztrmm.h"
#include "blas/ztrmv.h"
#include "blas/ztrsv.h"
#include "linpack/cqrdc.h"
#include "linpack/cqrsl.h"
#include "linpack/csvdc.h"
#include "linpack/dpoco.h"
#include "linpack/dpodi.h"
#include "linpack/dpofa.h"
#include "linpack/dposl.h"
#include "linpack/dqrdc.h"
#include "linpack/dqrsl.h"
#include "linpack/dsvdc.h"
#include "linpack/sqrdc.h"
#include "linpack/sqrsl.h"
#include "linpack/ssvdc.h"
#include "linpack/zqrdc.h"
#include "linpack/zqrsl.h"
#include "linpack/zsvdc.h"
#include "temperton/dgpfa.h"
#include "temperton/dgpfa2f.h"
#include "temperton/dgpfa3f.h"
#include "temperton/dgpfa5f.h"
#include "temperton/gpfa.h"
#include "temperton/gpfa2f.h"
#include "temperton/gpfa3f.h"
#include "temperton/gpfa5f.h"
#include "temperton/setdgpfa.h"
#include "temperton/setgpfa.h"
#include "eispack/balanc.h"
#include "eispack/balbak.h"
#include "eispack/cdiv.h"
#include "eispack/elmhes.h"
#include "eispack/eltran.h"
#include "eispack/epslon.h"
#include "eispack/hqr.h"
#include "eispack/hqr2.h"
#include "eispack/otqlrat.h"
#include "eispack/pythag.h"
#include "eispack/rebak.h"
#include "eispack/reduc.h"
#include "eispack/rg.h"
#include "eispack/rs.h"
#include "eispack/rsg.h"
#include "eispack/tql1.h"
#include "eispack/tql2.h"
#include "eispack/tred1.h"
#include "eispack/tred2.h"
#include "laso/dlabax.h"
#include "laso/dlabcm.h"
#include "laso/dlabfc.h"
#include "laso/dlaeig.h"
#include "laso/dlager.h"
#include "laso/dlaran.h"
#include "laso/dmvpc.h"
#include "laso/dnlaso.h"
#include "laso/dnppla.h"
#include "laso/dortqr.h"
#include "laso/dvsort.h"
#include "laso/urand.h"
#include "arpack/dgetv0.h"
#include "arpack/dsaitr.h"
#include "arpack/dsapps.h"
#include "arpack/dsaup2.h"
#include "arpack/dsaupd.h"
#include "arpack/dsconv.h"
#include "arpack/dseigt.h"
#include "arpack/dsesrt.h"
#include "arpack/dseupd.h"
#include "arpack/dsgets.h"
#include "arpack/dsortr.h"
#include "arpack/dstats.h"
#include "arpack/dstqrb.h"
#include "arpack/second.h"
#include "lapack/complex16/zgebak.h"
#include "lapack/complex16/zgebal.h"
#include "lapack/complex16/zgeev.h"
#include "lapack/complex16/zgehd2.h"
#include "lapack/complex16/zgehrd.h"
#include "lapack/complex16/zhseqr.h"
#include "lapack/complex16/zlacgv.h"
#include "lapack/complex16/zlacpy.h"
#include "lapack/complex16/zladiv.h"
#include "lapack/complex16/zlahqr.h"
#include "lapack/complex16/zlahrd.h"
#include "lapack/complex16/zlange.h"
#include "lapack/complex16/zlanhs.h"
#include "lapack/complex16/zlarf.h"
#include "lapack/complex16/zlarfb.h"
#include "lapack/complex16/zlarfg.h"
#include "lapack/complex16/zlarft.h"
#include "lapack/complex16/zlarfx.h"
#include "lapack/complex16/zlascl.h"
#include "lapack/complex16/zlaset.h"
#include "lapack/complex16/zlassq.h"
#include "lapack/complex16/zlatrs.h"
#include "lapack/complex16/ztrevc.h"
#include "lapack/complex16/zung2r.h"
#include "lapack/complex16/zunghr.h"
#include "lapack/complex16/zungqr.h"
#include "lapack/double/dgecon.h"
#include "lapack/double/dgeqr2.h"
#include "lapack/double/dgeqrf.h"
#include "lapack/double/dgerq2.h"
#include "lapack/double/dgesc2.h"
#include "lapack/double/dgetc2.h"
#include "lapack/double/dggbak.h"
#include "lapack/double/dggbal.h"
#include "lapack/double/dgges.h"
#include "lapack/double/dgghrd.h"
#include "lapack/double/dhgeqz.h"
#include "lapack/double/dlabad.h"
#include "lapack/double/dlacon.h"
#include "lapack/double/dlacpy.h"
#include "lapack/double/dladiv.h"
#include "lapack/double/dlae2.h"
#include "lapack/double/dlaev2.h"
#include "lapack/double/dlag2.h"
#include "lapack/double/dlagv2.h"
#include "lapack/double/dlange.h"
#include "lapack/double/dlanhs.h"
#include "lapack/double/dlanst.h"
#include "lapack/double/dlapy2.h"
#include "lapack/double/dlapy3.h"
#include "lapack/double/dlarf.h"
#include "lapack/double/dlarfb.h"
#include "lapack/double/dlarfg.h"
#include "lapack/double/dlarft.h"
#include "lapack/double/dlarnv.h"
#include "lapack/double/dlartg.h"
#include "lapack/double/dlaruv.h"
#include "lapack/double/dlascl.h"
#include "lapack/double/dlaset.h"
#include "lapack/double/dlasr.h"
#include "lapack/double/dlasrt.h"
#include "lapack/double/dlassq.h"
#include "lapack/double/dlasv2.h"
#include "lapack/double/dlaswp.h"
#include "lapack/double/dlatdf.h"
#include "lapack/double/dlatrs.h"
#include "lapack/double/dorg2r.h"
#include "lapack/double/dorgqr.h"
#include "lapack/double/dorgr2.h"
#include "lapack/double/dorm2r.h"
#include "lapack/double/dormqr.h"
#include "lapack/double/dormr2.h"
#include "lapack/double/drscl.h"
#include "lapack/double/dspr.h"
#include "lapack/double/dsptrf.h"
#include "lapack/double/dsptrs.h"
#include "lapack/double/dsteqr.h"
#include "lapack/double/dtgex2.h"
#include "lapack/double/dtgexc.h"
#include "lapack/double/dtgsen.h"
#include "lapack/double/dtgsy2.h"
#include "lapack/double/dtgsyl.h"
#include "lapack/single/sgeqpf.h"
#include "lapack/single/sgeqr2.h"
#include "lapack/single/sgerq2.h"
#include "lapack/single/sggsvd.h"
#include "lapack/single/sggsvp.h"
#include "lapack/single/slacpy.h"
#include "lapack/single/slags2.h"
#include "lapack/single/slange.h"
#include "lapack/single/slapll.h"
#include "lapack/single/slapmt.h"
#include "lapack/single/slapy2.h"
#include "lapack/single/slarf.h"
#include "lapack/single/slarfg.h"
#include "lapack/single/slartg.h"
#include "lapack/single/slas2.h"
#include "lapack/single/slaset.h"
#include "lapack/single/slassq.h"
#include "lapack/single/slasv2.h"
#include "lapack/single/sorg2r.h"
#include "lapack/single/sorm2r.h"
#include "lapack/single/sormr2.h"
#include "lapack/single/stgsja.h"
#include "lapack/util/ieeeck.h"
#include "lapack/util/ilaenv.h"
#include "lapack/util/lsame.h"
#include "napack/cg.h"
#include "minpack/dpmpar.h"
#include "minpack/enorm.h"
#include "minpack/fdjac2.h"
#include "minpack/lmder.h"
#include "minpack/lmder1.h"
#include "minpack/lmdif.h"
#include "minpack/lmpar.h"
#include "minpack/qrfac.h"
#include "minpack/qrsolv.h"
#include "opt/lbfgs.h"
#include "opt/lbfgsb.h"
#include "linalg/lsqr.h"
#include "toms/rpoly.h"
#include "datapac/camsun.h"
#include "mathews/adaquad.h"
#include "mathews/simpson.h"
#include "mathews/trapezod.h"
