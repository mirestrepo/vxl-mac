#ifdef __GNUG__
#pragma implementation
#endif

#include "FMatrixPlanar.h"

#include <vcl_cassert.h>
#include <vcl_iostream.h>
#include <vcl_cmath.h>
//#include <vcl_memory.h>
#include <vcl_cstdlib.h>
#include <vnl/vnl_matrix.h>

#include <vnl/algo/vnl_svd.h>
#include <vnl/vnl_matops.h> // use vnl_matlab_print.h for pretty printing
#include <vnl/algo/vnl_symmetric_eigensystem.h>

#include <mvl/HomgOperator2D.h>
#include <mvl/HomgPoint2D.h>
#include <mvl/FMatrix.h>

//--------------------------------------------------------------
//
// -- Default constructor.

FMatrixPlanar::FMatrixPlanar()
{
  _rank2_flag = true;
}

//--------------------------------------------------------------
//
// -- Constructor.

FMatrixPlanar::FMatrixPlanar(const double* f_matrix)
{
  _rank2_flag = true;
  set(f_matrix);
}

//--------------------------------------------------------------
//
// -- Constructor.

FMatrixPlanar::FMatrixPlanar(const vnl_matrix<double>& f_matrix)
{
  _rank2_flag = true;
  set(f_matrix.data_block());
}



//--------------------------------------------------------------
//
// -- Destructor.

FMatrixPlanar::~FMatrixPlanar()
{
}


//-------------------------------------------------------------------
//
// -- Null function as already Rank 2.

inline void 
FMatrixPlanar::set_rank2_using_svd (void)
{
}

//-----------------------------------------------------------------------------
//
// -- Returns current matrix which is already Rank 2.

inline  FMatrixPlanar
FMatrixPlanar::get_rank2_truncated()
{
     return *this;
}




//--------------------------------------------------------------
// 
// -- Set the fundamental matrix using the two-dimensional 
// array f_matrix. Only returns true if f_matrix contained a 
// planar matrix, not an approximation to one.
// Otherwise returns false and the matrix is not set. 
// Patch on FMatrixSkew::set (const vnl_matrix<double>& f_matrix ).

bool FMatrixPlanar::set (const double* f_matrix )
{
     vnl_matrix<double> temp(f_matrix,3,3);
     return set(temp);
}


//--------------------------------------------------------------
// 
// -- Set the fundamental matrix using the vnl_matrix<double> 
// f_matrix. Only returns true if f_matrix contained a 
// planar matrix, not an approximation to one.
// The test is against a Rank 2 constraint for 
// both @{${\tt F}$@} and the symmetric part @{\(({\tt F}+{\tt F}^\top)\)@}.
// Otherwise returns false and the matrix is not set. 

inline bool 
FMatrixPlanar::set (const vnl_matrix<double>& f_matrix )
{
     int 
	  row_index, col_index;

#if PARANOID

     // CRUDE test for planar form with tolerance 0
     // test F and F+F' are Rank 2
     // HACK: has been alterd to have some tolerances
     bool planar = true;
     vnl_svd<double> svd(f_matrix,1e-8);
     if (svd.rank()!=2) 
     { 
	  planar = false; 
	  vcl_cerr << "WARNING in FMatrixPlanar::set" << vcl_endl;
	  vcl_cerr << "F matrix not rank 2: svd = " << svd.W() << vcl_endl;
     }
     else
     {
	  vnl_svd<double> svd(f_matrix + f_matrix.transpose(),1e-8);
	  if (svd.rank()!=2) 
	  { 
	       planar = false; 
	       vcl_cerr << "WARNING in FMatrixPlanar::set" << vcl_endl;
	       vcl_cerr << "Symmetric part matrix not rank 2: svd = " << svd.W() << vcl_endl;
	  }
     }
     
     if (!planar)
     {
	  vcl_cerr << "WARNING: F matrix not planar so cannot allocate to FMatrixPlanar\n" ;
	  return FALSE;
     }
		 
#endif

     for (row_index = 0; row_index < 3; row_index++)
	  for (col_index = 0; col_index < 3; col_index++)
	  {
	       _f_matrix. put (row_index, col_index,f_matrix.get(row_index,col_index));
	       _ft_matrix. put (col_index, row_index,f_matrix.get(row_index,col_index));
	  }


     // set rank flag true
     
     this->set_rank2_flag(true);
     
     return true;
}    


//----------------------------------------------------------------
//
// -- Returns the _rank2_flag which is always true for FMatrixPlanar.

inline bool 
FMatrixPlanar::get_rank2_flag (void) const
{
     return true;
}

//----------------------------------------------------------------
//
// -- Set the _rank2_flag. Null function as always set true.

inline void 
FMatrixPlanar::set_rank2_flag (bool) const
{
}

//----------------------------------------------------------------
//
// -- Initilises the FMatrixPlanar using a general fundamental matrix F
// by finding the nearest planar fundamental matrix to F.
// This should be used prior to FMPlanarComputeNonLinear to give
// a initial value for the non-linear minimisation.
// This function is rreuiqred as trying to set FMatrixPlanar using a 
// general fundamental matrix
// will fail as it does not satisfy the extra constraint of 
// @{\(\det ({\tt F} + {\tt F}^\top) = 0\)@}. 

void FMatrixPlanar::init(const FMatrix& F)
{
     // this converts to 6 parameter form of [e2]x[ls]x[e1]x - see A Zisserman
     // HACK this is not the most efficient/accurate way to covert to this form
     // as it goes via the Armstrong inplementation of the 
     // Lingrand Veiville formula (ECCV96).
     // This should be redone at some point.

     HomgPoint2D e1,e2;
     F.get_epipoles(&e1,&e2);
     
     vnl_symmetric_eigensystem<double>  symm_eig(F.get_matrix()+F.get_matrix().transpose());

     double eig0 = symm_eig.D(0,0);
     double eig1 = symm_eig.D(2,2);

     vnl_vector<double> v0(symm_eig.get_eigenvector(0));
     vnl_vector<double> v1(symm_eig.get_eigenvector(2));
     
     vnl_vector<double> f1(3),f2(3);

     if (eig0 > 0 && eig1 < 0) {
	  f1 = sqrt(eig0)*v0 + sqrt(-eig1)*v1;
	  f2 = sqrt(eig0)*v0 - sqrt(-eig1)*v1;
     }
     else if (eig0 < 0 && eig1 > 0) {
	  f1 = sqrt(eig1)*v1 + sqrt(-eig0)*v0;
	  f2 = sqrt(eig1)*v1 - sqrt(-eig0)*v0;
     }
     else {
	  vcl_cerr << "ERROR in FMatrix::init" << vcl_endl
	       << "EXITING..." << vcl_endl;
	  assert(false);
     }

     vnl_vector<double> ls;
     if (fabs(HomgOperator2D::dot(e1,f1)/e1.w())+
	 fabs(HomgOperator2D::dot(e2,f1)/e2.w()) >
	 fabs(HomgOperator2D::dot(e1,f2)/e1.w())+
	 fabs(HomgOperator2D::dot(e2,f2)/e2.w()) )
	  ls = f1;
     else
	  ls = f2;

     ls /= ls.magnitude();

     double ls_thi = acos(ls.z());
     if (ls_thi < 0) ls_thi += vnl_math::pi;
     
     double ls_theta;
     if (ls.y() >= 0)
	  ls_theta =  acos(ls.x()/sin(ls_thi));
     else
	  ls_theta = -acos(ls.x()/sin(ls_thi));

     
     double ls1 = cos(ls_theta)*sin(ls_thi);
     double ls2 = sin(ls_theta)*sin(ls_thi);
     double ls3 = cos(ls_thi);

     double list1[9] = {0,-1.0,e1.y()/e1.w(),
			1,0,-e1.x()/e1.w(),
			-e1.y()/e1.w(),e1.x()/e1.w(),0};
     double list2[9] = {0,-ls3,ls2,ls3,0,-ls1,-ls2,ls1,0};
     double list3[9] = {0,-1.0,e2.y()/e2.w(),
			1,0,-e2.x()/e2.w(),
			-e2.y()/e2.w(),e2.x()/e2.w(),0};
  
     vnl_matrix<double> mat1(3,3,9,list1),mat2(3,3,9,list2),mat3(3,3,9,list3);
     
     vnl_matrix<double> fmat = mat1*mat2*mat3;

     fmat /= fmat.fro_norm();
     
     // store the corrected fmatrix
     set(fmat);

}
