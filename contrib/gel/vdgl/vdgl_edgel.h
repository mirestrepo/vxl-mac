#ifndef vdgl_edgel_h
#define vdgl_edgel_h
#ifdef __GNUC__
#pragma interface
#endif

// .NAME vdgl_edgel - Represents a 2D edgel
// .INCLUDE vgl/vdgl_edgel.h
// .FILE vdgl_edgel.txx
//
// .SECTION Description
//  A 2d image edgel
//
// .SECTION Author
//    Geoff Cross
// Created: xxx xx xxxx

#include <vcl_iostream.h>
#include <vgl/vgl_point_2d.h>

class vdgl_edgel {
   // PUBLIC INTERFACE----------------------------------------------------------
public:
  
  // Constructors/Destructors--------------------------------------------------
  vdgl_edgel() {}
  vdgl_edgel( const double x, const double y, const double grad= -1, const double theta= 0 );
  ~vdgl_edgel() {}

  // Operators----------------------------------------------------------------

  vdgl_edgel& operator=(const vdgl_edgel& that);

  friend bool operator==( const vdgl_edgel &e1, const vdgl_edgel &e2);
  friend ostream& operator<<(ostream& s, const vdgl_edgel& p);
  // Data Access---------------------------------------------------------------
  
  // getters
  inline vgl_point_2d<double> get_pt() const { return p_; }
  inline double get_x() const { return p_.x(); }
  inline double get_y() const { return p_.y(); }
  inline double get_grad() const { return grad_; }
  inline double get_theta() const { return theta_; }
  inline double x() const { return p_.x(); }
  inline double y() const { return p_.y(); }

  // setters
  inline void set_x( const double x) { p_.set_x( x); }
  inline void set_y( const double y) { p_.set_y( y); }
  inline void set_grad( const double grad) { grad_= grad; }
  inline void set_theta( const double theta) { theta_= theta; }
    
  // Data Control--------------------------------------------------------------

  // Computations--------------------------------------------------------------
  
  // INTERNALS-----------------------------------------------------------------
protected:
  // Data Members--------------------------------------------------------------

  vgl_point_2d<double> p_;
  double grad_;
  double theta_; 

private:
  // Helpers-------------------------------------------------------------------
};


bool operator==( const vdgl_edgel &e1, const vdgl_edgel &e2);
ostream& operator<<(ostream& s, const vdgl_edgel& p);

#endif // _vdgl_edgel_h
