#ifndef vsph_spherical_coord_h_
#define vsph_spherical_coord_h_
//:
// \file
// \brief 3D spherical coordinate system
// \author Gamze Tunali
//
// \verbatim
//  Modifications
// \endverbatim

#include "vsph_spherical_coord_sptr.h"
#include <vgl/vgl_point_3d.h>
#include <vbl/vbl_ref_count.h>
#include <vcl_iostream.h>

//: a point in the spherical coordinate system
class vsph_sph_point_3d
{
 public:
  //: Default constructor
  vsph_sph_point_3d() : radius_(1.0), theta_(0.0), phi_(0.0) {}
  vsph_sph_point_3d(double r, double theta, double phi) : radius_(r), theta_(theta), phi_(phi) {}

  ~vsph_sph_point_3d(){}

  void set(double r, double theta, double phi) { radius_=r; theta_=theta; phi_=phi; }

  void print(vcl_ostream& os) const;

  double radius_;
  double theta_;
  double phi_;
};


//: 3D coordinate system specified by distance rho, angles theta (azimuth) and phi (polar, zenith).
class vsph_spherical_coord : public vbl_ref_count
{
 public:
  //***************************************************************************
  // Constructors/Destructor
  //***************************************************************************

  // Default constructor
  vsph_spherical_coord() : radius_(1), origin_(vgl_point_3d<double>(0,0,0)) {}

  vsph_spherical_coord(vgl_point_3d<double> origin, double radius);

  //: Copy constructor
  vsph_spherical_coord(vsph_spherical_coord const& rhs) 
    : origin_(rhs.origin_), radius_(rhs.radius_) {}

  // Destructor
  virtual ~vsph_spherical_coord() {}

  //***************************************************************************
  // Methods
  //***************************************************************************

  double radius() const { return radius_; }

  vgl_point_3d<double> origin() const { return origin_; }

  vsph_sph_point_3d create_point(double theta, double phi) { return vsph_sph_point_3d(radius_,theta,phi); }

  void spherical_coord(vgl_point_3d<double> cp, vsph_sph_point_3d& sp);

  //: converts to cartesian coordinate
  vgl_point_3d<double> cart_coord(vsph_sph_point_3d const& p) const;

  //: converts to cartesian coordinate
  vgl_point_3d<double> cart_coord(double theta, double phi) const { return cart_coord(vsph_sph_point_3d(radius_,theta,phi)); }

  //: moves the point onto the surface of the sphereon the ray from origin to the point
  // Returns true if the point changed, false if it was already on the sphere
  bool move_point(vsph_sph_point_3d& p);

  void print(vcl_ostream& os) const;

 private:
  double radius_;                // distance from the origin
  vgl_point_3d<double> origin_;  // the origin in cartesian coordinates
};

vcl_ostream& operator<<(vcl_ostream& os, vsph_sph_point_3d const& p);
vcl_ostream& operator<<(vcl_ostream& os, vsph_spherical_coord const& p);
vcl_ostream& operator<<(vcl_ostream& os, vsph_spherical_coord_sptr const& p);

#endif // vsph_spherical_coord_h_
