#include "bvrml_write.h"
#include <vcl_cmath.h>

void bvrml_write::write_vrml_header(vcl_ofstream& str)
{
  str << "#VRML V2.0 utf8\n"
      << "Background {\n"
      << "  skyColor [ 0 0 0 ]\n"
      << "  groundColor [ 0 0 0 ]\n"
      << "}\n"
      << "PointLight {\n"
      << "  on FALSE\n"
      << "  intensity 1\n"
      << "ambientIntensity 0\n"
      << "color 1 1 1\n"
      << "location 0 0 0\n"
      << "attenuation 1 0 0\n"
      << "radius 100\n"
      << "}\n";
}


void bvrml_write::write_vrml_sphere(vcl_ofstream& str, vgl_sphere_3d<float> const& sphere,
                                    const float r, const float g, const float b,
                                    const float transparency)
{
  double x0 = sphere.centre().x(), y0 = sphere.centre().y(), z0 = sphere.centre().z();
  double rad = sphere.radius();
  str << "Transform {\n"
      << "translation " << x0 << ' ' << y0 << ' '
      << ' ' << z0 << '\n'
      << "children [\n"
      << "Shape {\n"
      << " appearance Appearance{\n"
      << "   material Material\n"
      << "    {\n"
      << "      diffuseColor " << r << ' ' << g << ' ' << b << '\n'
      << "      transparency " << transparency << '\n'
      << "    }\n"
      << "  }\n"
      << " geometry Sphere\n"
      <<   "{\n"
      << "  radius " << rad << '\n'
      <<  "   }\n"
      <<  "  }\n"
      <<  " ]\n"
      << "}\n";
}


void bvrml_write::write_vrml_disk(vcl_ofstream& str,
                                  vgl_point_3d<double> const& pt,
                                  vgl_vector_3d<double> const &dir,
                                  float radius,
                                  const float r, const float g, const float b)
{
  double denom=vcl_sqrt(dir.x()*dir.x()+dir.y()*dir.y());
  float axis_x;
  float axis_y;


  if (denom==0.0)
  {
        axis_x=1.0f;
        axis_y=0.0f;
  }
  else
  {
      axis_x=-float(dir.y()/denom);
      axis_y=float(dir.x()/denom);
  }
  float phi=float(vcl_acos(dir.z()));

  double rad = radius;
  double height = 0.1;

  str << "Transform {\n"
      << "translation " << pt.x() << ' ' << pt.y() << ' '
      << ' ' << pt.z() << '\n'
      << "rotation " << 1 << ' ' <<0 << ' ' << 0 <<' ' <<-1.57<< '\n'
      << "children [\n"
      << "Transform {\n"
      << "rotation " << axis_x << ' ' <<0.0 << ' ' << axis_y <<' ' <<phi<< '\n'
      << "children [\n"
      << "Shape {\n"
      << " appearance Appearance{\n"
      << "   material Material\n"
      << "    {\n"
      << "      diffuseColor " << r << ' ' << g << ' ' << b << '\n'
      << "    }\n"
      << "  }\n"
      << " geometry Cylinder\n"
      <<   "{\n"
      << "  radius " << rad << '\n'
      << "  height " << height << '\n'
      <<  "   }\n"
      <<  "  }\n"
      <<  " ]\n"
      <<  "  }\n"
      <<  " ]\n"
      << "}\n";
}


void bvrml_write::write_vrml_line(vcl_ofstream& str,vgl_point_3d<double> pt,
                                  vgl_vector_3d<double> dir,
                                  const float length,
                                  const float r,
                                  const float g,
                                  const float b)
{
  str << "Transform {\n"
      << "translation " << 0 << ' ' << 0 << ' '
      << ' ' << 0 << '\n'
      << "children [\n"
      << "Shape {\n"
      << " appearance Appearance{\n"
      << "   material Material\n"
      << "    {\n"
      << "      diffuseColor " << r << ' ' << g << ' ' << b << '\n'
      << "      emissiveColor " << r << ' ' << g << ' ' << b << '\n'
      << "    }\n"
      << "  }\n"
      << " geometry IndexedLineSet\n"
      <<   "{\n"
      << "      coord Coordinate{\n"
      << "       point[\n"
      << pt.x() << ' ' << pt.y() << ' ' << pt.z() << '\n'
      << pt.x()+dir.x()*length << ' ' << pt.y()+dir.y()*length << ' ' << pt.z()+dir.z()*length<< '\n'
      << "   ]\n\n }"
      << "   coordIndex [\n"
      << "0,1   ]\n  }\n}"
      <<  " ]\n"
      << "}\n";
}







