#include <testlib/testlib_register.h>

DECLARE( test_cartesian );
DECLARE( test_distance );
DECLARE( test_conic );
DECLARE( test_homg );
DECLARE( test_polygon );
DECLARE( test_convex );
DECLARE( test_polygon_scan_iterator );
DECLARE( test_triangle_scan_iterator );
DECLARE( test_ellipse_scan_iterator );
DECLARE( test_window_scan_iterator );
DECLARE( test_area );
DECLARE( test_clip );
DECLARE( test_h_matrix_1d );
DECLARE( test_h_matrix_2d );
DECLARE( test_h_matrix_3d );
DECLARE( test_fit_lines_2d );
DECLARE( test_fit_conics_2d );
DECLARE( test_p_matrix );
DECLARE( test_closest_point );
DECLARE( test_convex_hull_2d );
DECLARE( test_sphere );
DECLARE( test_line_3d_2_points );
DECLARE( test_line_segment_3d );
DECLARE( test_orient_box_3d );
DECLARE( test_fit_plane_3d );
DECLARE( test_intersection );
DECLARE( test_rotation_3d );
DECLARE( test_triangle_3d );
DECLARE( test_triangle_3d_line_intersection );
DECLARE( test_rtree );
DECLARE( test_ellipsoid );
DECLARE( test_infinite_line_3d );

void
register_tests()
{
  REGISTER( test_cartesian );
  REGISTER( test_distance );
  REGISTER( test_conic );
  REGISTER( test_homg );
  REGISTER( test_polygon );
  REGISTER( test_convex );
  REGISTER( test_polygon_scan_iterator );
  REGISTER( test_triangle_scan_iterator );
  REGISTER( test_ellipse_scan_iterator );
  REGISTER( test_window_scan_iterator );
  REGISTER( test_area );
  REGISTER( test_clip );
  REGISTER( test_h_matrix_1d );
  REGISTER( test_h_matrix_2d );
  REGISTER( test_h_matrix_3d );
  REGISTER( test_fit_lines_2d );
  REGISTER( test_fit_conics_2d );
  REGISTER( test_p_matrix );
  REGISTER( test_closest_point );
  REGISTER( test_convex_hull_2d );
  REGISTER( test_sphere );
  REGISTER( test_line_3d_2_points );
  REGISTER( test_line_segment_3d );
  REGISTER( test_orient_box_3d );
  REGISTER( test_fit_plane_3d );
  REGISTER( test_intersection );
  REGISTER( test_rotation_3d );
  REGISTER( test_triangle_3d );
  REGISTER( test_triangle_3d_line_intersection );
  REGISTER( test_rtree );
  REGISTER( test_ellipsoid );
  REGISTER( test_infinite_line_3d );
}

DEFINE_MAIN;
