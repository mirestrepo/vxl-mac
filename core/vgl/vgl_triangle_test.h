#ifndef vgl_triangle_test_h_
#define vgl_triangle_test_h_
#ifdef __GNUC__
#pragma interface
#endif
// .NAME vgl_triangle_test
// .INCLUDE vgl/vgl_triangle_test.h
// .FILE vgl_triangle_test.cxx
// @author fsm@robots.ox.ac.uk

// Returns determinant of [ x1 x2 x3 ]
//                        [ y1 y2 y3 ]
//                        [ 1  1  1  ]
template <class T>
T vgl_triangle_test_discriminant(T x1, T y1,
				 T x2, T y2,
				 T x3, T y3);

// Function returns true if (x, y) is inside, or on the boundary 
// of, the triangle whose vertices are (xi, yi), i=1,2,3.
template <class T>
bool vgl_triangle_test_inside(T x1, T y1, 
			      T x2, T y2, 
			      T x3, T y3,
			      T x , T y );


#endif // vgl_triangle_test_h_
