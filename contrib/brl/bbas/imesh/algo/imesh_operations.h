// This is brl/bbas/imesh/algo/imesh_operations.h
#ifndef imesh_algo_operations_h_
#define imesh_algo_operations_h_

//:
// \file
// \brief Operations on meshes that have additional dependencies
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 6/24/08
//
// \verbatim
//  Modifications
// \endverbatim


#include <imesh/imesh_mesh.h>



//: Compute the dual mesh using vertex normals to compute dual vertices
//  the old vertices are used as constraints when the solution is singular
//  the singularity is enforced if a singular value is less than tau * max singular value 
imesh_mesh dual_mesh_with_normals(const imesh_mesh& mesh,
                                  const imesh_vertex_array_base& old_verts,
                                  double tau = 1e-2);


#endif // imesh_algo_operations_h_
