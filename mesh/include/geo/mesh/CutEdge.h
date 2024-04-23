#ifndef GEO_CUT_EDGE_H_
#define GEO_CUT_EDGE_H_

#include <geo/mesh/IndexBasedHalfEdge.h>
#include <geo/mesh/Mesh.h>

void cutEdge(Mesh &mesh, IndexBasedHalfEdgeStructure &heStruct, int he,
             double t, int *updatedFaces = nullptr);

#endif
