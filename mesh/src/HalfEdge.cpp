#include <geo/mesh/Vertex.h>
#include <geo/mesh/HalfEdge.h>

Eigen::Vector3d HalfEdge::vec() const
{
    return next->origin->position - origin->position;
}
