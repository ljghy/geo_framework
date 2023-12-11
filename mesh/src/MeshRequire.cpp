#include <geo/mesh/Mesh.h>

using RequireFunc = void (Mesh::*)();

static RequireFunc requireFuncMap[]{
    &Mesh::constructHalfEdge,     &Mesh::computeFaceAngles,
    &Mesh::computeFaceNormals,    &Mesh::computeFaceAreas,
    &Mesh::computeVertexNormals,  &Mesh::computeVertexAreas,
    &Mesh::computeMeanEdgeLength, &Mesh::computeConnectionAngles,
    &Mesh::setVertexIndices,      &Mesh::setFaceIndices,
    &Mesh::computeVertexAngleSums};

void Mesh::require(MeshBitFlag requiredFlags)
{
    int i = 0;
    while (requiredFlags)
    {
        if ((requiredFlags & 1) && !(flags & (1 << i)))
        {
            (this->*requireFuncMap[i])();
            flags |= (1 << i);
        }
        requiredFlags >>= 1;
        ++i;
    }
}
