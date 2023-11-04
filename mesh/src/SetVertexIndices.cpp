#include <geo/mesh/Mesh.h>

void Mesh::setVertexIndices()
{
    for (size_t i = 0; i < vertices.size(); ++i)
        vertices[i].index = i;
    flags |= VertexIndices;
}
