#include <geo/mesh/Mesh.h>

void Mesh::setFaceIndices()
{
    for (size_t i = 0; i < faces.size(); ++i)
        faces[i].index = i;
    flags |= FaceIndices;
}
