#include <geo/mesh/Mesh.h>

void Mesh::computeMeanEdgeLength()
{
    meanEdgeLength = 0.0;

    for (const auto &face : faces)
    {
        for (int j = 0; j < 3; ++j)
        {
            meanEdgeLength += (vertices[face.indices(j)].position -
                               vertices[face.indices((j + 1) % 3)].position)
                                  .norm();
        }
    }
    meanEdgeLength /= 3.0 * faces.size();

    flags |= MeanEdgeLength;
}