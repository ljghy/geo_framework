#include <geo/mesh/Mesh.h>

void Mesh::require(uint32_t requiredFlags)
{
    int i = 0;
    while (requiredFlags)
    {
        if ((requiredFlags & 1) && !(flags & (1 << i)))
        {
            switch (i)
            {
            case 0:
                constructHalfEdge();
                break;
            case 1:
                computeFaceAngles();
                break;
            case 2:
                computeFaceNormals();
                break;
            case 3:
                computeFaceAreas();
                break;
            case 4:
                computeVertexNormals();
                break;
            case 5:
                computeVertexAreas();
                break;
            case 6:
                computeMeanEdgeLength();
                break;
            }
        }
        requiredFlags >>= 1;
        ++i;
    }
}
