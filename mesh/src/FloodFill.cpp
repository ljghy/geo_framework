#include <stack>

#include <geo/mesh/FloodFill.h>

void floodFillFace(Mesh &mesh, const std::vector<int> &barrier, int startFace,
                   std::vector<int> &filledFaces)
{
    mesh.require(Mesh::HalfEdgeStructure | Mesh::FaceIndices);

    std::vector<bool> visited(mesh.nF(), false);
    for (int i : barrier)
        visited[i] = true;
    visited[startFace] = true;

    filledFaces.push_back(startFace);

    std::stack<int> s;
    s.push(startFace);
    while (!s.empty())
    {
        const Face *f = &mesh.faces[s.top()];
        s.pop();
        const HalfEdge *he = f->he;
        for (int i = 0; i < 3; ++i)
        {
            if (he->twin != nullptr)
            {
                int j = he->twin->face->index;
                if (!visited[j])
                {
                    visited[j] = true;
                    filledFaces.push_back(j);
                    s.push(j);
                }
            }
            he = he->next;
        }
    }
}
