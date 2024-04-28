#include <queue>
#include <unordered_set>

#include <geo/mesh/Mesh.h>

std::vector<std::vector<int>> Mesh::getVertexNRing(int n)
{
    std::vector<std::vector<int>> vertexNRing(vertices.size());
    require(Mesh::VertexIndices);
    auto oneRing = getVertexOneRing();
    for (size_t i = 0; i < nV(); ++i)
    {
        std::queue<int> q;
        q.push(i);
        std::unordered_set<int> visited;
        visited.insert(i);

        for (int j = 0; j < n; ++j)
        {
            auto sz = q.size();
            for (size_t s = 0; s < sz; ++s)
            {
                int k = q.front();
                q.pop();
                for (auto l : oneRing[k])
                {
                    auto [_, inserted] = visited.insert(l);
                    if (inserted)
                        q.push(l);
                }
            }
        }

        vertexNRing[i] = std::vector<int>(visited.begin(), visited.end());
    }

    return vertexNRing;
}