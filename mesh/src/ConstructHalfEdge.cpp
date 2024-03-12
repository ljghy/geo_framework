#include <geo/mesh/PairHash.h>
#include <geo/mesh/Mesh.h>

void Mesh::constructHalfEdge()
{
    halfEdges.clear();
    halfEdges.reserve(2 * (faces.size() + vertices.size()));

    std::unordered_map<std::pair<int, int>, HalfEdge *, pair_hash> edgeMap;
    edgeMap.reserve(2 * (faces.size() + vertices.size()));

    for (size_t f = 0; f < faces.size(); ++f)
    {
        auto &face = faces[f];

        HalfEdge *prevEdge = nullptr;
        HalfEdge *firstEdge = nullptr;

        for (int i = 0; i < 3; ++i)
        {
            int startIndex = face.indices[i];
            int endIndex = face.indices[(i + 1) % 3];

            auto &startVertex = vertices[startIndex];

            HalfEdge *edge, *twinEdge;

            auto edgePair = std::make_pair(startIndex, endIndex),
                 twinPair = std::make_pair(endIndex, startIndex);

            auto iter = edgeMap.find(edgePair);
            if (iter == edgeMap.end())
            {
                halfEdges.emplace_back(new HalfEdge);
                edge = halfEdges.back().get();
                edgeMap.insert({edgePair, edge});
            }
            else
                edge = iter->second;

            iter = edgeMap.find(twinPair);
            if (iter == edgeMap.end())
            {
                halfEdges.emplace_back(new HalfEdge);
                twinEdge = halfEdges.back().get();
                edgeMap.insert({twinPair, twinEdge});
            }
            else
                twinEdge = iter->second;

            edge->twin = twinEdge;
            twinEdge->twin = edge;

            edge->origin = &startVertex;
            edge->face = &face;

            if (!startVertex.he)
                startVertex.he = edge;

            if (prevEdge)
            {
                prevEdge->next = edge;
                edge->prev = prevEdge;
            }
            else
                firstEdge = edge;

            prevEdge = edge;
        }

        firstEdge->prev = prevEdge;
        prevEdge->next = firstEdge;
        face.he = firstEdge;
    }

    for (auto &vert : vertices)
        vert.onBoundary = false;

    for (const auto &edge : halfEdges)
    {
        if (edge->face != nullptr)
            continue;

        edge->origin = edge->twin->next->origin;
        edge->origin->onBoundary = true;

        auto he = edge->twin;
        while (he->face)
        {
            he = he->next->twin;
        }
        edge->prev = he;
        he->next = edge.get();
    }
    flags |= HalfEdgeStructure;
}
