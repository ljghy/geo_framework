#include <geo/mesh/Mesh.h>

// boost hash_combine
#include <functional>

template <typename T>
inline void hash_combine(std::size_t &seed, const T &val)
{
    seed ^= std::hash<T>()(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}
template <typename T>
inline void hash_val(std::size_t &seed, const T &val)
{
    hash_combine(seed, val);
}
template <typename T, typename... Types>
inline void hash_val(std::size_t &seed, const T &val, const Types &...args)
{
    hash_combine(seed, val);
    hash_val(seed, args...);
}
template <typename... Types>
inline std::size_t hash_val(const Types &...args)
{
    std::size_t seed = 0;
    hash_val(seed, args...);
    return seed;
}
struct pair_hash
{
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &p) const
    {
        return hash_val(p.first, p.second);
    }
};

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
