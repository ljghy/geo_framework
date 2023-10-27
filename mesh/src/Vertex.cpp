#include <geo/mesh/Vertex.h>
#include <geo/mesh/HalfEdge.h>

void Vertex::forEachHalfEdge(std::function<void(HalfEdge *)> func)
{
    HalfEdge *hi = he, *start = he;
    do
    {
        func(hi);
    } while ((hi = hi->prev->twin) != start);
}

void Vertex::forEachHalfEdge(std::function<void(const HalfEdge *)> func) const
{
    const HalfEdge *hi = he, *start = he;
    do
    {
        func(hi);
    } while ((hi = hi->prev->twin) != start);
}

void Vertex::forEachFace(std::function<void(Face *)> func)
{
    HalfEdge *hi = he, *start = he;
    do
    {
        if (hi->face != nullptr)
            func(hi->face);
    } while ((hi = hi->prev->twin) != start);
}

void Vertex::forEachFace(std::function<void(const Face *)> func) const
{
    const HalfEdge *hi = he, *start = he;
    do
    {
        if (hi->face != nullptr)
            func(hi->face);
    } while ((hi = hi->prev->twin) != start);
}
