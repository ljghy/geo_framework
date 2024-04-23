#include <unordered_map>

#include <geo/mesh/IndexBasedHalfEdge.h>

void IndexBasedHalfEdgeStructure::construct(Mesh &mesh)
{
    mesh.require(Mesh::HalfEdgeStructure | Mesh::VertexIndices |
                 Mesh::FaceIndices);

    std::unordered_map<HalfEdge *, int> halfEdgeIndex;

    halfEdges.resize(mesh.halfEdges.size());
    vertHalfEdge.resize(mesh.vertices.size());
    faceHalfEdge.resize(mesh.faces.size());

    for (size_t i = 0; i < mesh.halfEdges.size(); ++i)
        halfEdgeIndex[mesh.halfEdges[i].get()] = i;

    for (size_t i = 0; i < mesh.halfEdges.size(); ++i)
    {
        auto &he = mesh.halfEdges[i];

        halfEdges[i] = std::make_unique<IndexBasedHalfEdge>();

        halfEdges[i]->origin = he->origin->index;
        halfEdges[i]->twin = halfEdgeIndex[he->twin];
        halfEdges[i]->prev = he->prev ? halfEdgeIndex[he->prev] : -1;
        halfEdges[i]->next = he->next ? halfEdgeIndex[he->next] : -1;
        halfEdges[i]->face = he->face ? he->face->index : -1;
    }

    for (size_t i = 0; i < mesh.nV(); ++i)
        vertHalfEdge[i] = halfEdgeIndex[mesh.vertices[i].he];

    for (size_t i = 0; i < mesh.nF(); ++i)
        faceHalfEdge[i] = halfEdgeIndex[mesh.faces[i].he];
}

void IndexBasedHalfEdgeStructure::toPointerBased(Mesh &mesh)
{
    mesh.halfEdges.resize(halfEdges.size());
    for (size_t i = 0; i < halfEdges.size(); ++i)
        mesh.halfEdges[i] = std::make_unique<HalfEdge>();

    for (size_t i = 0; i < halfEdges.size(); ++i)
    {
        auto &he = mesh.halfEdges[i];
        const auto &heIndex = halfEdges[i];

        he->origin = &mesh.vertices[heIndex->origin];
        he->twin = mesh.halfEdges[heIndex->twin].get();
        he->prev =
            heIndex->prev == -1 ? nullptr : mesh.halfEdges[heIndex->prev].get();
        he->next =
            heIndex->next == -1 ? nullptr : mesh.halfEdges[heIndex->next].get();
        he->face = heIndex->face == -1 ? nullptr : &mesh.faces[heIndex->face];
    }

    for (size_t i = 0; i < mesh.nV(); ++i)
        mesh.vertices[i].he = mesh.halfEdges[vertHalfEdge[i]].get();

    for (size_t i = 0; i < mesh.nF(); ++i)
        mesh.faces[i].he = mesh.halfEdges[faceHalfEdge[i]].get();
}
