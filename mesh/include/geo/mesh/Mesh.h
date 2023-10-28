#ifndef GEO_MESH_H_
#define GEO_MESH_H_

#include <memory>
#include <vector>
#include <cstdint>

#include <geo/mesh/Face.h>
#include <geo/mesh/HalfEdge.h>
#include <geo/mesh/Vertex.h>

struct Mesh
{
    enum MeshBitFlag : uint32_t
    {
        HalfEdgeStructure = 1 << 0,

        FaceAngles = 1 << 1,
        FaceNormals = 1 << 2,
        FaceAreas = 1 << 3,

        VertexNormals = 1 << 4,
        VertexAreas = 1 << 5,

        MeanEdgeLength = 1 << 6,

        ConnectionAngles = 1 << 7,
    };

    uint32_t flags = 0;

    std::vector<Vertex> vertices;
    std::vector<Face> faces;
    std::vector<std::unique_ptr<HalfEdge>> halfEdges;

    double meanEdgeLength;

    void require(uint32_t flags);

    void constructHalfEdge();
    void computeFaceAngles();
    void computeFaceNormals();
    void computeFaceAreas();
    void computeVertexNormals();
    void computeVertexAreas();
    void computeMeanEdgeLength();
    void computeConnectionAngles();

    Eigen::Matrix3Xd getPositionMatrix() const;
    Eigen::Matrix3Xd getVertexNormalMatrix();
    Eigen::VectorXd getVertexMassVector();

    size_t nV() const { return vertices.size(); }
    size_t nF() const { return faces.size(); }

    size_t getVertexIndex(const Vertex *vertex) const
    {
        return vertex - vertices.data();
    }
    size_t getFaceIndex(const Face *face) const { return face - faces.data(); }
};

#endif
