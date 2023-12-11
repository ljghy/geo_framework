#ifndef GEO_MESH_H_
#define GEO_MESH_H_

#include <memory>
#include <vector>
#include <map>
#include <cstdint>

#include <geo/mesh/Face.h>
#include <geo/mesh/HalfEdge.h>
#include <geo/mesh/Vertex.h>

struct Mesh
{
    Mesh() = default;

    template <typename DerivedV, typename DerivedF>
    Mesh(const Eigen::MatrixBase<DerivedV> &V,
         const Eigen::MatrixBase<DerivedF> &F)
    {
        vertices.reserve(V.rows());
        for (int i = 0; i < V.rows(); ++i)
        {
            vertices.emplace_back(V.row(i));
        }

        faces.reserve(F.rows());
        for (int i = 0; i < F.rows(); ++i)
        {
            faces.emplace_back(F.row(i));
        }
    }

    using MeshBitFlag = uint32_t;

    enum MeshBitFlag_ : MeshBitFlag
    {
        HalfEdgeStructure = 1 << 0,

        FaceAngles = 1 << 1,
        FaceNormals = 1 << 2,
        FaceAreas = 1 << 3,

        VertexNormals = 1 << 4,
        VertexAreas = 1 << 5,

        MeanEdgeLength = 1 << 6,

        ConnectionAngles = 1 << 7,

        VertexIndices = 1 << 8,
        FaceIndices = 1 << 9,

        VertexAngleSums = 1 << 10,
    };

    MeshBitFlag flags = 0;

    std::vector<Vertex> vertices;
    std::vector<Face> faces;
    std::vector<std::unique_ptr<HalfEdge>> halfEdges;

    double meanEdgeLength;

    void require(MeshBitFlag flags);

    void constructHalfEdge();
    void computeFaceAngles();
    void computeFaceNormals();
    void computeFaceAreas();
    void computeVertexNormals();
    void computeVertexAreas();
    void computeMeanEdgeLength();
    void computeConnectionAngles();
    void setVertexIndices();
    void setFaceIndices();
    void computeVertexAngleSums();

    void computeIntrinsicFaceAngles(const Eigen::MatrixX3d &l);
    void computeIntrinsicFaceAreas(const Eigen::MatrixX3d &l);

    Eigen::Matrix3Xd getVertexPositionMatrix() const;
    Eigen::MatrixX3d getVertexPositionMatrixTransposed() const;
    Eigen::MatrixX3d getVMatrix() const;
    Eigen::Matrix3Xi getFaceIndicesMatrix() const;
    Eigen::MatrixX3i getFaceIndicesMatrixTransposed() const;
    Eigen::MatrixX3i getFMatrix() const;
    Eigen::Matrix3Xd getVertexNormalMatrix();
    Eigen::VectorXd getVertexMassVector();
    Eigen::MatrixX3d getEdgeLengthMatrix() const;

    std::vector<std::vector<int>> getVertexOneRing();
    std::vector<std::vector<int>> getVertexOneRingWithCenter();
    std::vector<std::map<int, int>> getVertexOneRingMap();
    std::vector<std::map<int, int>> getVertexOneRingMapWithCenter();

    size_t nV() const { return vertices.size(); }
    size_t nF() const { return faces.size(); }

    size_t getVertexIndex(const Vertex *vertex) const
    {
        return vertex - vertices.data();
    }
    size_t getFaceIndex(const Face *face) const { return face - faces.data(); }
};

#endif
