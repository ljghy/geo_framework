#include <algorithm>
#include <numbers>

#include <geo/field/VertexFaceConnectionAngle.h>

static inline double getVertexAngleInFace(const Face *f, const Vertex *v)
{
    int i = f->indices(0) == v->index ? 0 : (f->indices(1) == v->index ? 1 : 2);
    return f->angles(i);
}

static double halfEdgeAngle(const Vertex *v, const HalfEdge *he)
{
    const HalfEdge *hi = v->he;

    double angle = 0.0;
    if (v->onBoundary)
    {
        while (hi != he && hi->face != nullptr)
        {
            angle += getVertexAngleInFace(hi->face, v);
            hi = hi->prev->twin;
        }
        if (hi != he)
        {
            angle = 0.0;
            he = he->twin;
            hi = v->he->twin;
            while (hi != he)
            {
                angle -= getVertexAngleInFace(hi->face, v);
                hi = hi->next->twin;
            }
        }
    }
    else
    {
        while (hi != he)
        {
            angle += getVertexAngleInFace(hi->face, v);
            hi = hi->prev->twin;
        }
        angle *= std::numbers::pi * 2.0 / v->angleSum;
    }

    return angle;
}

Eigen::MatrixX3d vertexFaceConnectionAngle(Mesh &mesh)
{
    mesh.require(Mesh::FaceIndices | Mesh::HalfEdgeStructure |
                 Mesh::FaceAngles | Mesh::VertexAngleSums);

    Eigen::MatrixX3d angles(mesh.nF(), 3);

    std::for_each(
        mesh.faces.begin(), mesh.faces.end(),
        [&mesh, &angles](const Face &f)
        {
            angles(f.index, 0) =
                halfEdgeAngle(&mesh.vertices[f.indices(0)], f.he);
            angles(f.index, 1) =
                halfEdgeAngle(&mesh.vertices[f.indices(1)], f.he->twin) -
                std::numbers::pi;
            angles(f.index, 2) =
                halfEdgeAngle(&mesh.vertices[f.indices(2)], f.he->prev) -
                std::numbers::pi - f.angles(0);
        });

    return angles;
}
