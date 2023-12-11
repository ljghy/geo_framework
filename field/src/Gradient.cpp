#include <algorithm>
#include <numbers>

#include <Eigen/Dense>

#include <geo/field/Gradient.h>

void computeGradient(Mesh &mesh, const Eigen::VectorXd &phi,
                     Eigen::Matrix3Xd &g)
{
    mesh.require(Mesh::FaceAreas | Mesh::FaceNormals);

    g.resize(3, mesh.nF());
    std::for_each(
        mesh.faces.begin(), mesh.faces.end(),
        [&mesh, &phi, &g](const Face &face)
        {
            auto i = mesh.getFaceIndex(&face);
            const auto &I = face.indices;
            Eigen::Vector3d e[3]{
                mesh.vertices[I(1)].position - mesh.vertices[I(0)].position,
                mesh.vertices[I(2)].position - mesh.vertices[I(1)].position,
                mesh.vertices[I(0)].position - mesh.vertices[I(2)].position};

            g.col(i) = face.normal.cross(phi(I(2)) * e[0] + phi(I(0)) * e[1] +
                                         phi(I(1)) * e[2]) /
                       (2.0 * face.area);
        });
}

std::vector<Eigen::Matrix3d> gradientOperator(Mesh &mesh)
{
    mesh.require(Mesh::FaceAreas | Mesh::FaceNormals);

    std::vector<Eigen::Matrix3d> G(mesh.nF());
    std::for_each(
        mesh.faces.begin(), mesh.faces.end(),
        [&mesh, &G](const Face &face)
        {
            auto i = mesh.getFaceIndex(&face);
            const auto &I = face.indices;
            Eigen::Vector3d e[3]{
                mesh.vertices[I(1)].position - mesh.vertices[I(0)].position,
                mesh.vertices[I(2)].position - mesh.vertices[I(1)].position,
                mesh.vertices[I(0)].position - mesh.vertices[I(2)].position};

            G[i].col(0) = face.normal.cross(e[1]) / (2.0 * face.area);
            G[i].col(1) = face.normal.cross(e[2]) / (2.0 * face.area);
            G[i].col(2) = face.normal.cross(e[0]) / (2.0 * face.area);
        });

    return G;
}

static inline double getVertexAngleInFace(const Face *f, const Vertex *v)
{
    int i = f->indices(0) == v->index ? 0 : (f->indices(1) == v->index ? 1 : 2);
    return f->angles(i);
}

std::vector<Eigen::VectorXcd>
intrinsicVertexGradientOperator(Mesh &mesh, const Eigen::MatrixX3d &l)
{
    std::vector<Eigen::VectorXcd> gradOps(mesh.nV());

    mesh.require(Mesh::VertexIndices | Mesh::FaceIndices |
                 Mesh::HalfEdgeStructure | Mesh::FaceAngles |
                 Mesh::VertexAngleSums | Mesh::VertexAreas);

    std::for_each(
        mesh.vertices.begin(), mesh.vertices.end(),
        [&](const auto &v)
        {
            size_t oneRingSize = 0;
            v.forEachHalfEdge([&oneRingSize](const HalfEdge *)
                              { ++oneRingSize; });

            auto &g = gradOps[v.index];
            g = Eigen::VectorXcd::Zero(oneRingSize + 1);

            Eigen::VectorXd angleCumSum(oneRingSize + 1);
            angleCumSum(0) = angleCumSum(oneRingSize) = 0.0;

            if (v.onBoundary)
            {
                auto he = v.he;
                int i = 0;
                while (he->face != nullptr)
                {
                    angleCumSum(i + 1) =
                        angleCumSum(i) + getVertexAngleInFace(he->face, &v);
                    ++i;
                    he = he->prev->twin;
                }
                he = v.he->twin;
                i = oneRingSize;
                while (he->face != nullptr)
                {
                    angleCumSum(i - 1) =
                        angleCumSum(i) - getVertexAngleInFace(he->face, &v);
                    --i;
                    he = he->next->twin;
                }
            }
            else
            {
                int i = 0;
                v.forEachFace(
                    [&v, &i, &angleCumSum](const Face *f)
                    {
                        angleCumSum(i + 1) =
                            angleCumSum(i) + getVertexAngleInFace(f, &v) *
                                                 std::numbers::pi * 2.0 /
                                                 v.angleSum;
                        ++i;
                    });
            }

            int i = 0;
            v.forEachHalfEdge(
                [oneRingSize, &v, &i, &l, &g, &angleCumSum](const HalfEdge *he)
                {
                    const Face *f = he->face;
                    if (f != nullptr)
                    {
                        int j = v.index == f->indices(0)
                                    ? 0
                                    : (v.index == f->indices(1) ? 1 : 2);

                        double a = l(f->index, (j + 2) % 3);
                        double b = l(f->index, (j + 1) % 3);

                        auto g1 =
                            std::polar(0.5 * b, angleCumSum(i + 1) -
                                                    std::numbers::pi * 0.5);
                        auto g2 = std::polar(
                            0.5 * a, angleCumSum(i) + std::numbers::pi * 0.5);
                        g(0) -= g1 + g2;
                        g(i + 1) += g1;
                        g((i + 1) % oneRingSize + 1) += g2;
                    }
                    ++i;
                });
            g /= v.area * 3.0;
        });

    return gradOps;
}
