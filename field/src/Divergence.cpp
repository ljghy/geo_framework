#include <algorithm>

#include <Eigen/Dense>

#include <geo/field/Divergence.h>

void computeDivergence(Mesh &mesh, const Eigen::Matrix3Xd &X,
                       Eigen::VectorXd &div)
{
    mesh.require(Mesh::FaceAngles | Mesh::VertexAreas);

    div = Eigen::VectorXd::Zero(mesh.nV());
    std::for_each(
        mesh.faces.begin(), mesh.faces.end(),
        [&mesh, &X, &div](const Face &face)
        {
            auto i = mesh.getFaceIndex(&face);

            const auto &I = face.indices;

            Eigen::Vector3d e[3]{
                (mesh.vertices[I(1)].position - mesh.vertices[I(0)].position) /
                    std::tan(face.angles(2)),
                (mesh.vertices[I(2)].position - mesh.vertices[I(1)].position) /
                    std::tan(face.angles(0)),
                (mesh.vertices[I(0)].position - mesh.vertices[I(2)].position) /
                    std::tan(face.angles(1))};

            for (int j = 0; j < 3; ++j)
            {
                div(I(j)) += (e[j] - e[(j + 2) % 3]).dot(X.col(i)) /
                             (2.0 * mesh.vertices[I(j)].area);
            }
        });
}
