#include <algorithm>

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
