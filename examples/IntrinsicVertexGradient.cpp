#include <iostream>

#include <Eigen/Geometry>

#include <geo/io/IO.h>
#include <geo/field/Gradient.h>
#include <geo/field/VertexFaceConnectionAngle.h>

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        std::cout << "Usage: " << argv[0]
                  << " mesh.vtk vertex_grad_output.vtk face_grad_output.vtk"
                  << std::endl;
        return 0;
    }

    Mesh mesh;
    Eigen::VectorXd phi;
    loadMeshVertexScalarFieldFromVtk(argv[1], mesh, phi);

    auto l = mesh.getEdgeLengthMatrix();
    auto oneRing = mesh.getVertexOneRingWithCenter();
    auto oneRingMap = mesh.getVertexOneRingMapWithCenter();
    auto gradOps = intrinsicVertexGradientOperator(mesh, l);

    Eigen::VectorXcd gradPhi(mesh.nV());
    for (size_t i = 0; i < mesh.nV(); ++i)
    {
        gradPhi(i) = gradOps[i].transpose() * phi(oneRing[i]);
    }

    mesh.require(Mesh::VertexNormals | Mesh::HalfEdgeStructure |
                 Mesh::VertexAngleSums);
    Eigen::Matrix3Xd gradPhiExtrinsic(3, mesh.nV());
    for (size_t i = 0; i < mesh.nV(); ++i)
    {
        const auto &v = mesh.vertices[i];
        Eigen::Vector3d b =
            (v.he->vec() - v.he->vec().dot(v.normal) * v.normal).normalized();
        double arg = std::arg(gradPhi(i));
        gradPhiExtrinsic.col(i) =
            Eigen::AngleAxisd(arg, v.normal) * b * std::abs(gradPhi(i));
    }
    writeMeshVertexVectorFieldToVtk(argv[2], mesh, gradPhiExtrinsic);

    mesh.require(Mesh::FaceNormals);

    auto connectionAngles = vertexFaceConnectionAngle(mesh);
    Eigen::Matrix3Xd gradPhiFace(3, mesh.nF());
    for (size_t i = 0; i < mesh.nF(); ++i)
    {
        const auto &f = mesh.faces[i];
        std::complex<double> g{};

        for (size_t j = 0; j < 3; ++j)
        {
            g += std::polar(1.0, -connectionAngles(i, j)) *
                 gradPhi(f.indices(j));
        }
        g /= 3.0;
        double arg = std::arg(g);
        gradPhiFace.col(i) = Eigen::AngleAxisd(arg, f.normal) *
                             f.he->vec().normalized() * std::abs(g);
    }
    writeMeshFaceVectorFieldToVtk(argv[3], mesh, gradPhiFace);

    return 0;
}
