#include <iostream>

#include <Eigen/Geometry>

#include <geo/io/IO.h>
#include <geo/field/Gradient.h>

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cout << "Usage: " << argv[0] << " mesh.vtk output.vtk"
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

    Eigen::Matrix3Xd gradPhi(3, mesh.nV());

    mesh.require(Mesh::VertexNormals | Mesh::HalfEdgeStructure |
                 Mesh::VertexAngleSums);
    for (size_t i = 0; i < mesh.nV(); ++i)
    {
        std::complex<double> g = gradOps[i].transpose() * phi(oneRing[i]);
        const auto &v = mesh.vertices[i];
        Eigen::Vector3d b =
            (v.he->vec() - v.he->vec().dot(v.normal) * v.normal).normalized();
        double arg = std::arg(g);
        gradPhi.col(i) = Eigen::AngleAxisd(arg, v.normal) * b * std::abs(g);
    }

    writeMeshVertexVectorFieldToVtk(argv[2], mesh, gradPhi);

    return 0;
}
