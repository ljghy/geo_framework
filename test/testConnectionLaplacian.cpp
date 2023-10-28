#include <iostream>
#include <chrono>

#include <Eigen/Geometry>

#include <geo/io/IO.h>
#include <geo/field/ConnectionLaplacian.h>
#include <geo/numeric/InversePowerIteration.h>

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cout << "Usage: " << argv[0] << " mesh.vtk output.vtk"
                  << std::endl;
        return 0;
    }

    Mesh mesh;
    loadMeshFromVtk(argv[1], mesh);

    std::chrono::high_resolution_clock::time_point t1 =
        std::chrono::high_resolution_clock::now();
    int n = 1;
    Eigen::SparseMatrix<std::complex<double>> L;
    connectionLaplacian(mesh, L, n);

    Eigen::VectorXcd phase;
    hermitianInversePowerIteration(L, &phase);

    Eigen::Matrix3Xd dir(3, mesh.nV());
    for (size_t i = 0; i < mesh.nV(); ++i)
    {
        const auto &v = mesh.vertices[i];
        Eigen::Vector3d b =
            (v.he->vec() - v.he->vec().dot(v.normal) * v.normal).normalized();
        double arg = std::arg(phase(i)) / n;
        dir.col(i) = Eigen::AngleAxisd(arg, v.normal) * b;
    }

    std::chrono::high_resolution_clock::time_point t2 =
        std::chrono::high_resolution_clock::now();
    std::cout << "Computed direction field in "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                     .count()
              << " ms" << std::endl;

    writeMeshVertexVectorFieldToVtk(argv[2], mesh, dir);
}
