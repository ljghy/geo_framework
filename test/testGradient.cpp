#include <iostream>
#include <chrono>

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

    std::chrono::high_resolution_clock::time_point t1 =
        std::chrono::high_resolution_clock::now();

    Eigen::Matrix3Xd grad;
    computeGradient(mesh, phi, grad);

    std::chrono::high_resolution_clock::time_point t2 =
        std::chrono::high_resolution_clock::now();
    std::cout << "Computed gradient in "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                     .count()
              << " ms" << std::endl;

    writeMeshFaceVectorFieldToVtk(argv[2], mesh, grad);
}
