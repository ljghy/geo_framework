#include <iostream>
#include <chrono>

#include <geo/io/IO.h>
#include <geo/field/HeatGeodesicDistance.h>

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

    Eigen::VectorXd phi;
    heatGeodesicDistance(mesh,
                                {{&mesh.faces[0], {0.5, 0.25, 0.25}},
                                 {&mesh.faces[100], {0.3, 0.3, 0.4}}},
                                phi);

    std::chrono::high_resolution_clock::time_point t2 =
        std::chrono::high_resolution_clock::now();
    std::cout << "Computed heat geodesic distance in "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                     .count()
              << " ms" << std::endl;

    writeMeshVertexScalarFieldToVtk(argv[2], mesh, phi);

    return 0;
}
