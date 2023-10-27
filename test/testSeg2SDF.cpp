#include <iostream>
#include <chrono>
#include <fstream>
#include <sstream>

#include <geo/io/IO.h>
#include <geo/field/Seg2SDF.h>

int main(int argc, char **argv)
{
    if (argc != 5)
    {
        std::cout << "Usage: " << argv[0]
                  << " mesh.vtk seg.seg required_seg output.vtk" << std::endl;
        return 0;
    }

    Mesh mesh;
    loadMeshFromVtk(argv[1], mesh);

    std::ifstream fin(argv[2]);
    if (!fin)
    {
        std::cerr << "Failed to open file " << argv[2] << '\n';
        return 1;
    }

    std::vector<int> segs(mesh.nF());
    for (size_t i = 0; i < mesh.nF(); ++i)
        fin >> segs[i];

    std::stringstream ss(argv[3]);
    int requiredSeg;
    ss >> requiredSeg;

    std::chrono::high_resolution_clock::time_point t1 =
        std::chrono::high_resolution_clock::now();

    Eigen::VectorXd phi;
    seg2SDF(mesh, segs, requiredSeg, phi);

    std::chrono::high_resolution_clock::time_point t2 =
        std::chrono::high_resolution_clock::now();
    std::cout << "Converted mesh segmentation to SDF in "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                     .count()
              << " ms" << std::endl;

    writeMeshVertexScalarFieldToVtk(argv[4], mesh, phi);

    return 0;
}
