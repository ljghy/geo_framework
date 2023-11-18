#include <iostream>

#include <geo/io/IO.h>
#include <geo/field/TraceZeroLevelSet.h>

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cout << "Usage: " << argv[0] << " mesh.vtk curve.vtk" << std::endl;
        return 0;
    }

    Eigen::VectorXd phi;
    Mesh mesh;
    loadMeshVertexScalarFieldFromVtk(argv[1], mesh, phi);

    auto result = traceZeroLevelSet(mesh, phi);
    std::cout << "Components: " << result.size() << std::endl;
    for (const auto &r : result)
    {
        std::cout << "Loop: " << r->isLoop
                  << ", Vertices: " << r->vertices.size() << std::endl;
    }
    if (!result.empty())
    {
        int idx = 0;
        writeCurveToVtk(argv[2], result[idx]->vertices, result[idx]->isLoop);
    }
}
