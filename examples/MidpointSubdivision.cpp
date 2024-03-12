#include <iostream>

#include <geo/mesh/Subdivision.h>
#include <geo/io/IO.h>

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

    std::vector<std::pair<int, int>> newVertToEdge;
    auto ret = midpointSubdivision(mesh, &newVertToEdge);

    Eigen::VectorXd phi2(ret->nV());
    phi2.head(mesh.nV()) = phi;
    for (size_t i = 0; i < ret->nV() - mesh.nV(); ++i)
    {
        phi2(mesh.nV() + i) =
            0.5 * (phi(newVertToEdge[i].first) + phi(newVertToEdge[i].second));
    }
    writeMeshVertexScalarFieldToVtk(argv[2], *ret, phi2);
    return 0;
}
