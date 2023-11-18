#include <iostream>
#include <chrono>

#include <geo/mesh/SubMesh.h>
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

    double threshold = 0.1;

    std::vector<size_t> selectedFaces;
    for (size_t i = 0; i < mesh.faces.size(); ++i)
    {
        const auto &f = mesh.faces[i];
        if (std::abs(phi(f.indices(0))) < threshold &&
            std::abs(phi(f.indices(1))) < threshold &&
            std::abs(phi(f.indices(2))) < threshold)
        {
            selectedFaces.push_back(i);
        }
    }

    Mesh subMesh;
    std::vector<int> vertexIndicesMapping;
    getSubMesh(mesh, selectedFaces, subMesh, &vertexIndicesMapping);

    Eigen::VectorXd subPhi(subMesh.nV());
    for (size_t i = 0; i < mesh.nV(); ++i)
        if (vertexIndicesMapping[i] != -1)
            subPhi(vertexIndicesMapping[i]) = phi(i);

    writeMeshVertexScalarFieldToVtk(argv[2], subMesh, subPhi);

    return 0;
}
