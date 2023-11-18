#include <algorithm>
#include <iostream>
#include <chrono>

#include <geo/mesh/Components.h>
#include <geo/io/IO.h>

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cout << "Usage: " << argv[0] << " mesh output.vtk" << std::endl;
        return 0;
    }

    Mesh mesh;
    loadMeshFromFile(argv[1], mesh);

    auto components = getConnectedComponents(mesh);

    for (const auto &component : components)
    {
        std::cout << "Component with " << component->nV() << " vertices and "
                  << component->nF() << " faces" << std::endl;
    }

    auto largestComponent = *std::max_element(
        components.begin(), components.end(),
        [](const std::shared_ptr<Mesh> &a, const std::shared_ptr<Mesh> &b)
        { return a->nV() < b->nV(); });

    writeMeshToVtk(argv[2], *largestComponent);

    return 0;
}
