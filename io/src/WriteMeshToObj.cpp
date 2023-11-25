#include <fstream>

#include <geo/io/IO.h>

void writeMeshToObj(const std::string &filename, const Mesh &mesh)
{
    std::ofstream fout(filename);
    if (!fout)
        throw std::runtime_error("Failed to open file " + filename);

    for (const auto &v : mesh.vertices)
    {
        fout << "v " << v.position(0) << " " << v.position(1) << " "
             << v.position(2) << std::endl;
    }

    for (const auto &f : mesh.faces)
    {
        fout << "f " << f.indices(0) + 1 << " " << f.indices(1) + 1 << " "
             << f.indices(2) + 1 << std::endl;
    }
}
