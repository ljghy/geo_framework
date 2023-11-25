#include <geo/io/IO.h>

void writeMeshToFile(const std::string &filename, const Mesh &mesh)
{

    std::string_view ext(filename.begin() + filename.find_last_of(".") + 1,
                         filename.end());

    if (ext == "vtk")
        writeMeshToVtk(filename, mesh);
    else if (ext == "obj")
        writeMeshToObj(filename, mesh);
    else
        throw std::runtime_error("Unknown file format");
}
