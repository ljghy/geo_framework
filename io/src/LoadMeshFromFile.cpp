#include <string_view>

#include <geo/io/IO.h>

void loadMeshFromFile(const std::string &filename, Mesh &mesh)
{
    std::string_view ext(filename.begin() + filename.find_last_of(".") + 1,
                         filename.end());

    if (ext == "vtk")
        loadMeshFromVtk(filename, mesh);
    else if (ext == "obj")
        loadMeshFromObj(filename, mesh);
    else
        throw std::runtime_error("Unknown file format");
}

void loadMeshFromFile(const std::string &filename, Eigen::Matrix3Xd &V,
                      Eigen::Matrix3Xi &F)
{
    std::string_view ext(filename.begin() + filename.find_last_of(".") + 1,
                         filename.end());

    if (ext == "vtk")
        loadMeshFromVtk(filename, V, F);
    else if (ext == "obj")
        loadMeshFromObj(filename, V, F);
    else
        throw std::runtime_error("Unknown file format");
}
