#include <fstream>
#include <cassert>

#include <geo/io/IO.h>

void writeMeshFaceScalarFieldToVtk(const std::string &filename,
                                   const Mesh &mesh, const Eigen::VectorXd &phi)
{
    std::ofstream fout(filename);

    if (!fout.is_open())
        throw std::runtime_error("Failed to open file " + filename);

    fout << "# vtk DataFile Version 3.0\n"
         << "TRIMESH\n"
         << "ASCII\n"
         << "DATASET UNSTRUCTURED_GRID\n";

    fout << "POINTS " << mesh.nV() << " double\n";
    for (const auto &vert : mesh.vertices)
    {
        fout << vert.position(0) << ' ' << vert.position(1) << ' '
             << vert.position(2) << '\n';
    }

    fout << "CELLS " << mesh.nF() << ' ' << mesh.nF() * 4 << '\n';
    for (const auto &face : mesh.faces)
    {
        fout << 3 << ' ' << face.indices(0) << ' ' << face.indices(1) << ' '
             << face.indices(2) << '\n';
    }

    fout << "CELL_TYPES " << mesh.nF() << '\n';
    for (size_t i = 0; i < mesh.nF(); ++i)
    {
        fout << "5\n";
    }

    fout << "CELL_DATA " << mesh.nF() << '\n'
         << "SCALARS field double\n"
         << "LOOKUP_TABLE default\n";
    for (size_t i = 0; i < mesh.nF(); ++i)
    {
        fout << phi(i) << '\n';
    }
}
