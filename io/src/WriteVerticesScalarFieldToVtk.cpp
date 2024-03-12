#include <fstream>
#include <cassert>

#include <geo/io/IO.h>

void writeVerticesScalarFieldToVtk(const std::string &filename,
                                   const std::vector<Eigen::Vector3d> &vertices,
                                   const Eigen::VectorXd &phi)
{
    std::ofstream fout(filename);
    if (!fout)
        throw std::runtime_error("Failed to open file " + filename);

    fout << "# vtk DataFile Version 3.0\n"
         << "VERTICES\n"
         << "ASCII\n"
         << "DATASET UNSTRUCTURED_GRID\n";

    size_t nV = vertices.size();
    fout << "POINTS " << nV << " double\n";
    for (const auto &vert : vertices)
    {
        fout << vert(0) << ' ' << vert(1) << ' ' << vert(2) << '\n';
    }

    fout << "CELLS " << nV << ' ' << nV * 2 << '\n';
    for (size_t i = 0; i < nV; ++i)
    {
        fout << 1 << ' ' << i << '\n';
    }

    fout << "CELL_TYPES " << nV << '\n';
    for (size_t i = 0; i < nV; ++i)
    {
        fout << "1\n";
    }

    fout << "POINT_DATA " << nV << '\n'
         << "SCALARS field double 1\n"
         << "LOOKUP_TABLE default\n";
    for (size_t i = 0; i < nV; ++i)
    {
        fout << phi(i) << '\n';
    }
}
