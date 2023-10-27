#include <fstream>
#include <cassert>

#include <geo/io/IO.h>

void writeCurveToVtk(const std::string &filename,
                     const std::vector<Eigen::Vector3d> &vertices, bool isLoop)
{
    if (vertices.empty())
        return;

    std::ofstream fout(filename);

    if (!fout)
        throw std::runtime_error("Failed to open file " + filename);

    fout << "# vtk DataFile Version 2.0\n"
         << "CURVE\n"
         << "ASCII\n"
         << "DATASET UNSTRUCTURED_GRID\n";

    fout << "POINTS " << vertices.size() << " double\n";
    for (const auto &v : vertices)
    {
        fout << v(0) << ' ' << v(1) << ' ' << v(2) << '\n';
    }

    size_t nC = isLoop ? vertices.size() : (vertices.size() - 1);
    fout << "CELLS " << nC << ' ' << nC * 3 << '\n';
    for (size_t i = 0; i < nC; ++i)
    {
        fout << 2 << ' ' << i << ' ' << (i + 1) % vertices.size() << '\n';
    }

    fout << "CELL_TYPES " << nC << '\n';
    for (size_t i = 0; i < nC; ++i)
    {
        fout << 3 << '\n';
    }
}
