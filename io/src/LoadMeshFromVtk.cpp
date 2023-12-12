#include <fstream>
#include <cassert>

#include <geo/io/IO.h>

void loadMeshFromVtk(const std::string &filename, Mesh &mesh)
{
    std::ifstream fin(filename);
    if (!fin)
        throw std::runtime_error("Failed to open file " + filename);

    std::string buffer;
    std::getline(fin, buffer); // version
    std::getline(fin, buffer); // header
    std::getline(fin, buffer); // file format
    std::getline(fin, buffer); // dataset structure "UNSTRUCTURED_GRID"

    int nV;
    fin >> buffer >> nV >> buffer;

    mesh.vertices.resize(nV);
    for (auto &v : mesh.vertices)
    {
        fin >> v.position(0) >> v.position(1) >> v.position(2);
    }

    int nF, nI;
    fin >> buffer >> nF >> nI;
    assert(nI == 4 * nF);

    mesh.faces.resize(nF);
    for (auto &f : mesh.faces)
    {
        fin >> nI >> f.indices(0) >> f.indices(1) >> f.indices(2);
        assert(nI == 3);
    }

    int nCT, cellType;
    fin >> buffer >> nCT;
    for (int i = 0; i < nCT; ++i)
    {
        fin >> cellType;
        assert(cellType == 5);
    }
}

void loadMeshFromVtk(const std::string &filename, Eigen::MatrixX3d &V,
                     Eigen::MatrixX3i &F)
{
    std::ifstream fin(filename);
    if (!fin)
        throw std::runtime_error("Failed to open file " + filename);

    std::string buffer;
    std::getline(fin, buffer); // version
    std::getline(fin, buffer); // header
    std::getline(fin, buffer); // file format
    std::getline(fin, buffer); // dataset structure "UNSTRUCTURED_GRID"

    int nV;
    fin >> buffer >> nV >> buffer;

    V.resize(nV, 3);
    for (int i = 0; i < nV; ++i)
    {
        fin >> V(i, 0) >> V(i, 1) >> V(i, 2);
    }

    int nF, nI;
    fin >> buffer >> nF >> nI;
    assert(nI == 4 * nF);

    F.resize(nF, 3);
    for (int i = 0; i < nF; ++i)
    {
        fin >> nI >> F(i, 0) >> F(i, 1) >> F(i, 2);
        assert(nI == 3);
    }

    int nCT, cellType;
    fin >> buffer >> nCT;
    for (int i = 0; i < nCT; ++i)
    {
        fin >> cellType;
        assert(cellType == 5);
    }
}
