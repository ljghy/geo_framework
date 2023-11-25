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

void loadMeshFromVtk(const std::string &filename, Eigen::Matrix3Xd &V,
                     Eigen::Matrix3Xi &F)
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

    V.resize(3, nV);
    for (int i = 0; i < nV; ++i)
    {
        fin >> V(0, i) >> V(1, i) >> V(2, i);
    }

    int nF, nI;
    fin >> buffer >> nF >> nI;
    assert(nI == 4 * nF);

    F.resize(3, nF);
    for (int i = 0; i < nF; ++i)
    {
        fin >> nI >> F(0, i) >> F(1, i) >> F(2, i);
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
