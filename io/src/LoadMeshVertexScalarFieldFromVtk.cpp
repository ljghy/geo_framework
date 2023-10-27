#include <fstream>
#include <cassert>

#include <geo/io/IO.h>

void loadMeshVertexScalarFieldFromVtk(const std::string &filename, Mesh &mesh,
                                      Eigen::VectorXd &phi)
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

    int nD;
    fin >> buffer >> nD;
    std::getline(fin, buffer); // '\n'
    std::getline(fin, buffer); // "SCALARS field double 1"
    std::getline(fin, buffer); // "LOOKUP_TABLE default"

    assert(nD == nV);
    phi.resize(nV);
    for (int i = 0; i < nV; ++i)
    {
        fin >> phi(i);
    }
}
