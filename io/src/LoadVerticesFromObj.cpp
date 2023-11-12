#include <fstream>
#include <cassert>

#include <geo/io/IO.h>

void loadVerticesFromObj(const std::string &filename,
                         Eigen::Matrix3Xd &vertices)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;

    std::string err;

    bool ret =
        tinyobj::LoadObj(&attrib, &shapes, nullptr, &err, filename.c_str());

    if (!ret)
        throw std::runtime_error(err);

    size_t nV = attrib.vertices.size() / 3;
    vertices.resize(3, nV);

    for (size_t i = 0; i < nV; ++i)
    {
        for (int j = 0; j < 3; ++j)
            vertices(j, i) = attrib.vertices[3 * i + j];
    }
}
