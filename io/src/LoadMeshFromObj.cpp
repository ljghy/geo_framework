#include <fstream>
#include <cassert>

#include <geo/io/IO.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include <geo/io/tiny_obj_loader.h>

void loadMeshFromObj(const std::string &filename, Mesh &mesh)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;

    std::string err;

    bool ret = tinyobj::LoadObj(&attrib, &shapes, nullptr, &err,
                                filename.c_str()); // triangulate = true

    if (!ret)
        throw std::runtime_error(err);

    if (shapes.size() == 0)
        throw std::runtime_error("No shape in .obj");

    auto &indices = shapes[0].mesh.indices;

    size_t nV = attrib.vertices.size() / 3;
    mesh.vertices.resize(nV);

    for (size_t i = 0; i < nV; ++i)
    {
        mesh.vertices[i].position(0) = attrib.vertices[3 * i + 0];
        mesh.vertices[i].position(1) = attrib.vertices[3 * i + 1];
        mesh.vertices[i].position(2) = attrib.vertices[3 * i + 2];
    }

    size_t nF = indices.size() / 3;
    mesh.faces.resize(nF);

    for (size_t i = 0; i < nF; ++i)
    {
        mesh.faces[i].indices(0) = indices[3 * i + 0].vertex_index;
        mesh.faces[i].indices(1) = indices[3 * i + 1].vertex_index;
        mesh.faces[i].indices(2) = indices[3 * i + 2].vertex_index;
    }
}
