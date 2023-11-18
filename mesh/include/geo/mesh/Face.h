#ifndef GEO_FACE_H_
#define GEO_FACE_H_

#include <Eigen/Core>

struct Vertex;
struct HalfEdge;

struct Face
{
    Face() = default;

    Face(const Eigen::Vector3i &i)
        : indices(i)
    {
    }

    HalfEdge *he = nullptr; // from v0 to v1

    int index = -1;

    Eigen::Vector3i indices;
    Eigen::Vector3d normal;
    Eigen::Vector3d angles;

    double area;
};

#endif
