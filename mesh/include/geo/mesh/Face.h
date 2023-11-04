#ifndef GEO_FACE_H_
#define GEO_FACE_H_

#include <Eigen/Core>

struct Vertex;
struct HalfEdge;

struct Face
{
    HalfEdge *he; // from v0 to v1

    size_t index;

    Eigen::Vector3i indices;
    Eigen::Vector3d normal;
    Eigen::Vector3d angles;

    double area;
};

#endif
