#ifndef HALF_EDGE_H_
#define HALF_EDGE_H_

#include <Eigen/Core>

struct Vertex;
struct Face;

struct HalfEdge
{
    Vertex *origin = nullptr;
    HalfEdge *twin = nullptr;
    HalfEdge *prev = nullptr;
    HalfEdge *next = nullptr;
    Face *face = nullptr;

    double connectionAngle;

    Eigen::Vector3d vec() const;
};

#endif
