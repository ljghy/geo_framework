#ifndef GEO_VERTEX_H_
#define GEO_VERTEX_H_

#include <functional>

#include <Eigen/Core>

struct HalfEdge;
struct Face;

struct Vertex
{
    Vertex() = default;

    Vertex(const Eigen::Vector3d &p)
        : position(p)
    {
    }

    HalfEdge *he = nullptr;

    int index = -1;

    Eigen::Vector3d position;
    Eigen::Vector3d normal;

    double area;
    double angleSum;

    bool onBoundary;

    void forEachHalfEdge(std::function<void(HalfEdge *)>);
    void forEachHalfEdge(std::function<void(const HalfEdge *)>) const;
    void forEachFace(std::function<void(Face *)>);
    void forEachFace(std::function<void(const Face *)>) const;
};

#endif
