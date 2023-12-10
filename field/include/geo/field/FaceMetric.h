#ifndef GEO_FACE_METRIC_H_
#define GEO_FACE_METRIC_H_

#include <geo/mesh/Mesh.h>

#include <Eigen/Core>

// For computing the inner product of vectors in the tangent space of a face
// The tangent space is standard simplex in R^2
// v0 -> (0, 0), v1 -> (1, 0), v2 -> (0, 1)
struct FaceMetric
{
    // a = ||v1 - v0||, b = ||v2 - v0||
    // theta = angle between v1 - v0 and v2 - v0
    // U = [ a  bcos(theta)]
    //     [ 0  bsin(theta)]
    Eigen::Matrix2d U;

    double squaredNorm(Eigen::Vector2d v) const
    {
        v = U * v;
        return v.squaredNorm();
    }

    double norm(Eigen::Vector2d v) const { return std::sqrt(squaredNorm(v)); }

    double dot(Eigen::Vector2d v1, Eigen::Vector2d v2) const
    {
        v1 = U * v1;
        v2 = U * v2;
        return v1.dot(v2);
    }

    Eigen::Vector2d invU(Eigen::Vector2d v) const
    {
        v(1) /= U(1, 1);
        v(0) = (v(0) - U(0, 1) * v(1)) / U(0, 0);
        return v;
    }
};

std::vector<FaceMetric> computeFaceMetrics(const Mesh &mesh);

std::vector<FaceMetric> computeIntrinsicFaceMetrics(const Mesh &mesh,
                                                    const Eigen::MatrixX3d &l);
#endif
