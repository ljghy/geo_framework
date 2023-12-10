#include <geo/field/FaceMetric.h>

std::vector<FaceMetric> computeFaceMetrics(const Mesh &mesh)
{
    std::vector<FaceMetric> faceMetrics(mesh.nF());
    for (size_t i = 0; i < mesh.nF(); ++i)
    {
        const auto &f = mesh.faces[i];
        const auto &v0 = mesh.vertices[f.indices(0)].position;
        const auto &v1 = mesh.vertices[f.indices(1)].position;
        const auto &v2 = mesh.vertices[f.indices(2)].position;
        Eigen::Vector3d e1 = v1 - v0;
        Eigen::Vector3d e2 = v2 - v0;
        double a = e1.norm();
        double b = e2.norm();
        double c = std::clamp(e1.dot(e2) / (a * b), -1.0, 1.0);
        double s = std::sqrt(1.0 - c * c);
        faceMetrics[i].U << a, b * c, 0, b * s;
    }
    return faceMetrics;
}

std::vector<FaceMetric> computeIntrinsicFaceMetrics(const Mesh &mesh,
                                                    const Eigen::MatrixX3d &l)
{
    std::vector<FaceMetric> faceMetrics(mesh.nF());
    for (size_t i = 0; i < mesh.nF(); ++i)
    {
        double a = l(i, 2);
        double b = l(i, 1);
        double c = std::clamp(
            (a * a + b * b - l(i, 0) * l(i, 0)) / (2.0 * a * b), -1.0, 1.0);
        double s = std::sqrt(1.0 - c * c);
        faceMetrics[i].U << a, b * c, 0, b * s;
    }
    return faceMetrics;
}
