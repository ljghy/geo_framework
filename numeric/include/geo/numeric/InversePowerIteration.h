#ifndef GEO_INVERSE_POWER_ITERATION_H_
#define GEO_INVERSE_POWER_ITERATION_H_

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>

inline double realPart(double x) { return x; }

inline double realPart(std::complex<double> x) { return x.real(); }

template <unsigned int UpLo = Eigen::Lower, typename ScalarType>
inline void hermitianInversePowerIteration(
    const Eigen::SparseMatrix<ScalarType> &A,
    Eigen::Vector<ScalarType, Eigen::Dynamic> *x = nullptr,
    double *lambda = nullptr,
    const Eigen::Vector<ScalarType, Eigen::Dynamic> *x0 = nullptr,
    int maxIter = 256, double tol = 1e-6)
{
    if (x == nullptr && lambda == nullptr)
        return;

    using Vec = Eigen::Vector<ScalarType, Eigen::Dynamic>;
    Vec u = x0 == nullptr ? Vec::Random(A.rows()) : *x0;

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<ScalarType>, UpLo> chol;
    chol.compute(A);

    double lam = 0.0;
    for (int i = 0; i < maxIter; ++i)
    {
        Vec v = chol.solve(u);
        lam = realPart(u.dot(v));
        double vNorm = v.norm();
        if (std::abs(std::abs(lam) - vNorm) < tol)
            break;
        u = v.normalized();
    }

    if (lambda != nullptr)
        *lambda = 1.0 / lam;
    if (x != nullptr)
        *x = u;
}

#endif
