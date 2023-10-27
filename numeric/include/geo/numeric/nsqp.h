#ifndef GEO_NSQP_H_
#define GEO_NSQP_H_

#include <cassert>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

namespace nsqp
{

/**
 * @brief Solves quadratic programming problem
 *          min x' H x / 2 + c' x
 *          s.t. A x = 0
 *        with a CG method variant proposed in doi:10.1137/S1064827598345667
 *
 * @param H       n x n sparse symmetric positive definite matrix
 * @param c       n x 1 vector
 * @param A       m x n sparse matrix, m < n, better be row-major
 * @param mu      regularization for AA', AA' <- AA' + mu * I
 * @param eps     tolerance, stop if r' g < eps
 * @param maxIter maximum number of iterations
 *
 * @return minimizer x
 */
template <typename Scalar, typename HessianType, typename AType>
inline Eigen::Vector<Scalar, Eigen::Dynamic>
solve(const HessianType &H, const Eigen::Vector<Scalar, Eigen::Dynamic> &c,
      const AType &A, Scalar mu = 1e-10, Scalar eps = 1e-10, int maxIter = 1000)
{
    const size_t n = H.rows();
    const size_t m = A.rows();

    assert(n == H.cols());
    assert(n == c.rows());
    assert(n == A.cols());
    assert(m < n);

    Eigen::SparseMatrix<Scalar> AAT = A * A.transpose();
    for (size_t i = 0; i < m; ++i)
        AAT.coeffRef(i, i) += mu;
    AAT.prune([](size_t, size_t, Scalar v) { return std::abs(v) > Scalar{0}; });

    Eigen::SimplicialLLT<decltype(AAT)> chol;
    chol.analyzePattern(AAT);
    chol.compute(AAT);

    using Vec = Eigen::Vector<Scalar, Eigen::Dynamic>;

    auto P = [&chol, &A](const Vec &r) -> Vec
    { return r - A.transpose() * chol.solve(A * r); };

    Vec x = Vec::Zero(n);

    Vec r = P(c);
    Vec g = P(r);

    Vec p = -g;
    Vec Hp(n);

    int iter = 0;
    Scalar rho = r.dot(g);

    while (rho > eps && iter++ < maxIter)
    {
        Hp = H * p;

        Scalar alpha = rho / p.dot(Hp);
        x += alpha * p;
        r += alpha * Hp;

        g = P(r);
        g = P(g);

        Scalar rhoNew = r.dot(g);
        Scalar beta = rhoNew / rho;
        rho = rhoNew;
        p = -g + beta * p;
        r = g;
    }

    return x;
}

} // namespace nsqp

#endif
