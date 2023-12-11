#ifndef GEO_SPARSE_SYMMETRIC_INVERSE_SOLVER_H_
#define GEO_SPARSE_SYMMETRIC_INVERSE_SOLVER_H_

#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>

template <typename CholType,
          typename ColContainer = std::unordered_map<int, double>>
class SparseSymmetricInverseSolver
{
public:
    SparseSymmetricInverseSolver(const CholType &chol);

    void compute();
    double get(int i, int j);
    int nnz() const { return L.nonZeros(); }

private:
    double getInternal(int i, int j);

private:
    Eigen::SparseMatrix<double> L;
    Eigen::VectorXd d;
    Eigen::PermutationMatrix<Eigen::Dynamic> P;
    std::vector<ColContainer> Z;
};

template <typename CholType, typename ColContainer>
inline SparseSymmetricInverseSolver<
    CholType, ColContainer>::SparseSymmetricInverseSolver(const CholType &chol)
    : L(chol.matrixL())
    , d(chol.vectorD())
    , P(chol.permutationP())
    , Z(L.rows())
{
}

template <typename CholType, typename ColContainer>
inline void SparseSymmetricInverseSolver<CholType, ColContainer>::compute()
{
    int n = L.rows();
    Z[n - 1][n - 1] = 1.0 / d(n - 1);

    for (int i = n - 2; i >= 0; --i)
    {
        std::vector<int> colNonZeros;
        colNonZeros.reserve(L.col(i).nonZeros());
        for (Eigen::SparseMatrix<double>::InnerIterator it(L, i); it; ++it)
        {
            colNonZeros.emplace_back(it.row());
        }

        for (auto rit = colNonZeros.crbegin(); rit != colNonZeros.crend();
             ++rit)
        {
            int j = *rit;
            double v = i == j ? 1.0 / d(j) : 0.0;

            Eigen::SparseMatrix<double>::InnerIterator it(L, i);
            ++it;
            for (; it; ++it)
            {
                int k = it.row();
                v -= it.value() * (j < k ? Z[j].at(k) : Z[k].at(j));
            }

            Z[i].emplace(j, v);
        }
    }
}

template <typename CholType, typename ColContainer>
inline double SparseSymmetricInverseSolver<CholType, ColContainer>::get(int i,
                                                                        int j)
{
    i = P.indices().coeff(i);
    j = P.indices().coeff(j);

    return getInternal(i, j);
}

template <typename CholType, typename ColContainer>
inline double
SparseSymmetricInverseSolver<CholType, ColContainer>::getInternal(int i, int j)
{
    if (i > j)
        std::swap(i, j);

    auto it = Z[i].find(j);
    if (it != Z[i].end())
        return it->second;

    double v = i == j ? 1.0 / d(i) : 0.0;
    Eigen::SparseMatrix<double>::InnerIterator itL(L, i);
    ++itL;
    for (; itL; ++itL)
    {
        int k = itL.row();
        v -= itL.value() * getInternal(j, k);
    }

    Z[i].emplace(j, v);
    return v;
}

#endif
