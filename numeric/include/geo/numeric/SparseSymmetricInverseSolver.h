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
    int nnz() const { return L_.nonZeros(); }

private:
    double getInternal(int i, int j);

private:
    Eigen::SparseMatrix<double> L_;
    Eigen::VectorXd d_;
    Eigen::PermutationMatrix<Eigen::Dynamic> P_;
    std::vector<ColContainer> Z_;
};

template <typename CholType, typename ColContainer>
inline SparseSymmetricInverseSolver<
    CholType, ColContainer>::SparseSymmetricInverseSolver(const CholType &chol)
    : L_(chol.matrixL())
    , d_(chol.vectorD())
    , P_(chol.permutationP())
    , Z_(L_.rows())
{
}

template <typename CholType, typename ColContainer>
inline void SparseSymmetricInverseSolver<CholType, ColContainer>::compute()
{
    int n = L_.rows();
    Z_[n - 1][n - 1] = 1.0 / d_(n - 1);

    for (int i = n - 2; i >= 0; --i)
    {
        for (auto rit = L_.outerIndexPtr()[i + 1] - 1;
             rit >= L_.outerIndexPtr()[i]; --rit)
        {
            int j = L_.innerIndexPtr()[rit];
            double v = i == j ? 1.0 / d_(j) : 0.0;

            Eigen::SparseMatrix<double>::InnerIterator it(L_, i);
            ++it;
            for (; it; ++it)
            {
                int k = it.row();
                v -= it.value() * (j < k ? Z_[j].at(k) : Z_[k].at(j));
            }

            Z_[i].emplace(j, v);
        }
    }
}

template <typename CholType, typename ColContainer>
inline double SparseSymmetricInverseSolver<CholType, ColContainer>::get(int i,
                                                                        int j)
{
    return getInternal(P_.indices().coeff(i), P_.indices().coeff(j));
}

template <typename CholType, typename ColContainer>
inline double
SparseSymmetricInverseSolver<CholType, ColContainer>::getInternal(int i, int j)
{
    if (i > j)
        std::swap(i, j);

    auto it = Z_[i].find(j);
    if (it != Z_[i].end())
        return it->second;

    double v = i == j ? 1.0 / d_(i) : 0.0;
    Eigen::SparseMatrix<double>::InnerIterator itL(L_, i);
    ++itL;
    for (; itL; ++itL)
    {
        int k = itL.row();
        v -= itL.value() * getInternal(j, k);
    }

    Z_[i].emplace(j, v);
    return v;
}

#endif
