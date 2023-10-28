#include <cassert>

#include <Eigen/SparseCholesky>

#include <geo/field/Seg2SDF.h>
#include <geo/field/Gradient.h>
#include <geo/field/Divergence.h>
#include <geo/field/CotanLaplacian.h>

static void extractSegBoundary(Mesh &mesh, const std::vector<int> &segs,
                               int requiredSeg,
                               std::vector<size_t> &boundaryVertices)
{
    std::vector<int> in(mesh.nV(), 0);
    std::vector<int> out(mesh.nV(), 0);

    boundaryVertices.clear();

    for (size_t i = 0; i < mesh.nF(); ++i)
    {
        const auto &I = mesh.faces[i].indices;
        bool b = segs[i] == requiredSeg;
        for (int j = 0; j < 3; ++j)
            b ? in[I(j)]++ : out[I(j)]++;
    }

    for (size_t i = 0; i < mesh.nV(); ++i)
        if (in[i] > 0 && out[i] > 0)
            boundaryVertices.push_back(i);
}

void seg2SDF(Mesh &mesh, const std::vector<int> &segs, int requiredSeg,
             Eigen::VectorXd &phi)
{
    mesh.require(Mesh::VertexAreas | Mesh::MeanEdgeLength);

    std::vector<size_t> sourceVertices;
    extractSegBoundary(mesh, segs, requiredSeg, sourceVertices);

    Eigen::SparseMatrix<double> L;
    cotanLaplacian(mesh, L);

    double dt = mesh.meanEdgeLength * mesh.meanEdgeLength;
    Eigen::SparseMatrix<double> H = -dt * L;
    Eigen::VectorXd mass = mesh.getVertexMassVector();
    H.diagonal() += mass;

    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(mesh.nV());
    rhs(sourceVertices).array() = 1.0;

    Eigen::SimplicialLDLT<decltype(H)> chol;
    phi = chol.compute(H).solve(rhs);

    Eigen::Matrix3Xd grad;
    computeGradient(mesh, phi, grad);
    grad.colwise().normalize();
    for (size_t i = 0; i < mesh.nF(); ++i)
        if (segs[i] == requiredSeg)
            grad.col(i) = -grad.col(i);

    Eigen::VectorXd div;
    computeDivergence(mesh, grad, div);
    div.array() *= mass.array();

    div(sourceVertices).array() = 0.0;
    L.prune([&rhs](int i, int j, double)
            { return (rhs(i) < 0.5 && rhs(j) < 0.5) || i == j; });
    phi = chol.compute(L).solve(-div);
}
