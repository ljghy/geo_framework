#include <Eigen/SparseCholesky>

#include <geo/numeric/nsqp.h>
#include <geo/field/HeatGeodesicDistance.h>
#include <geo/field/Gradient.h>
#include <geo/field/Divergence.h>
#include <geo/field/CotanLaplacian.h>

void heatGeodesicDistance(Mesh &mesh, const std::vector<size_t> &sourceVertices,
                          Eigen::VectorXd &phi)
{
    mesh.require(Mesh::VertexAreas | Mesh::MeanEdgeLength);

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

    Eigen::VectorXd div;
    computeDivergence(mesh, grad, div);
    div.array() *= mass.array();

    div(sourceVertices).array() = 0.0;
    L.prune([&rhs](int i, int j, double)
            { return (rhs(i) < 0.5 && rhs(j) < 0.5) || i == j; });
    phi = chol.compute(L).solve(-div);
}

void heatGeodesicDistance(Mesh &mesh,
                          const std::vector<PointEmbedding> &sourcePoints,
                          Eigen::VectorXd &phi)
{
    mesh.require(Mesh::VertexAreas | Mesh::MeanEdgeLength);

    Eigen::SparseMatrix<double> L;
    cotanLaplacian(mesh, L);

    double dt = mesh.meanEdgeLength * mesh.meanEdgeLength;
    Eigen::SparseMatrix<double> H = -dt * L;
    Eigen::VectorXd mass = mesh.getVertexMassVector();
    H.diagonal() += mass;

    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(mesh.nV());
    for (const auto &s : sourcePoints)
        rhs(s.face->indices) = s.barycentric;

    Eigen::SimplicialLDLT<decltype(H)> chol;
    phi = chol.compute(H).solve(rhs);

    Eigen::Matrix3Xd grad;
    computeGradient(mesh, phi, grad);
    grad.colwise().normalize();

    Eigen::VectorXd div;
    computeDivergence(mesh, grad, div);
    div.array() *= mass.array();

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(sourcePoints.size() * 3);
    for (size_t i = 0; i < sourcePoints.size(); i++)
    {
        const auto &[f, bc] = sourcePoints[i];
        for (int j = 0; j < 3; j++)
            triplets.emplace_back(i, f->indices(j), bc(j));
    }
    Eigen::SparseMatrix<double, Eigen::RowMajorBit> C(sourcePoints.size(),
                                                      mesh.nV());
    C.setFromTriplets(triplets.begin(), triplets.end());

    phi = nsqp::solve(L.selfadjointView<Eigen::Lower>(), div, C);
}
