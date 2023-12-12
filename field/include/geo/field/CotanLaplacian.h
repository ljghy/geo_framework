#ifndef GEO_COTAN_LAPLACIAN_H_
#define GEO_COTAN_LAPLACIAN_H_

#include <algorithm>

#include <Eigen/SparseCore>

#include <geo/mesh/Mesh.h>

template <unsigned int UpLo = Eigen::Lower, typename SparseType>
inline void cotanLaplacian(Mesh &mesh, SparseType &L)
{
    constexpr bool lo = UpLo == Eigen::Lower;

    mesh.require(Mesh::FaceAngles);

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.resize(6 * mesh.nF());

    std::for_each(
        mesh.faces.begin(), mesh.faces.end(),
        [&mesh, &triplets](const Face &face)
        {
            double cot[3]{1.0 / std::tan(face.angles(0)),
                          1.0 / std::tan(face.angles(1)),
                          1.0 / std::tan(face.angles(2))};

            const auto &I = face.indices;

            size_t offset = mesh.getFaceIndex(&face);
            int i = 0;
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    if (j == k)
                    {
                        triplets[6 * offset + (i++)] = {
                            I(j), I(j),
                            -0.5 * (cot[(j + 1) % 3] + cot[(j + 2) % 3])};
                    }
                    else if ((lo && I(j) > I(k)) || (!lo && I(j) < I(k)))
                    {
                        int l = 3 - j - k;
                        triplets[6 * offset + (i++)] = {I(j), I(k),
                                                        0.5 * cot[l]};
                    }
        });

    L.resize(mesh.nV(), mesh.nV());
    L.setFromTriplets(triplets.begin(), triplets.end());
}

template <unsigned int UpLo = Eigen::Lower, typename SparseType>
inline void intrinsicCotanLaplacian(Mesh &mesh, const Eigen::MatrixX3d &l,
                                    SparseType &L)
{
    constexpr bool lo = UpLo == Eigen::Lower;

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.resize(6 * mesh.nF());

    std::for_each(
        mesh.faces.begin(), mesh.faces.end(),
        [&mesh, &triplets, &l](const Face &face)
        {
            // double cot[3]{1.0 / std::tan(face.angles(0)),
            //               1.0 / std::tan(face.angles(1)),
            //               1.0 / std::tan(face.angles(2))};

            auto fi = mesh.getFaceIndex(&face);
            double s[3]{l(fi, 0) * l(fi, 0), l(fi, 1) * l(fi, 1),
                        l(fi, 2) * l(fi, 2)};
            double cot[3];
            for (int j = 0; j < 3; ++j)
            {
                int j1 = (j + 1) % 3;
                int j2 = (j + 2) % 3;
                double c = std::clamp((s[j1] + s[j2] - s[j]) /
                                          (2.0 * l(fi, j1) * l(fi, j2)),
                                      -1.0, 1.0);
                cot[j] = c / std::sqrt(1.0 - c * c);
            }

            const auto &I = face.indices;

            size_t offset = mesh.getFaceIndex(&face);
            int i = 0;
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    if (j == k)
                    {
                        triplets[6 * offset + (i++)] = {
                            I(j), I(j),
                            -0.5 * (cot[(j + 1) % 3] + cot[(j + 2) % 3])};
                    }
                    else if ((lo && I(j) > I(k)) || (!lo && I(j) < I(k)))
                    {
                        int l = 3 - j - k;
                        triplets[6 * offset + (i++)] = {I(j), I(k),
                                                        0.5 * cot[l]};
                    }
        });

    L.resize(mesh.nV(), mesh.nV());
    L.setFromTriplets(triplets.begin(), triplets.end());
}

#endif
