#ifndef GEO_CONNECTION_LAPLACIAN_H_
#define GEO_CONNECTION_LAPLACIAN_H_

#include <cassert>
#include <cmath>
#include <algorithm>
#include <complex>

#include <Eigen/SparseCore>

#include <geo/mesh/Mesh.h>

template <bool Lower = true, typename SparseType>
inline void connectionLaplacian(Mesh &mesh, SparseType &L, int n = 1)
{
    assert(n >= 1);

    mesh.require(Mesh::FaceAngles | Mesh::ConnectionAngles);

    std::vector<Eigen::Triplet<std::complex<double>>> triplets;
    triplets.resize(6 * mesh.nF());

    std::for_each(
        mesh.faces.begin(), mesh.faces.end(),
        [&mesh, &triplets, n](const Face &face)
        {
            double cot[3]{1.0 / std::tan(face.angles(0)),
                          1.0 / std::tan(face.angles(1)),
                          1.0 / std::tan(face.angles(2))};

            const auto &I = face.indices;

            const HalfEdge *he[3]{
                I(1) > I(2) ? face.he->next : face.he->next->twin,
                I(2) > I(0) ? face.he->prev : face.he->prev->twin,
                I(0) > I(1) ? face.he : face.he->twin};

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
                    else if ((Lower && I(j) > I(k)) || (!Lower && I(j) < I(k)))
                    {
                        int l = 3 - j - k;
                        std::complex<double> r =
                            std::polar(1.0, -he[l]->connectionAngle * n);
                        triplets[6 * offset + (i++)] = {I(j), I(k),
                                                        0.5 * cot[l] * r};
                    }
        });

    L.resize(mesh.nV(), mesh.nV());
    L.setFromTriplets(triplets.begin(), triplets.end());
}

#endif
