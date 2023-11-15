#include <geo/field/CotanVectors.h>

std::vector<Eigen::VectorXd> cotanVectors(Mesh &mesh)
{
    mesh.require(Mesh::HalfEdgeStructure | Mesh::FaceAngles |
                 Mesh::VertexIndices);

    std::vector<Eigen::VectorXd> cots(mesh.nV());

    for (const auto &vert : mesh.vertices)
    {
        int oneRingSize = 0;
        vert.forEachHalfEdge([&oneRingSize](const HalfEdge *)
                             { ++oneRingSize; });
        cots[vert.index] = Eigen::VectorXd::Zero(oneRingSize + 1);
    }
    for (const auto &vert : mesh.vertices)
    {
        int i = vert.index;
        int j = 0;
        vert.forEachHalfEdge(
            [&cots, i, &j](const HalfEdge *he)
            {
                ++j;
                const Face *f1 = he->face;
                if (f1 != nullptr)
                {
                    int k =
                        f1->indices(0) == i ? 0 : (f1->indices(1) == i ? 1 : 2);
                    cots[i](j) += 0.5 / std::tan(f1->angles[(k + 2) % 3]);
                }
                const Face *f2 = he->twin->face;
                if (f2 != nullptr)
                {
                    int k =
                        f2->indices(0) == i ? 0 : (f2->indices(1) == i ? 1 : 2);
                    cots[i](j) += 0.5 / std::tan(f2->angles[(k + 2) % 3]);
                }
                cots[i](0) -= cots[i](j);
            });
    }
    return cots;
}
