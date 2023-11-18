#include <cassert>
#include <map>
#include <array>
#include <algorithm>

#include <geo/field/TraceZeroLevelSet.h>

static Eigen::VectorXd pushOutSDF(const Mesh &mesh, const Eigen::VectorXd &phi,
                                  double pushOut)
{
    Eigen::VectorXd phiPushedOut = phi;

    for (size_t i = 0; i < mesh.nV(); ++i)
    {
        if (phi(i) != 0.0)
            continue;
        int np = 0, nn = 0;
        auto start = mesh.vertices[i].he, he = start;
        do
        {
            if (phi(he->twin->origin - mesh.vertices.data()) > 0)
                ++np;
            else
                ++nn;
            he = he->twin->next;
        } while (he != start);
        if (np == 0)
            phiPushedOut(i) = -pushOut * 2.0;
        else if (nn == 0)
            phiPushedOut(i) = pushOut * 2.0;
        else if (np >= nn)
            phiPushedOut(i) = pushOut;
        else
            phiPushedOut(i) = -pushOut;
    }
    return phiPushedOut;
}

static std::vector<std::shared_ptr<Curve>>
splitCurves(const std::vector<Eigen::Vector3d> &vertices,
            std::vector<std::array<int, 2>> &indexMapping, double dupThreshold)
{
    std::vector<std::shared_ptr<Curve>> ret;
    while (true)
    {
        int j = -1;
        for (size_t i = 0; i < indexMapping.size(); ++i)
        {
            if (indexMapping[i][0] != -1 || indexMapping[i][1] != -1)
            {
                j = i;
                break;
            }
        }
        if (j == -1)
            break;

        auto &result = ret.emplace_back(new Curve);

        int start = j;
        int p = indexMapping[j][0] == -1;
        int k = indexMapping[j][p];
        indexMapping[j][p] = -1;

        result->vertices.emplace_back(vertices[j]);

        while (k != start && k != -1)
        {
            if ((vertices[k] - vertices[j]).norm() > dupThreshold)
                result->vertices.emplace_back(vertices[k]);

            p = indexMapping[k][0] == j;
            int l = indexMapping[k][p];
            indexMapping[k][0] = indexMapping[k][1] = -1;
            j = k;
            k = l;
        }

        if (k == -1)
        {
            result->isLoop = false;
            std::reverse(result->vertices.begin(), result->vertices.end());
            j = start;
            p = indexMapping[j][0] == -1;
            k = indexMapping[j][p];
            indexMapping[j][p] = -1;
            while (k != -1)
            {
                if ((vertices[k] - vertices[j]).norm() > dupThreshold)
                    result->vertices.emplace_back(vertices[k]);

                p = indexMapping[k][0] == j;
                int l = indexMapping[k][p];
                indexMapping[k][0] = indexMapping[k][1] = -1;
                j = k;
                k = l;
            }
        }
        else
        {
            result->isLoop = true;
            indexMapping[start][0] = indexMapping[start][1] = -1;
        }
    }

    return ret;
}

std::vector<std::shared_ptr<Curve>>
traceZeroLevelSet(Mesh &mesh, const Eigen::VectorXd &phi, double dupThreshold)
{
    mesh.require(Mesh::HalfEdgeStructure);

    assert(phi.size() == static_cast<int>(mesh.nV()));

    double minAbs = std::numeric_limits<double>::max();
    for (size_t i = 0; i < mesh.nV(); ++i)
    {
        if (phi(i) != 0.0)
            minAbs = std::min(minAbs, std::abs(phi(i)));
    }
    if (minAbs == std::numeric_limits<double>::max())
        return {};

    auto phiPushedOut = pushOutSDF(mesh, phi, 1e-2 * minAbs);

    std::map<std::pair<int, int>, int> verticesEdges;
    std::vector<std::array<int, 2>> tracedEdges;
    std::vector<Eigen::Vector3d> vertices;

    for (const auto &f : mesh.faces)
    {
        Eigen::Vector3d v = phiPushedOut(f.indices);
        if ((v(0) > 0 && v(1) > 0 && v(2) > 0) ||
            (v(0) < 0 && v(1) < 0 && v(2) < 0))
            continue;

        int k = 0;
        tracedEdges.emplace_back();
        for (int i = 0; i < 3; ++i)
        {
            int j = (i + 1) % 3;

            int vertexCurveIndex = -1;
            if (v(i) * v(j) < 0)
            {
                double d = v(i) - v(j);
                double t = d != 0.0 ? v(i) / d : 0.5;

                auto p = f.indices(i) < f.indices(j)
                             ? std::make_pair(f.indices(i), f.indices(j))
                             : std::make_pair(f.indices(j), f.indices(i));
                auto it = verticesEdges.find(p);
                if (it == verticesEdges.end())
                {
                    verticesEdges[p] = vertexCurveIndex = vertices.size();
                    vertices.emplace_back(
                        (1.0 - t) * mesh.vertices[f.indices(i)].position +
                        t * mesh.vertices[f.indices(j)].position);
                }
                else
                {
                    vertexCurveIndex = it->second;
                }

                assert(vertexCurveIndex >= 0);
                assert(k < 2);
                tracedEdges.back()[k++] = vertexCurveIndex;
            }
        }
        assert(k == 2);
    }

    std::vector<std::array<int, 2>> indexMapping(vertices.size(), {{-1, -1}});
    for (const auto &e : tracedEdges)
    {
        (indexMapping[e[0]][0] == -1 ? indexMapping[e[0]][0]
                                     : indexMapping[e[0]][1]) = e[1];
        (indexMapping[e[1]][0] == -1 ? indexMapping[e[1]][0]
                                     : indexMapping[e[1]][1]) = e[0];
    }

    return splitCurves(vertices, indexMapping, dupThreshold);
}
