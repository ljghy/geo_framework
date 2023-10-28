#ifndef GEO_HEAT_GEODESIC_DISTANCE_H_
#define GEO_HEAT_GEODESIC_DISTANCE_H_

#include <vector>

#include <geo/mesh/Mesh.h>
#include <geo/field/PointEmbedding.h>

void heatGeodesicDistance(Mesh &mesh, const std::vector<size_t> &sourceVertices,
                          Eigen::VectorXd &phi);

void heatGeodesicDistance(Mesh &mesh,
                          const std::vector<PointEmbedding> &sourcePoints,
                          Eigen::VectorXd &phi);

#endif
