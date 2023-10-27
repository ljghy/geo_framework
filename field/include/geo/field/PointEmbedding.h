#ifndef GEO_POINT_EMBEDDING_H_
#define GEO_POINT_EMBEDDING_H_

#include <geo/mesh/Face.h>

struct PointEmbedding
{
    const Face *face;
    Eigen::Vector3d barycentric;
};

#endif
