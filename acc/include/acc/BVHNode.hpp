#ifndef ACC_BVH_NODE_HPP_
#define ACC_BVH_NODE_HPP_

#include "BoundingBox.hpp"

inline constexpr int nullIndex = -1;

template <typename ObjectType>
struct BVHNode
{
    BoundingBox aabb;

    ObjectType *object = nullptr;

    int parent = nullIndex;
    int child1 = nullIndex;
    int child2 = nullIndex;
};

#endif
