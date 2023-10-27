#ifndef ACC_DYNAMIC_BVH_HPP_
#define ACC_DYNAMIC_BVH_HPP_

#include <limits>
#include <vector>
#include <queue>
#include <stack>
#include <functional>
#include <numeric>
#include <cstdlib>

#include "BVHNode.hpp"

template <typename ObjectType>
class DynamicBVH
{
public:
    DynamicBVH();

    void staticConstruct(const std::vector<BoundingBox> &aabbs,
                         const std::vector<ObjectType *> objs);

    void insert(const BoundingBox &aabb, ObjectType *obj);
    void reserve(size_t n);

    ObjectType *nearestObject(
        const Eigen::Vector3d &p,
        const std::function<double(ObjectType *, Eigen::Vector3d)> &dist) const;

    std::pair<ObjectType *, double>
    rayHit(const Ray &ray,
           const std::function<double(ObjectType *, const Ray &)> &hit) const;

private:
    int staticConstruct(const std::vector<BoundingBox> &aabbs,
                        const std::vector<ObjectType *> objs,
                        std::vector<size_t>::iterator first,
                        std::vector<size_t>::iterator last, int parentIndex);

    double inheritedAreaDiff(BoundingBox aabb, int index);

    void removeAndInsert(int index, const BoundingBox &newAABB);
    void insertLeafAt(int leafIndex, int newParent, const BoundingBox &aabb,
                      ObjectType *obj);
    void refit(int index);

private:
    struct AreaCost
    {
        int index;
        double cost;

        bool operator<(const AreaCost &other) const
        {
            return cost < other.cost;
        }
    };

private:
    std::vector<BVHNode<ObjectType>> m_nodes;
    int m_rootIndex;
};

template <typename ObjectType>
inline DynamicBVH<ObjectType>::DynamicBVH()
    : m_rootIndex(nullIndex)
{
}

template <typename ObjectType>
inline void DynamicBVH<ObjectType>::reserve(size_t n)
{
    m_nodes.reserve(2 * n);
}

template <typename ObjectType>
inline void
DynamicBVH<ObjectType>::staticConstruct(const std::vector<BoundingBox> &aabbs,
                                        const std::vector<ObjectType *> objs)
{
    reserve(aabbs.size());

    std::vector<size_t> indexOrder(aabbs.size());
    std::iota(indexOrder.begin(), indexOrder.end(), 0u);

    staticConstruct(aabbs, objs, indexOrder.begin(), indexOrder.end(),
                    nullIndex);
    m_rootIndex = 0;
}

template <typename ObjectType>
inline int DynamicBVH<ObjectType>::staticConstruct(
    const std::vector<BoundingBox> &aabbs, const std::vector<ObjectType *> objs,
    std::vector<size_t>::iterator first, std::vector<size_t>::iterator last,
    int parentIndex)
{
    if (first == last)
        return nullIndex;

    if (last - first == 1)
    {
        m_nodes.push_back({aabbs[*first], objs[*first], parentIndex});
        return static_cast<int>(m_nodes.size()) - 1;
    }

    auto m = first + (last - first) / 2;
    int dim = rand() % 3;
    std::nth_element(first, m, last,
                     [&aabbs, dim](size_t i, size_t j)
                     {
                         return aabbs[i].lb(dim) + aabbs[i].ub(dim) <
                                aabbs[j].lb(dim) + aabbs[j].ub(dim);
                     });

    BoundingBox b = aabbs[*first];
    for (auto i = first + 1; i != last; ++i)
        b += aabbs[*i];
    m_nodes.push_back(BVHNode<ObjectType>{b, nullptr, parentIndex});
    int index = static_cast<int>(m_nodes.size()) - 1;

    int child1 = staticConstruct(aabbs, objs, first, m, index);
    int child2 = staticConstruct(aabbs, objs, m, last, index);

    m_nodes[index].child1 = child1;
    m_nodes[index].child2 = child2;

    return index;
}

template <typename ObjectType>
inline double DynamicBVH<ObjectType>::inheritedAreaDiff(BoundingBox aabb,
                                                        int index)
{
    aabb = BoundingBox::merge(aabb, m_nodes[index].aabb);
    double area = 0.f;
    index = m_nodes[index].parent;
    while (index != nullIndex)
    {
        aabb = BoundingBox::merge(aabb, m_nodes[index].aabb);
        area += aabb.area() - m_nodes[index].aabb.area();
        index = m_nodes[index].parent;
    }
    return area;
}

template <typename ObjectType>
inline void DynamicBVH<ObjectType>::refit(int index)
{
    while (index != nullIndex)
    {
        int child1 = m_nodes[index].child1;
        int child2 = m_nodes[index].child2;
        m_nodes[index].aabb =
            BoundingBox::merge(m_nodes[child1].aabb, m_nodes[child2].aabb);
        index = m_nodes[index].parent;
    }
}

template <typename ObjectType>
inline void DynamicBVH<ObjectType>::removeAndInsert(int index,
                                                    const BoundingBox &newAABB)
{
    assert(m_nodes[index].child1 == nullIndex &&
           m_nodes[index].child2 == nullIndex && index < m_nodes.size());
    if (m_nodes.size() == 1)
    {
        m_nodes[0].aabb = newAABB;
        return;
    }
    int parent = m_nodes[index].parent;
    int theOtherChild =
        (index == m_nodes[parent].child1 ? m_nodes[parent].child2
                                         : m_nodes[parent].child1);
    if (parent == m_rootIndex)
        m_rootIndex = theOtherChild;
    int grandparent = m_nodes[parent].parent;
    m_nodes[theOtherChild].parent = grandparent;
    if (grandparent != nullIndex)
        (parent == m_nodes[grandparent].child1 ? m_nodes[grandparent].child1
                                               : m_nodes[grandparent].child2) =
            theOtherChild;
    refit(grandparent);

    insertLeafAt(index, parent, newAABB, m_nodes[index].object);
}

template <typename ObjectType>
inline void DynamicBVH<ObjectType>::insertLeafAt(int leafIndex, int newParent,
                                                 const BoundingBox &aabb,
                                                 ObjectType *obj)
{
    m_nodes[leafIndex].aabb = aabb;
    m_nodes[leafIndex].object = obj;

    int sibling = m_rootIndex;
    double minCost = BoundingBox::merge(aabb, m_nodes[m_rootIndex].aabb).area();
    std::priority_queue<AreaCost> q;
    q.push({m_rootIndex, minCost});

    while (!q.empty())
    {
        AreaCost curr = q.top();
        q.pop();
        if (curr.index != m_rootIndex)
        {
            curr.cost =
                BoundingBox::merge(aabb, m_nodes[curr.index].aabb).area() +
                inheritedAreaDiff(aabb, curr.index);
            if (curr.cost < minCost)
            {
                minCost = curr.cost;
                sibling = curr.index;
            }
        }
        if (m_nodes[curr.index].child1 == nullIndex)
        {
            continue;
        }
        double lowerBound =
            aabb.area() + curr.cost - m_nodes[curr.index].aabb.area();
        if (lowerBound < minCost)
        {
            q.push({m_nodes[curr.index].child1, lowerBound});
            q.push({m_nodes[curr.index].child2, lowerBound});
        }
    }

    int oldParent = m_nodes[sibling].parent;
    m_nodes[newParent] = {BoundingBox::merge(aabb, m_nodes[sibling].aabb),
                          nullptr, oldParent};

    if (oldParent != nullIndex)
    {
        (m_nodes[oldParent].child1 == sibling ? m_nodes[oldParent].child1
                                              : m_nodes[oldParent].child2) =
            newParent;
        m_nodes[newParent].child1 = sibling;
        m_nodes[newParent].child2 = leafIndex;
        m_nodes[sibling].parent = m_nodes[leafIndex].parent = newParent;
    }
    else
    {
        m_nodes[newParent].child1 = sibling;
        m_nodes[newParent].child2 = leafIndex;
        m_nodes[sibling].parent = m_nodes[leafIndex].parent = newParent;
        m_rootIndex = newParent;
    }

    refit(m_nodes[leafIndex].parent);
}

template <typename ObjectType>
inline void DynamicBVH<ObjectType>::insert(const BoundingBox &aabb,
                                           ObjectType *obj)
{
    int leafIndex = static_cast<int>(m_nodes.size());
    m_nodes.push_back({aabb, obj});
    if (m_nodes.size() == 1)
    {
        m_rootIndex = leafIndex;
        return;
    }

    int sibling = m_rootIndex;
    double minCost = BoundingBox::merge(aabb, m_nodes[m_rootIndex].aabb).area();
    std::priority_queue<AreaCost> q;
    q.push({m_rootIndex, minCost});

    while (!q.empty())
    {
        AreaCost curr = q.top();
        q.pop();
        if (curr.index != m_rootIndex)
        {
            curr.cost =
                BoundingBox::merge(aabb, m_nodes[curr.index].aabb).area() +
                inheritedAreaDiff(aabb, curr.index);
            if (curr.cost < minCost)
            {
                minCost = curr.cost;
                sibling = curr.index;
            }
        }
        if (m_nodes[curr.index].child1 == nullIndex)
        {
            continue;
        }
        double lowerBound =
            aabb.area() + curr.cost - m_nodes[curr.index].aabb.area();
        if (lowerBound < minCost)
        {
            q.push({m_nodes[curr.index].child1, lowerBound});
            q.push({m_nodes[curr.index].child2, lowerBound});
        }
    }

    int oldParent = m_nodes[sibling].parent;
    int newParent = static_cast<int>(m_nodes.size());
    m_nodes.push_back(
        {BoundingBox::merge(aabb, m_nodes[sibling].aabb), nullptr, oldParent});

    if (oldParent != nullIndex)
    {
        (m_nodes[oldParent].child1 == sibling ? m_nodes[oldParent].child1
                                              : m_nodes[oldParent].child2) =
            newParent;
        m_nodes[newParent].child1 = sibling;
        m_nodes[newParent].child2 = leafIndex;
        m_nodes[sibling].parent = m_nodes[leafIndex].parent = newParent;
    }
    else
    {
        m_nodes[newParent].child1 = sibling;
        m_nodes[newParent].child2 = leafIndex;
        m_nodes[sibling].parent = m_nodes[leafIndex].parent = newParent;
        m_rootIndex = newParent;
    }

    refit(m_nodes[leafIndex].parent);
}

template <typename ObjectType>
inline ObjectType *DynamicBVH<ObjectType>::nearestObject(
    const Eigen::Vector3d &p,
    const std::function<double(ObjectType *, Eigen::Vector3d)> &dist) const
{
    double minDist = std::numeric_limits<double>::max();

    ObjectType *currentNearestObj = nullptr;

    std::stack<int> s;
    s.push(m_rootIndex);

    while (!s.empty())
    {
        int nodeIndex = s.top();
        s.pop();

        if (nodeIndex == nullIndex)
            continue;

        const auto &node = m_nodes[nodeIndex];

        double d = node.aabb.minDist(p);
        if (d >= minDist)
            continue;

        if (node.object == nullptr)
        {
            double d1 = m_nodes[node.child1].aabb.minDist(p);
            double d2 = m_nodes[node.child2].aabb.minDist(p);
            if (d1 < d2)
            {
                s.push(node.child2);
                s.push(node.child1);
            }
            else
            {
                s.push(node.child1);
                s.push(node.child2);
            }
        }
        else
        {
            double d = dist(node.object, p);

            if (d < minDist)
            {
                minDist = d;
                currentNearestObj = node.object;
            }
        }
    }

    return currentNearestObj;
}

template <typename ObjectType>
inline std::pair<ObjectType *, double> DynamicBVH<ObjectType>::rayHit(
    const Ray &ray,
    const std::function<double(ObjectType *, const Ray &)> &hit) const
{
    double minT = std::numeric_limits<double>::max();

    ObjectType *hitObj = nullptr;

    std::stack<int> s;
    s.push(m_rootIndex);

    while (!s.empty())
    {
        int nodeIndex = s.top();
        s.pop();

        if (nodeIndex == nullIndex)
            continue;

        const auto &node = m_nodes[nodeIndex];

        if (node.object == nullptr)
        {
            double t1 = m_nodes[node.child1].aabb.rayHit(ray);
            double t2 = m_nodes[node.child2].aabb.rayHit(ray);

            if (t1 < t2)
            {
                if (t2 < std::numeric_limits<double>::max())
                    s.push(node.child2);
                if (t1 < std::numeric_limits<double>::max())
                    s.push(node.child1);
            }
            else
            {
                if (t1 < std::numeric_limits<double>::max())
                    s.push(node.child1);
                if (t2 < std::numeric_limits<double>::max())
                    s.push(node.child2);
            }
        }
        else
        {
            double t = hit(node.object, ray);
            if (t < minT)
            {
                minT = t;
                hitObj = node.object;
            }
        }
    }

    return {hitObj, minT};
}

#endif
