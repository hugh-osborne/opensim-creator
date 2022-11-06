#pragma once

#include "src/Maths/AABB.hpp"

#include <glm/vec3.hpp>
#include <nonstd/span.hpp>

#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace osc { struct Line; }

namespace osc
{
    class BVHNode final {
    public:
        static BVHNode leaf(AABB const& bounds, uint32_t primOffset)
        {
            return BVHNode{bounds, primOffset | g_LeafMask};
        }

        static BVHNode node(AABB const& bounds, uint32_t numLhs)
        {
            return BVHNode{bounds, numLhs & ~g_LeafMask};
        }
    private:
        BVHNode(AABB const& bounds_, uint32_t data_) :
            m_Bounds{bounds_},
            m_Data{std::move(data_)}
        {
        }
    public:
        AABB const& getBounds() const
        {
            return m_Bounds;
        }

        bool isLeaf() const
        {
            return (m_Data & g_LeafMask) > 0;
        }

        bool isNode() const
        {
            return !isLeaf();
        }

        uint32_t getNumLhsNodes() const
        {
            return m_Data & ~g_LeafMask;
        }

        void setNumLhsNodes(uint32_t n)
        {
            m_Data = n & ~g_LeafMask;
        }

        uint32_t getFirstPrimOffset() const
        {
            return m_Data & ~g_LeafMask;
        }

    private:
        static constexpr uint32_t g_LeafMask = 0x80000000;
        AABB m_Bounds;  // union of all AABBs below/including this one
        uint32_t m_Data;
    };

    class BVHPrim final {
    public:
        BVHPrim(int32_t id_, AABB const& bounds_) :
            m_ID{std::move(id_)},
            m_Bounds{bounds_}
        {
        }

        int32_t getID() const
        {
            return m_ID;
        }

        AABB const& getBounds() const
        {
            return m_Bounds;
        }

    private:
        int32_t m_ID;
        AABB m_Bounds;
    };

    struct BVHCollision final {
        int primId;
        float distance;
    };

    struct BVH final {
        std::vector<BVHNode> nodes;
        std::vector<BVHPrim> prims;

        void clear();
    };

    // triangle BVHes
    //
    // these are BVHes where prim.id refers to the first index of a triangle

    // prim.id will refer to the index of the first vertex in the triangle
    void BVH_BuildFromIndexedTriangles(BVH&, nonstd::span<glm::vec3 const> verts, nonstd::span<uint16_t const> indices);
    void BVH_BuildFromIndexedTriangles(BVH&, nonstd::span<glm::vec3 const> verts, nonstd::span<uint32_t const> indices);

    // populates `out` with the closest collision along the ray - if there is a collision
    //
    // returns `true` if there was a collision; otherwise, `false` and `out` if left untouched
    bool BVH_GetClosestRayIndexedTriangleCollision(BVH const&, nonstd::span<glm::vec3 const> verts, nonstd::span<uint16_t const> indices, Line const&, BVHCollision* out);
    bool BVH_GetClosestRayIndexedTriangleCollision(BVH const&, nonstd::span<glm::vec3 const> verts, nonstd::span<uint32_t const> indices, Line const&, BVHCollision* out);


    // AABB BVHes
    //
    // these are BVHes where prim.id refers to the index of the AABB the node was built from

    // prim.id will refer to the index of the AABB
    void BVH_BuildFromAABBs(BVH&, AABB const*, size_t naabbs);

    // returns prim.id of the AABB (leaf) that the line intersects, or -1 if no intersection
    //
    // no assumptions about prim.id required here - it's using the BVH's AABBs
    //
    // returns true if at least one collision was found and appended to the output
    bool BVH_GetRayAABBCollisions(BVH const&, Line const&, std::vector<BVHCollision>* appendTo);

    // returns the maximum depth of the given BVH tree
    int32_t BVH_GetMaxDepth(BVH const&);
}
