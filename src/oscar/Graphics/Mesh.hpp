#pragma once

#include <oscar/Graphics/MeshIndicesView.hpp>
#include <oscar/Graphics/MeshTopology.hpp>
#include <oscar/Maths/Mat4.hpp>
#include <oscar/Maths/Transform.hpp>
#include <oscar/Maths/Triangle.hpp>
#include <oscar/Maths/Vec2.hpp>
#include <oscar/Maths/Vec3.hpp>
#include <oscar/Maths/Vec4.hpp>
#include <oscar/Utils/CopyOnUpdPtr.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <iosfwd>
#include <span>
#include <vector>

namespace osc { struct AABB; }
namespace osc { struct Color; }
namespace osc { class SubMeshDescriptor; }
namespace osc { struct Transform; }

// note: implementation is in `GraphicsImplementation.cpp`
namespace osc
{
    // mesh
    //
    // encapsulates mesh data, which may include vertices, indices, normals, texture
    // coordinates, etc.
    class Mesh final {
    public:
        Mesh();

        MeshTopology getTopology() const;
        void setTopology(MeshTopology);

        bool hasVerts() const;
        size_t getNumVerts() const;
        std::vector<Vec3> getVerts() const;
        void setVerts(std::span<Vec3 const>);
        void transformVerts(std::function<void(Vec3&)> const&);
        void transformVerts(Transform const&);
        void transformVerts(Mat4 const&);

        bool hasNormals() const;
        std::vector<Vec3> getNormals() const;
        void setNormals(std::span<Vec3 const>);
        void transformNormals(std::function<void(Vec3&)> const&);

        bool hasTexCoords() const;
        std::vector<Vec2> getTexCoords() const;
        void setTexCoords(std::span<Vec2 const>);
        void transformTexCoords(std::function<void(Vec2&)> const&);

        std::vector<Color> getColors() const;
        void setColors(std::span<Color const>);

        std::vector<Vec4> getTangents() const;
        void setTangents(std::span<Vec4 const>);

        MeshIndicesView getIndices() const;
        void setIndices(MeshIndicesView);
        void setIndices(std::span<uint16_t const>);
        void setIndices(std::span<uint32_t const>);

        void forEachIndexedVert(std::function<void(Vec3)> const&) const;
        void forEachIndexedTriangle(std::function<void(Triangle)> const&) const;

        AABB const& getBounds() const;  // local-space

        void clear();

        size_t getSubMeshCount() const;
        void pushSubMeshDescriptor(SubMeshDescriptor const&);
        SubMeshDescriptor const& getSubMeshDescriptor(size_t) const;
        void clearSubMeshDescriptors();

        friend void swap(Mesh& a, Mesh& b) noexcept
        {
            swap(a.m_Impl, b.m_Impl);
        }

        friend bool operator==(Mesh const&, Mesh const&) = default;
        friend std::ostream& operator<<(std::ostream&, Mesh const&);
    private:
        friend class GraphicsBackend;
        friend struct std::hash<Mesh>;

        class Impl;
        CopyOnUpdPtr<Impl> m_Impl;
    };

    std::ostream& operator<<(std::ostream&, Mesh const&);
}

template<>
struct std::hash<osc::Mesh> final {
    size_t operator()(osc::Mesh const& mesh) const
    {
        return std::hash<osc::CopyOnUpdPtr<osc::Mesh::Impl>>{}(mesh.m_Impl);
    }
};
