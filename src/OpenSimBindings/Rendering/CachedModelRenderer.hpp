#pragma once

#include "src/Maths/AABB.hpp"
#include "src/Graphics/SceneCollision.hpp"

#include <glm/vec2.hpp>
#include <nonstd/span.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace osc { class Config; }
namespace osc { struct Line; }
namespace osc { class MeshCache; }
namespace osc { class ModelRendererParams; }
namespace osc { class RenderTexture; }
namespace osc { class SceneDecoration; }
namespace osc { class ShaderCache; }
namespace osc { class VirtualConstModelStatePair; }

namespace osc
{
    class CachedModelRenderer {
    public:
        CachedModelRenderer(
            Config const&,
            std::shared_ptr<MeshCache>,
            ShaderCache&
        );
        CachedModelRenderer(CachedModelRenderer const&) = delete;
        CachedModelRenderer(CachedModelRenderer&&) noexcept;
        CachedModelRenderer& operator=(CachedModelRenderer const&) = delete;
        CachedModelRenderer& operator=(CachedModelRenderer&&) noexcept;
        ~CachedModelRenderer() noexcept;

        void populate(
            VirtualConstModelStatePair const&,
            ModelRendererParams const&
        );

        void draw(
            VirtualConstModelStatePair const&,
            ModelRendererParams const&,
            glm::vec2 dims,
            int32_t samples
        );

        RenderTexture& updRenderTexture();
        nonstd::span<SceneDecoration const> getDrawlist() const;
        std::optional<AABB> getRootAABB() const;
        std::vector<SceneCollision> getAllSceneCollisions(Line const& worldspaceRay) const;

    private:
        class Impl;
        std::unique_ptr<Impl> m_Impl;
    };
}