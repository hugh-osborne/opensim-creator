#pragma once

#include "src/Graphics/Material.hpp"
#include "src/Graphics/MaterialPropertyBlock.hpp"
#include "src/Graphics/Mesh.hpp"
#include "src/Graphics/SceneDecorationFlags.hpp"
#include "src/Maths/AABB.hpp"
#include "src/Maths/Transform.hpp"

#include <glm/vec4.hpp>

#include <memory>
#include <optional>
#include <string>

namespace osc
{
    // represents a renderable decoration for a component in a model
    class SceneDecoration final {
    public:
        Mesh mesh;
        Transform transform;
        glm::vec4 color;
        std::string id;
        SceneDecorationFlags flags = SceneDecorationFlags_None;
        std::optional<Material> maybeMaterial = std::nullopt;
        std::optional<MaterialPropertyBlock> maybeMaterialProps = std::nullopt;

        explicit SceneDecoration(
            Mesh const& mesh_
        ) :
            mesh{mesh_},
            transform{},
            color{1.0f, 1.0f, 1.0f, 1.0f}
        {
        }

        SceneDecoration(
            Mesh const& mesh_,
            Transform const& transform_,
            glm::vec4 const& color_
        ) :
            mesh{mesh_},
            transform{transform_},
            color{color_}
        {
        }

        SceneDecoration(
            Mesh const& mesh_,
            Transform const& transform_,
            glm::vec4 const& color_,
            std::string id_,
            SceneDecorationFlags flags_
        ) :
            mesh{mesh_},
            transform{transform_},
            color{color_},
            id{std::move(id_)},
            flags{std::move(flags_)}
        {
        }

        SceneDecoration(
            Mesh const& mesh_,
            Transform const& transform_,
            glm::vec4 const& color_,
            std::string id_,
            SceneDecorationFlags flags_,
            std::optional<Material> maybeMaterial_,
            std::optional<MaterialPropertyBlock> maybeProps_ = std::nullopt) :

            mesh{mesh_},
            transform{transform_},
            color{color_},
            id{std::move(id_)},
            flags{std::move(flags_)},
            maybeMaterial{std::move(maybeMaterial_)},
            maybeMaterialProps{std::move(maybeProps_)}
        {
        }
    };

    bool operator==(SceneDecoration const&, SceneDecoration const&) noexcept;
}
