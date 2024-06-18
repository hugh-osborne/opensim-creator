#pragma once

#include <oscar/Graphics/Mesh.h>
#include <oscar/Utils/CStringView.h>

#include <cstddef>

namespace osc
{
    class PlaneGeometry final {
    public:
        static constexpr CStringView name() { return "Plane"; }

        PlaneGeometry(
            float width = 1.0f,
            float height = 1.0f,
            size_t num_width_segments = 1,
            size_t num_height_segments = 1
        );

        const Mesh& mesh() const { return mesh_; }
        operator const Mesh& () const { return mesh_; }
    private:
        Mesh mesh_;
    };
}
