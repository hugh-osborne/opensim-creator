#pragma once

#include "gl.hpp"

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <array>
#include <vector>

// 3d common: common primitives/structs used for mesh generation/rendering
namespace osmv {
    struct Textured_vert final {
        glm::vec3 pos;
        glm::vec3 normal;
        glm::vec2 texcoord;
    };

    struct Untextured_vert final {
        glm::vec3 pos;
        glm::vec3 normal;
    };

    // standard textured quad
    // - dimensions [-1, +1] in xy and [0, 0] in z
    // - uv coords are (0, 0) bottom-left, (1, 1) top-right
    // - normal is +1 in Z, meaning that it faces toward the camera
    static constexpr std::array<Textured_vert, 6> shaded_textured_quad_verts = {{
        // CCW winding (culling)
        {{-1.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f}},  // bottom-left
        {{1.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f}},  // bottom-right
        {{1.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 1.0f}},  // top-right

        {{1.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 1.0f}},  // top-right
        {{-1.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f}},  // top-left
        {{-1.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f}},  // bottom-left
    }};

    // Returns triangles of a "unit" (radius = 1.0f, origin = 0,0,0) sphere
    void unit_sphere_triangles(std::vector<Untextured_vert>& out);
    std::vector<Untextured_vert> unit_sphere_triangles();

    // Returns triangles for a "unit" cylinder with `num_sides` sides.
    //
    // Here, "unit" means:
    //
    // - radius == 1.0f
    // - top == [0.0f, 0.0f, -1.0f]
    // - bottom == [0.0f, 0.0f, +1.0f]
    // - (so the height is 2.0f, not 1.0f)
    void unit_cylinder_triangles(size_t num_sides, std::vector<Untextured_vert>& out);
    std::vector<Untextured_vert> unit_cylinder_triangles(size_t num_sides);

    // Returns triangles for a standard "simbody" cylinder
    //
    // This matches simbody-visualizer.cpp's definition of a cylinder, which
    // is:
    //
    // radius
    //     1.0f
    // top
    //     [0.0f, 1.0f, 0.0f]
    // bottom
    //     [0.0f, -1.0f, 0.0f]
    //
    // see simbody-visualizer.cpp::makeCylinder for my source material
    void simbody_cylinder_triangles(std::vector<Untextured_vert>& out);
    std::vector<Untextured_vert> simbody_cylinder_triangles();

    // Returns triangles for a standard "Simbody" cube
    //
    // TODO: I have no idea what a Simbody cube is, the verts returned by this are
    // a pure guess
    void simbody_brick_triangles(std::vector<osmv::Untextured_vert>& out);
    std::vector<Untextured_vert> simbody_brick_triangles();
}
