#pragma once

#include "mesh_reference.hpp"
#include "raw_mesh_instance.hpp"

#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>

#include <cstddef>

namespace gl {
    struct Texture_2d;
}

namespace osmv {
    struct Untextured_vert;
    struct Textured_vert;
    class Raw_drawlist;
}

// raw renderer: an OpenGL renderer that is Application, Screen, and OpenSim agnostic.
//
// this API is designed with performance and power in mind, not convenience. Use a downstream
// renderer (e.g. a specialized OpenSim model renderer) if you need something more convenient.
namespace osmv {
    // globally allocate mesh data on the GPU
    //
    // the returned handle is a "mesh ID" and is guaranteed to be a non-negative number that
    // increases monotonically
    //
    // must only be called after OpenGL is initialized
    Mesh_reference globally_allocate_mesh(osmv::Untextured_vert const* verts, size_t n);
    Mesh_reference globally_allocate_mesh(osmv::Textured_vert const* verts, size_t n);
    Texture_reference globally_store_texture(gl::Texture_2d&&);
    void nuke_gpu_allocations();

    struct Raw_renderer_config final {
        int w;
        int h;
        int samples;
    };

    using Raw_renderer_flags = int;
    enum Raw_renderer_flags_ {
        RawRendererFlags_None = 0 << 0,

        // draw meshes in wireframe mode
        RawRendererFlags_WireframeMode = 1 << 0,

        // draw mesh normals on top of render
        RawRendererFlags_ShowMeshNormals = 1 << 1,

        // draw a chequered floor
        RawRendererFlags_ShowFloor = 1 << 2,

        // draw selection rims
        RawRendererFlags_DrawRims = 1 << 3,

        // draw debug quads (development)
        RawRendererFlags_DrawDebugQuads = 1 << 4,

        // perform hit testing on Raw_mesh_instance passthrough data
        RawRendererFlags_PerformPassthroughHitTest = 1 << 5,

        // use optimized hit testing (which might arrive a frame late)
        RawRendererFlags_UseOptimizedButDelayed1FrameHitTest = 1 << 6,

        // draw the scene
        RawRendererFlags_DrawSceneGeometry = 1 << 7,

        RawRendererFlags_Default = RawRendererFlags_ShowFloor | RawRendererFlags_DrawRims |
                                   RawRendererFlags_DrawDebugQuads | RawRendererFlags_PerformPassthroughHitTest |
                                   RawRendererFlags_UseOptimizedButDelayed1FrameHitTest |
                                   RawRendererFlags_DrawSceneGeometry
    };

    struct Raw_drawcall_params final {
        glm::mat4 view_matrix;
        glm::mat4 projection_matrix;
        glm::vec3 view_pos;
        glm::vec3 light_pos;
        glm::vec3 light_rgb;
        glm::vec4 background_rgba;
        glm::vec4 rim_rgba;

        Raw_renderer_flags flags;
        int passthrough_hittest_x;
        int passthrough_hittest_y;
    };

    struct Raw_drawcall_result final {
        gl::Texture_2d& texture;
        Passthrough_data passthrough_result;
    };

    struct Renderer_impl;
    class Raw_renderer final {
        Renderer_impl* impl;

    public:
        Raw_renderer(Raw_renderer_config const&);
        Raw_renderer(Raw_renderer const&) = delete;
        Raw_renderer(Raw_renderer&&) = delete;
        Raw_renderer& operator=(Raw_renderer const&) = delete;
        Raw_renderer& operator=(Raw_renderer&&) = delete;
        ~Raw_renderer() noexcept;

        void change_config(Raw_renderer_config const&);
        glm::vec2 dimensions() const noexcept;
        float aspect_ratio() const noexcept;

        Raw_drawcall_result draw(Raw_drawcall_params const&, Raw_drawlist const&);
    };
}
