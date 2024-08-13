#include "LOGLShadowMappingTab.h"

#include <oscar/oscar.h>
#include <SDL_events.h>

#include <cstdint>
#include <memory>
#include <optional>

using namespace osc::literals;
using namespace osc;

namespace
{
    constexpr CStringView c_tab_string_id = "LearnOpenGL/ShadowMapping";

    // this matches the plane vertices used in the LearnOpenGL tutorial
    Mesh generate_learnopengl_plane_mesh()
    {
        Mesh rv;
        rv.set_vertices({
            { 25.0f, -0.5f,  25.0f},
            {-25.0f, -0.5f,  25.0f},
            {-25.0f, -0.5f, -25.0f},

            { 25.0f, -0.5f,  25.0f},
            {-25.0f, -0.5f, -25.0f},
            { 25.0f, -0.5f, -25.0f},
        });
        rv.set_normals({
            {0.0f, 1.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},

            {0.0f, 1.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
        });
        rv.set_tex_coords({
            {25.0f,  0.0f},
            {0.0f,  0.0f},
            {0.0f, 25.0f},

            {25.0f,  0.0f},
            {0.0f, 25.0f},
            {25.0f, 25.0f},
        });
        rv.set_indices({0, 1, 2, 3, 4, 5});
        return rv;
    }

    MouseCapturingCamera create_camera()
    {
        MouseCapturingCamera rv;
        rv.set_position({-2.0f, 1.0f, 0.0f});
        rv.set_clipping_planes({0.1f, 100.0f});
        return rv;
    }

    RenderTexture create_depth_texture()
    {
        RenderTexture rv;
        RenderTextureDescriptor shadowmap_descriptor{Vec2i{1024, 1024}};
        shadowmap_descriptor.set_read_write(RenderTextureReadWrite::Linear);
        rv.reformat(shadowmap_descriptor);
        return rv;
    }
}

class osc::LOGLShadowMappingTab::Impl final : public StandardTabImpl {
public:
    Impl() : StandardTabImpl{c_tab_string_id}
    {}

private:
    void impl_on_mount() final
    {
        App::upd().make_main_loop_polling();
        camera_.on_mount();
    }

    void impl_on_unmount() final
    {
        camera_.on_unmount();
        App::upd().make_main_loop_waiting();
    }

    bool impl_on_event(const SDL_Event& e) final
    {
        return camera_.on_event(e);
    }

    void impl_on_draw() final
    {
        camera_.on_draw();
        draw_3d_scene();
    }

    void draw_3d_scene()
    {
        const Rect viewport_screenspace_rect = ui::get_main_viewport_workspace_screenspace_rect();
        const Vec2 top_left = top_left_rh(viewport_screenspace_rect);
        constexpr float depth_overlay_size = 200.0f;

        render_shadows_to_depth_texture();

        camera_.set_background_color({0.1f, 0.1f, 0.1f, 1.0f});

        scene_material_.set_vec3("uLightWorldPos", light_pos_);
        scene_material_.set_vec3("uViewWorldPos", camera_.position());
        scene_material_.set_mat4("uLightSpaceMat", latest_lightspace_matrix_);
        scene_material_.set_texture("uDiffuseTexture", wood_texture_);
        scene_material_.set_render_texture("uShadowMapTexture", depth_texture_);

        draw_meshes_with_material(scene_material_);
        camera_.set_pixel_rect(viewport_screenspace_rect);
        camera_.render_to_screen();
        camera_.set_pixel_rect(std::nullopt);
        graphics::blit_to_screen(depth_texture_, Rect{top_left - Vec2{0.0f, depth_overlay_size}, top_left + Vec2{depth_overlay_size, 0.0f}});

        scene_material_.unset("uShadowMapTexture");
    }

    void draw_meshes_with_material(const Material& material)
    {
        // floor
        graphics::draw(plane_mesh_, identity<Transform>(), material, camera_);

        // cubes
        graphics::draw(
            cube_mesh_,
            {.scale = Vec3{0.5f}, .position = {0.0f, 1.0f, 0.0f}},
            material,
            camera_
        );
        graphics::draw(
            cube_mesh_,
            {.scale = Vec3{0.5f}, .position = {2.0f, 0.0f, 1.0f}},
            material,
            camera_
        );
        graphics::draw(
            cube_mesh_,
            Transform{
                .scale = Vec3{0.25f},
                .rotation = angle_axis(60_deg, UnitVec3{1.0f, 0.0f, 1.0f}),
                .position = {-1.0f, 0.0f, 2.0f},
            },
            material,
            camera_
        );
    }

    void render_shadows_to_depth_texture()
    {
        const float znear = 1.0f;
        const float zfar = 7.5f;
        const Mat4 light_view_matrix = look_at(light_pos_, Vec3{0.0f}, {0.0f, 1.0f, 0.0f});
        const Mat4 light_projection_matrix = ortho(-10.0f, 10.0f, -10.0f, 10.0f, znear, zfar);
        latest_lightspace_matrix_ = light_projection_matrix * light_view_matrix;

        draw_meshes_with_material(depth_material_);

        camera_.set_view_matrix_override(light_view_matrix);
        camera_.set_projection_matrix_override(light_projection_matrix);
        camera_.render_to(depth_texture_);
        camera_.set_view_matrix_override(std::nullopt);
        camera_.set_projection_matrix_override(std::nullopt);
    }

    ResourceLoader loader_ = App::resource_loader();
    MouseCapturingCamera camera_ = create_camera();
    Texture2D wood_texture_ = load_texture2D_from_image(
        loader_.open("oscar_learnopengl/textures/wood.png"),
        ColorSpace::sRGB
    );
    Mesh cube_mesh_ = BoxGeometry{2.0f, 2.0f, 2.0f};
    Mesh plane_mesh_ = generate_learnopengl_plane_mesh();
    Material scene_material_{Shader{
        loader_.slurp("oscar_learnopengl/shaders/AdvancedLighting/shadow_mapping/Scene.vert"),
        loader_.slurp("oscar_learnopengl/shaders/AdvancedLighting/shadow_mapping/Scene.frag"),
    }};
    Material depth_material_{Shader{
        loader_.slurp("oscar_learnopengl/shaders/AdvancedLighting/shadow_mapping/MakeShadowMap.vert"),
        loader_.slurp("oscar_learnopengl/shaders/AdvancedLighting/shadow_mapping/MakeShadowMap.frag"),
    }};
    RenderTexture depth_texture_ = create_depth_texture();
    Mat4 latest_lightspace_matrix_ = identity<Mat4>();
    Vec3 light_pos_ = {-2.0f, 4.0f, -1.0f};
};


CStringView osc::LOGLShadowMappingTab::id()
{
    return c_tab_string_id;
}

osc::LOGLShadowMappingTab::LOGLShadowMappingTab(const ParentPtr<ITabHost>&) :
    impl_{std::make_unique<Impl>()}
{}
osc::LOGLShadowMappingTab::LOGLShadowMappingTab(LOGLShadowMappingTab&&) noexcept = default;
osc::LOGLShadowMappingTab& osc::LOGLShadowMappingTab::operator=(LOGLShadowMappingTab&&) noexcept = default;
osc::LOGLShadowMappingTab::~LOGLShadowMappingTab() noexcept = default;

UID osc::LOGLShadowMappingTab::impl_get_id() const
{
    return impl_->id();
}

CStringView osc::LOGLShadowMappingTab::impl_get_name() const
{
    return impl_->name();
}

void osc::LOGLShadowMappingTab::impl_on_mount()
{
    impl_->on_mount();
}

void osc::LOGLShadowMappingTab::impl_on_unmount()
{
    impl_->on_unmount();
}

bool osc::LOGLShadowMappingTab::impl_on_event(const SDL_Event& e)
{
    return impl_->on_event(e);
}

void osc::LOGLShadowMappingTab::impl_on_draw()
{
    impl_->on_draw();
}
