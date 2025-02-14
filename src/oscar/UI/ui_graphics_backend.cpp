#include "ui_graphics_backend.h"

#include <oscar/Graphics/Camera.h>
#include <oscar/Graphics/CameraClearFlags.h>
#include <oscar/Graphics/Color.h>
#include <oscar/Graphics/ColorSpace.h>
#include <oscar/Graphics/CullMode.h>
#include <oscar/Graphics/Graphics.h>
#include <oscar/Graphics/Material.h>
#include <oscar/Graphics/Mesh.h>
#include <oscar/Graphics/Shader.h>
#include <oscar/Graphics/SubMeshDescriptor.h>
#include <oscar/Graphics/Texture2D.h>
#include <oscar/Graphics/TextureFilterMode.h>
#include <oscar/Graphics/TextureFormat.h>
#include <oscar/Graphics/Unorm8.h>
#include <oscar/Graphics/VertexAttribute.h>
#include <oscar/Graphics/VertexAttributeFormat.h>
#include <oscar/Graphics/VertexFormat.h>
#include <oscar/Maths/MatFunctions.h>
#include <oscar/Maths/Mat4.h>
#include <oscar/Maths/Rect.h>
#include <oscar/Maths/Vec2.h>
#include <oscar/Maths/Vec3.h>
#include <oscar/Maths/Vec4.h>
#include <oscar/UI/oscimgui.h>
#include <oscar/Utils/Algorithms.h>
#include <oscar/Utils/Assertions.h>
#include <oscar/Utils/Concepts.h>
#include <oscar/Utils/CStringView.h>
#include <oscar/Utils/StdVariantHelpers.h>
#include <oscar/Utils/UID.h>

#include <ankerl/unordered_dense.h>
#define IMGUI_USER_CONFIG <oscar/UI/oscimgui_config.h>  // NOLINT(bugprone-macro-parentheses)
#include <imgui.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <variant>

namespace graphics = osc::graphics;
using namespace osc;

namespace
{
    constexpr CStringView c_ui_vertex_shader_src = R"(
        #version 330 core

        uniform mat4 uProjMat;

        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec2 aTexCoord;
        layout (location = 3) in vec4 aColor;

        out vec2 Frag_UV;
        out vec4 Frag_Color;

        void main()
        {
            Frag_UV = aTexCoord;
            Frag_Color = aColor;
            gl_Position = uProjMat * vec4(aPos, 1.0);
        }
    )";

    constexpr CStringView c_ui_fragment_shader_src = R"(
        #version 330 core

        uniform sampler2D uTexture;

        in vec2 Frag_UV;
        in vec4 Frag_Color;

        layout (location = 0) out vec4 Out_Color;

        void main()
        {
            Out_Color = Frag_Color * texture(uTexture, Frag_UV.st);
        }
    )";

    ImTextureID to_imgui_texture_id(UID id)
    {
        static_assert(std::is_same_v<ImTextureID, osc::ui::graphics_backend::InternalTextureID>);
        return static_cast<ImTextureID>(id.get());
    }

    UID to_uid(ImTextureID id)
    {
        return UID::from_int_unchecked(static_cast<UID::element_type>(id));
    }

    Texture2D create_font_texture(UID texture_id)
    {
        ImGuiIO& io = ImGui::GetIO();

        uint8_t* pixel_data = nullptr;
        Vec2i dims;
        io.Fonts->GetTexDataAsRGBA32(&pixel_data, &dims.x, &dims.y);
        io.Fonts->SetTexID(to_imgui_texture_id(texture_id));
        const size_t num_bytes = static_cast<size_t>(dims.x)*static_cast<size_t>(dims.y)*static_cast<size_t>(4);

        Texture2D rv{
            dims,
            TextureFormat::RGBA32,
            ColorSpace::Linear,
        };
        rv.set_pixel_data({pixel_data, num_bytes});
        rv.set_filter_mode(TextureFilterMode::Linear);

        return rv;
    }

    // Returns a lookup table that maps sRGB color bytes to linear-space color bytes
    std::array<uint8_t, 256> create_srgb_to_linear_lut()
    {
        std::array<uint8_t, 256> rv{};
        for (size_t i = 0; i < 256; ++i) {
            const auto ldr_color = Unorm8{static_cast<uint8_t>(i)};
            const float hdr_color = ldr_color.normalized_value();
            const float linear_hdr_color = to_linear_colorspace(hdr_color);
            rv[i] = Unorm8{linear_hdr_color}.raw_value();
        }
        return rv;
    }

    const std::array<uint8_t, 256>& get_srgc_to_linear_lut_singleton()
    {
        static const std::array<uint8_t, 256> s_srgb_to_linear_lut = create_srgb_to_linear_lut();
        return s_srgb_to_linear_lut;
    }

    void convert_draw_data_from_srgb_to_linear(ImDrawList& draw_list)
    {
        const std::array<uint8_t, 256>& lut = get_srgc_to_linear_lut_singleton();

        for (ImDrawVert& v : draw_list.VtxBuffer) {
            const auto r_srgb = static_cast<uint8_t>((v.col >> IM_COL32_R_SHIFT) & 0xFF);
            const auto g_srgb = static_cast<uint8_t>((v.col >> IM_COL32_G_SHIFT) & 0xFF);
            const auto b_srgb = static_cast<uint8_t>((v.col >> IM_COL32_B_SHIFT) & 0xFF);
            const auto alpha = static_cast<uint8_t>((v.col >> IM_COL32_A_SHIFT) & 0xFF);

            const uint8_t r_linear = lut[r_srgb];
            const uint8_t g_linear = lut[g_srgb];
            const uint8_t b_linear = lut[b_srgb];

            v.col =
                static_cast<ImU32>(r_linear) << IM_COL32_R_SHIFT |
                static_cast<ImU32>(g_linear) << IM_COL32_G_SHIFT |
                static_cast<ImU32>(b_linear) << IM_COL32_B_SHIFT |
                static_cast<ImU32>(alpha) << IM_COL32_A_SHIFT;
        }
    }

    struct OscarImguiBackendData final {

        OscarImguiBackendData()
        {
            ui_material.set_transparent(true);
            ui_material.set_cull_mode(CullMode::Off);
            ui_material.set_depth_tested(false);
            ui_material.set_wireframe(false);
        }

        UID font_texture_id;
        std::optional<Texture2D> font_texture;
        Material ui_material{Shader{c_ui_vertex_shader_src, c_ui_fragment_shader_src}};
        Camera camera;
        Mesh mesh;
        ankerl::unordered_dense::map<UID, std::variant<Texture2D, RenderTexture>> textures_allocated_this_frame;
    };

    // Backend data stored in io.BackendRendererUserData to allow support for multiple Dear ImGui contexts
    // It is STRONGLY preferred that you use docking branch with multi-viewports (== single Dear ImGui context + multiple windows) instead of multiple Dear ImGui contexts.
    OscarImguiBackendData* get_backend_data()
    {
        if (ImGui::GetCurrentContext()) {
            return static_cast<OscarImguiBackendData*>(ImGui::GetIO().BackendRendererUserData);
        }
        else {
            return nullptr;
        }
    }

    void setup_camera_view_matrix(const ImDrawData& draw_data, Camera& camera)
    {
        // Our visible imgui space lies from draw_data->DisplayPos (top left) to draw_data->DisplayPos+data_data->DisplaySize (bottom right). DisplayPos is (0,0) for single viewport apps.
        const float L = draw_data.DisplayPos.x;
        const float R = draw_data.DisplayPos.x + draw_data.DisplaySize.x;
        const float T = draw_data.DisplayPos.y;
        const float B = draw_data.DisplayPos.y + draw_data.DisplaySize.y;

        const Mat4 projection_matrix = {
            { 2.0f/(R-L),   0.0f,         0.0f,   0.0f },
            { 0.0f,         2.0f/(T-B),   0.0f,   0.0f },
            { 0.0f,         0.0f,        -1.0f,   0.0f },
            { (R+L)/(L-R),  (T+B)/(B-T),  0.0f,   1.0f },
        };

        camera.set_projection_matrix_override(projection_matrix);
    }

    void render_draw_command(
        OscarImguiBackendData& bd,
        const ImDrawData& draw_data,
        const ImDrawList&,
        Mesh& mesh,
        const ImDrawCmd& draw_command)
    {
        OSC_ASSERT(draw_command.UserCallback == nullptr && "user callbacks are not supported in oscar's ImGui renderer impl");

        // Project scissor/clipping rectangles from device-independent top-left coordinate
        // space into device-independent right-handed space
        const Vec2 clip_off = draw_data.DisplayPos;         // (0,0) unless using multi-viewports
        const Vec2 clip_min(draw_command.ClipRect.x - clip_off.x, draw_command.ClipRect.y - clip_off.y);
        const Vec2 clip_max(draw_command.ClipRect.z - clip_off.x, draw_command.ClipRect.w - clip_off.y);

        if (clip_max.x <= clip_min.x or clip_max.y <= clip_min.y) {
            return;
        }
        const Vec2 minflip{clip_min.x, (draw_data.DisplaySize.y) - clip_max.y};
        const Vec2 maxflip{clip_max.x, (draw_data.DisplaySize.y) - clip_min.y};

        // setup clipping rectangle
        bd.camera.set_clear_flags(CameraClearFlag::None);
        bd.camera.set_scissor_rect(Rect{minflip, maxflip});

        // setup sub-mesh description
        const size_t sub_mesh_index = mesh.num_submesh_descriptors();
        mesh.push_submesh_descriptor(SubMeshDescriptor{
            draw_command.IdxOffset,
            draw_command.ElemCount,
            MeshTopology::Triangles,
            draw_command.VtxOffset
        });

        if (const auto* texture = lookup_or_nullptr(bd.textures_allocated_this_frame, to_uid(draw_command.GetTexID()))) {
            std::visit(Overload{
                [&bd](const auto& texture) { bd.ui_material.set("uTexture", texture); },
            }, *texture);
            graphics::draw(mesh, identity<Mat4>(), bd.ui_material, bd.camera, std::nullopt, sub_mesh_index);
            bd.camera.render_to_screen();
        }
    }

    void render_drawlist(
        OscarImguiBackendData& bd,
        const ImDrawData& draw_data,
        ImDrawList& draw_list)
    {
        // HACK: convert all ImGui-provided colors from sRGB to linear
        //
        // this is necessary because the ImGui OpenGL backend's shaders
        // assume all color vertices and colors from textures are in
        // sRGB, but OSC can provide ImGui with linear OR sRGB textures
        // because OSC assumes the OpenGL backend is using automatic
        // color conversion support (in ImGui, it isn't)
        //
        // so what we do here is linearize all colors from ImGui and
        // always provide textures in the OSC style. The shaders in ImGui
        // then write linear color values to the screen, but because we
        // are *also* enabling GL_FRAMEBUFFER_SRGB, the OpenGL backend
        // will correctly convert those linear colors to sRGB if necessary
        // automatically
        //
        // (this shitshow is because ImGui's OpenGL backend behaves differently
        //  from OSCs - ultimately, we need an ImGui_ImplOSC backend)
        convert_draw_data_from_srgb_to_linear(draw_list);

        Mesh& mesh = bd.mesh;
        mesh.clear();
        mesh.set_vertex_buffer_params(draw_list.VtxBuffer.Size, {
            {VertexAttribute::Position,  VertexAttributeFormat::Float32x2},
            {VertexAttribute::TexCoord0, VertexAttributeFormat::Float32x2},
            {VertexAttribute::Color,     VertexAttributeFormat::Unorm8x4},
        });
        mesh.set_vertex_buffer_data(std::span<ImDrawVert>{draw_list.VtxBuffer.Data, static_cast<size_t>(draw_list.VtxBuffer.Size)});
        mesh.set_indices({draw_list.IdxBuffer.Data, static_cast<size_t>(draw_list.IdxBuffer.size())}, {MeshUpdateFlag::DontRecalculateBounds, MeshUpdateFlag::DontValidateIndices});

        // iterate through command buffer
        for (const ImDrawCmd& draw_command : draw_list.CmdBuffer) {
            render_draw_command(bd, draw_data, draw_list, mesh, draw_command);
        }
        mesh.clear();
    }

    template<IsAnyOf<Texture2D, RenderTexture> Texture>
    ImTextureID allocate_texture_for_current_frame(const Texture& texture)
    {
        OscarImguiBackendData* bd = get_backend_data();
        OSC_ASSERT(bd != nullptr && "no oscar ImGui renderer backend was available to shutdown - this is a developer error");
        const UID texture_uid = bd->textures_allocated_this_frame.try_emplace(UID{}, texture).first->first;
        return to_imgui_texture_id(texture_uid);
    }
}

bool osc::ui::graphics_backend::init()
{
    ImGuiIO& io = ImGui::GetIO();
    OSC_ASSERT(io.BackendRendererUserData == nullptr && "an oscar ImGui renderer backend is already initialized - this is a developer error (double-initialization)");

    // init backend data
    io.BackendRendererUserData = static_cast<void*>(new OscarImguiBackendData{});
    io.BackendRendererName = "imgui_impl_osc";
    io.BackendFlags |= ImGuiBackendFlags_RendererHasVtxOffset;

    return true;
}

void osc::ui::graphics_backend::shutdown()
{
    OscarImguiBackendData* bd = get_backend_data();
    OSC_ASSERT(bd != nullptr && "no oscar ImGui renderer backend was available to shutdown - this is a developer error (double-free)");

    // shutdown platform interface
    ImGui::DestroyPlatformWindows();

    // destroy backend data
    ImGuiIO& io = ImGui::GetIO();
    io.BackendRendererName = nullptr;
    io.BackendRendererUserData = nullptr;
    delete bd;  // NOLINT(cppcoreguidelines-owning-memory)
}

void osc::ui::graphics_backend::on_start_new_frame()
{
    // `ImGui_ImplOpenGL3_CreateDeviceObjects` is now part of constructing `OscarImguiBackendData`

    OscarImguiBackendData* bd = get_backend_data();
    OSC_ASSERT(bd != nullptr && "no oscar ImGui renderer backend was available - this is a developer error");
    bd->textures_allocated_this_frame.clear();
    if (not bd->font_texture) {
        bd->font_texture = create_font_texture(bd->font_texture_id);
    }
    bd->textures_allocated_this_frame.try_emplace(bd->font_texture_id, *bd->font_texture);  // (so that all lookups can hit the same LUT)
}

void osc::ui::graphics_backend::mark_fonts_for_reupload()
{
    if (OscarImguiBackendData* bd = get_backend_data()) {
        bd->font_texture.reset();
    }
}

void osc::ui::graphics_backend::render(ImDrawData* draw_data)
{
    OscarImguiBackendData* bd = get_backend_data();
    OSC_ASSERT(bd != nullptr && "no oscar ImGui renderer backend was available to shutdown - this is a developer error");

    setup_camera_view_matrix(*draw_data, bd->camera);
    for (int n = 0; n < draw_data->CmdListsCount; ++n) {
        render_drawlist(*bd, *draw_data, *draw_data->CmdLists[n]);
    }
}

osc::ui::graphics_backend::InternalTextureID osc::ui::graphics_backend::allocate_texture_for_current_frame(const Texture2D& texture)
{
    return ::allocate_texture_for_current_frame(texture);
}

osc::ui::graphics_backend::InternalTextureID osc::ui::graphics_backend::allocate_texture_for_current_frame(const RenderTexture& texture)
{
    return ::allocate_texture_for_current_frame(texture);
}
