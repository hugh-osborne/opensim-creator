#include "ScreenshotTab.h"

#include <oscar/Formats/Image.h>
#include <oscar/Graphics/Camera.h>
#include <oscar/Graphics/Color.h>
#include <oscar/Graphics/ColorSpace.h>
#include <oscar/Graphics/Graphics.h>
#include <oscar/Graphics/Material.h>
#include <oscar/Graphics/Mesh.h>
#include <oscar/Graphics/RenderTexture.h>
#include <oscar/Graphics/Shader.h>
#include <oscar/Graphics/Texture2D.h>
#include <oscar/Graphics/TextureFormat.h>
#include <oscar/Maths/CollisionTests.h>
#include <oscar/Maths/MatFunctions.h>
#include <oscar/Maths/Mat4.h>
#include <oscar/Maths/MathHelpers.h>
#include <oscar/Maths/Vec2.h>
#include <oscar/Maths/Vec3.h>
#include <oscar/Maths/Vec4.h>
#include <oscar/Platform/App.h>
#include <oscar/Platform/os.h>
#include <oscar/Platform/Screenshot.h>
#include <oscar/UI/ImGuiHelpers.h>
#include <oscar/UI/oscimgui.h>
#include <oscar/UI/Tabs/StandardTabImpl.h>
#include <oscar/Utils/Assertions.h>

#include <IconsFontAwesome5.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace osc;

namespace
{
    constexpr Color c_unselected_color = {1.0f, 1.0f, 1.0f, 0.4f};
    constexpr Color c_selected_color = {1.0f, 0.0f, 0.0f, 0.8f};

    // returns a rect that fully spans at least one dimension of the target rect, but has
    // the given aspect ratio
    //
    // the returned rectangle is in the same space as the target rectangle
    Rect shrink_to_fit(Rect target_rect, float aspect_ratio)
    {
        const float target_aspect_ratio = aspect_ratio_of(target_rect);
        const float ratio = target_aspect_ratio / aspect_ratio;
        const Vec2 target_dimensions = dimensions_of(target_rect);

        if (ratio >= 1.0f) {
            // it will touch the top/bottom but may (ratio != 1.0f) fall short of the left/right
            const Vec2 rv_dimensions = {target_dimensions.x/ratio, target_dimensions.y};
            const Vec2 rv_topleft = {target_rect.p1.x + 0.5f*(target_dimensions.x - rv_dimensions.x), target_rect.p1.y};
            return {rv_topleft, rv_topleft + rv_dimensions};
        }
        else {
            // it will touch the left/right but will not touch the top/bottom
            const Vec2 rv_dimensions = {target_dimensions.x, ratio*target_dimensions.y};
            const Vec2 rv_topleft = {target_rect.p1.x, target_rect.p1.y + 0.5f*(target_dimensions.y - rv_dimensions.y)};
            return {rv_topleft, rv_topleft + rv_dimensions};
        }
    }

    Rect map_rect(const Rect& source_rect, const Rect& target_rect, const Rect& rect)
    {
        const Vec2 scale = dimensions_of(target_rect) / dimensions_of(source_rect);

        return Rect{
            target_rect.p1 + scale*(rect.p1 - source_rect.p1),
            target_rect.p1 + scale*(rect.p2 - source_rect.p1),
        };
    }
}

class osc::ScreenshotTab::Impl final : public StandardTabImpl {
public:
    explicit Impl(Screenshot&& screenshot) :
        StandardTabImpl{ICON_FA_COOKIE " ScreenshotTab"},
        screenshot_{std::move(screenshot)}
    {
        image_texture_.set_filter_mode(TextureFilterMode::Mipmap);
    }

private:
    void impl_on_draw_main_menu() final
    {
        if (ui::BeginMenu("File")) {
            if (ui::MenuItem("Save")) {
                action_try_save_annotated_screenshot();
            }
            ui::EndMenu();
        }
    }

    void impl_on_draw() final
    {
        ui::DockSpaceOverViewport(ui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode);

        // draw screenshot window
        {
            ui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0.0f, 0.0f});
            ui::Begin("Screenshot");
            ui::PopStyleVar();

            const Rect ui_image_rect = draw_screenshot_as_image();
            draw_image_overlays(*ui::GetWindowDrawList(), ui_image_rect, c_unselected_color, c_selected_color);

            ui::End();
        }

        // draw controls window
        {
            int id = 0;
            ui::Begin("Controls");
            for (const ScreenshotAnnotation& annotation : screenshot_.annotations()) {
                ui::PushID(id++);
                ui::TextUnformatted(annotation.label());
                ui::PopID();
            }
            ui::End();
        }
    }

    // returns screenspace rect of the screenshot within the UI
    Rect draw_screenshot_as_image()
    {
        const Vec2 cursor_topleft = ui::GetCursorScreenPos();
        const Rect window_rect = {cursor_topleft, cursor_topleft + Vec2{ui::GetContentRegionAvail()}};
        const Rect image_rect = shrink_to_fit(window_rect, aspect_ratio_of(screenshot_.dimensions()));
        ui::SetCursorScreenPos(image_rect.p1);
        ui::Image(image_texture_, dimensions_of(image_rect));
        return image_rect;
    }

    void draw_image_overlays(
        ImDrawList& drawlist,
        const Rect& image_rect,
        const Color& unselected_color,
        const Color& selected_color)
    {
        const Vec2 mouse_pos = ui::GetMousePos();
        const bool left_click_released = ui::IsMouseReleased(ImGuiMouseButton_Left);
        const Rect image_source_rect = {{0.0f, 0.0f}, screenshot_.dimensions()};

        for (const ScreenshotAnnotation& annotation : screenshot_.annotations()) {
            const Rect annotation_rect_screen = map_rect(image_source_rect, image_rect, annotation.rect());
            const bool selected = user_selected_annotations_.contains(annotation.label());
            const bool hovered = is_intersecting(annotation_rect_screen, mouse_pos);

            Vec4 color = selected ? selected_color : unselected_color;
            if (hovered) {
                color.w = saturate(color.w + 0.3f);
            }

            if (hovered and left_click_released) {
                if (selected) {
                    user_selected_annotations_.erase(annotation.label());
                }
                else {
                    user_selected_annotations_.insert(annotation.label());
                }
            }

            drawlist.AddRect(
                annotation_rect_screen.p1,
                annotation_rect_screen.p2,
                ui::ColorConvertFloat4ToU32(color),
                3.0f,
                0,
                3.0f
            );
        }
    }

    void action_try_save_annotated_screenshot()
    {
        const std::optional<std::filesystem::path> maybe_image_path =
            PromptUserForFileSaveLocationAndAddExtensionIfNecessary("png");

        if (maybe_image_path) {
            std::ofstream fout{*maybe_image_path, std::ios_base::binary};
            if (!fout) {
                throw std::runtime_error{maybe_image_path->string() + ": cannot open for writing"};
            }
            const Texture2D annotated_screenshot = render_annotated_screenshot();
            write_to_png(annotated_screenshot, fout);
            open_file_in_os_default_application(*maybe_image_path);
        }
    }

    Texture2D render_annotated_screenshot()
    {
        RenderTexture render_texture{image_texture_.dimensions()};

        // blit the screenshot into the output
        graphics::blit(image_texture_, render_texture);

        // draw overlays to a local ImGui drawlist
        ImDrawList drawlist{ui::GetDrawListSharedData()};
        drawlist.Flags |= ImDrawListFlags_AntiAliasedLines;
        drawlist.AddDrawCmd();
        Color outline_color = c_selected_color;
        outline_color.a = 1.0f;
        draw_image_overlays(
            drawlist,
            Rect{{0.0f, 0.0f}, image_texture_.dimensions()},
            {0.0f, 0.0f, 0.0f, 0.0f},
            outline_color
        );

        // render drawlist to output
        {
            // upload vertex positions/colors
            Mesh mesh;
            {
                // vertices
                {
                    std::vector<Vec3> vertices;
                    vertices.reserve(drawlist.VtxBuffer.size());
                    for (const ImDrawVert& vert : drawlist.VtxBuffer) {
                        vertices.emplace_back(vert.pos.x, vert.pos.y, 0.0f);
                    }
                    mesh.set_vertices(vertices);
                }

                // colors
                {
                    std::vector<Color> colors;
                    colors.reserve(drawlist.VtxBuffer.size());
                    for (const ImDrawVert& vert : drawlist.VtxBuffer) {
                        const Color linear_color = ui::to_color(vert.col);
                        colors.push_back(linear_color);
                    }
                    mesh.set_colors(colors);
                }
            }

            // solid color material
            Material material{Shader{
                App::slurp("oscar/shaders/PerVertexColor.vert"),
                App::slurp("oscar/shaders/PerVertexColor.frag"),
            }};

            Camera c;
            c.set_view_matrix_override(identity<Mat4>());

            {
                // project screenspace overlays into NDC
                float L = 0.0f;
                float R = static_cast<float>(image_texture_.dimensions().x);
                float T = 0.0f;
                float B = static_cast<float>(image_texture_.dimensions().y);
                const Mat4 proj = {
                    { 2.0f/(R-L),   0.0f,         0.0f,   0.0f },
                    { 0.0f,         2.0f/(T-B),   0.0f,   0.0f },
                    { 0.0f,         0.0f,        -1.0f,   0.0f },
                    { (R+L)/(L-R),  (T+B)/(B-T),  0.0f,   1.0f },
                };
                c.set_projection_matrix_override(proj);
            }
            c.set_clear_flags(CameraClearFlags::Nothing);

            for (int cmd_index = 0; cmd_index < drawlist.CmdBuffer.Size; ++cmd_index) {
                const ImDrawCmd& cmd = drawlist.CmdBuffer[cmd_index];
                {
                    // upload indices
                    std::vector<ImDrawIdx> indices;
                    indices.reserve(cmd.ElemCount);
                    for (unsigned int i = cmd.IdxOffset; i < cmd.IdxOffset + cmd.ElemCount; ++i)
                    {
                        indices.push_back(drawlist.IdxBuffer[static_cast<int>(i)]);
                    }
                    mesh.set_indices(indices);
                }
                graphics::draw(mesh, Transform{}, material, c);
            }

            c.render_to(render_texture);
        }

        Texture2D rv{render_texture.dimensions(), TextureFormat::RGB24, ColorSpace::sRGB};
        graphics::copy_texture(render_texture, rv);
        return rv;
    }

    Screenshot screenshot_;
    Texture2D image_texture_ = screenshot_.image();
    std::unordered_set<std::string> user_selected_annotations_;
};

osc::ScreenshotTab::ScreenshotTab(const ParentPtr<ITabHost>&, Screenshot&& screenshot) :
    impl_{std::make_unique<Impl>(std::move(screenshot))}
{}

osc::ScreenshotTab::ScreenshotTab(ScreenshotTab&&) noexcept = default;
osc::ScreenshotTab& osc::ScreenshotTab::operator=(ScreenshotTab&&) noexcept = default;
osc::ScreenshotTab::~ScreenshotTab() noexcept = default;

UID osc::ScreenshotTab::impl_get_id() const
{
    return impl_->id();
}

CStringView osc::ScreenshotTab::impl_get_name() const
{
    return impl_->name();
}

void osc::ScreenshotTab::impl_on_draw_main_menu()
{
    impl_->on_draw_main_menu();
}

void osc::ScreenshotTab::impl_on_draw()
{
    impl_->on_draw();
}
