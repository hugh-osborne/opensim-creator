#pragma once

#include "src/Maths/Rect.hpp"

#include <glm/vec2.hpp>
#include <glm/mat4x4.hpp>
#include <nonstd/span.hpp>
#include <imgui.h>

#include <cstddef>
#include <initializer_list>
#include <string>
#include <string_view>

namespace osc { class Camera; }
namespace osc { class PolarPerspectiveCamera; }
namespace osc { class RenderTexture; }
namespace osc { class Texture2D; }
namespace osc { class UID; }

namespace osc
{
    inline constexpr float defaultImguiDragThreshold = 5.0f;

    // applies "dark" theme to current ImGui context
    void ImGuiApplyDarkTheme();

    // updates a polar comera's rotation, position, etc. based on ImGui input
    void UpdatePolarCameraFromImGuiUserInput(glm::vec2 viewportDims, PolarPerspectiveCamera&);
    void UpdateEulerCameraFromImGuiUserInput(Camera&, glm::vec3& eulers);

    // returns the ImGui content region available in screenspace as a `Rect`
    Rect ContentRegionAvailScreenRect();

    // draws a texutre as an ImGui::Image, assumes UV coords of (0.0, 1.0); (1.0, 0.0)
    void DrawTextureAsImGuiImage(Texture2D&, glm::vec2 dims);
    void DrawTextureAsImGuiImage(RenderTexture&, glm::vec2 dims);

    struct ImGuiImageHittestResult final {
        Rect rect;
        bool isHovered;
        bool isLeftClickReleasedWithoutDragging;
        bool isRightClickReleasedWithoutDragging;

        ImGuiImageHittestResult();
    };
    ImGuiImageHittestResult DrawTextureAsImGuiImageAndHittest(Texture2D&, glm::vec2 dims, float dragThreshold = defaultImguiDragThreshold);
    ImGuiImageHittestResult DrawTextureAsImGuiImageAndHittest(RenderTexture&, glm::vec2 dims, float dragThreshold = defaultImguiDragThreshold);

    // returns `true` if any scancode in the provided range is currently pressed down
    bool IsAnyKeyDown(nonstd::span<int const>);
    bool IsAnyKeyDown(std::initializer_list<int const>);

    // returns `true` if any scancode in the provided range was pressed down this frame
    bool IsAnyKeyPressed(nonstd::span<int const>);
    bool IsAnyKeyPressed(std::initializer_list<int const>);

    // returns true if the user is pressing either left- or right-Ctrl
    bool IsCtrlDown();

    // returns `true` if the user is pressing either:
    //
    // - left Ctrl
    // - right Ctrl
    // - left Super (mac)
    // - right Super (mac)
    bool IsCtrlOrSuperDown();

    // returns `true` if the user is pressing either left- or right-shift
    bool IsShiftDown();

    // returns `true` if the user is pressing either left- or right-alt
    bool IsAltDown();

    // returns `true` if the specified moouse button was released without the user dragging
    bool IsMouseReleasedWithoutDragging(ImGuiMouseButton, float threshold = defaultImguiDragThreshold);

    // draws an overlay tooltip (content only)
    void DrawTooltipBodyOnly(char const*);

    // draws an overlay tooltip (content only) if the last item is hovered
    void DrawTooltipBodyOnlyIfItemHovered(char const*);

    // draws an overlay tooltip with a header and description
    void DrawTooltip(char const* header, char const* description = nullptr);

    // equivalent to `if (ImGui::IsItemHovered()) DrawTooltip(header, description);`
    void DrawTooltipIfItemHovered(char const* header, char const* description = nullptr);

    // draw overlay axes in bottom-right of screenspace rect
    void DrawAlignmentAxesOverlayInBottomRightOf(glm::mat4 const& viewMtx, Rect const& renderRect);

    // draw a help text marker `"(?)"` and display a tooltip when the user hovers over it
    void DrawHelpMarker(char const* header, char const* desc);

    // draw a help text marker `"(?)"` and display a tooltip when the user hovers over it
    void DrawHelpMarker(char const*);

    // draw an ImGui::InputText that manipulates a std::string
    bool InputString(const char* label, std::string& s, std::size_t maxLen, ImGuiInputTextFlags flags = 0);

    // draw a vec3/f3 editor with a lock icon
    bool DrawF3Editor(char const* lock_id, char const* editor_id, float* v, bool* is_locked);

    // draw an ImGui::InputFloat that manipulates in the scene scale (note: some users work with very very small sizes)
    bool InputMetersFloat(const char* label, float* v, float step = 0.0f, float step_fast = 0.0f, ImGuiInputTextFlags flags = 0);

    // draw an ImGui::InputFloat3 that manipulates in the scene scale (note: some users work with very very small sizes)
    bool InputMetersFloat3(const char* label, float v[3], ImGuiInputTextFlags flags = 0);

    // draw an ImGui::SliderFloat that manipulates in the scene scale (note: some users work with very very small sizes)
    bool SliderMetersFloat(const char* label, float* v, float v_min, float v_max, ImGuiSliderFlags flags = 0);

    // draw an ImGui::InputFloat for masses (note: some users work with very very small masses)
    bool InputKilogramFloat(const char* label, float* v, float step = 0.0f, float step_fast = 0.0f, ImGuiInputTextFlags flags = 0);

    // push an osc::UID as if it were an ImGui ID (via ImGui::PushID)
    void PushID(UID const&);

    // returns "minimal" window flags (i.e. no title bar, can't move the window - ideal for images etc.)
    ImGuiWindowFlags GetMinimalWindowFlags();

    // returns a `Rect` that indicates where the current workspace area is in the main viewport
    //
    // handy if (e.g.) you want to know the rect of a tab area
    Rect GetMainViewportWorkspaceScreenRect();

    // returns `true` if the user's mouse is within the current workspace area of the main viewport
    bool IsMouseInMainViewportWorkspaceScreenRect();

    // begin a menu that's attached to the top of a viewport, end it with ImGui::End();
    bool BeginMainViewportTopBar(char const* label);

    // begin a menu that's attached to the bottom of a viewport, end it with ImGui::End();
    bool BeginMainViewportBottomBar(char const* label);

    // draw text, but centered on the current window/line
    void TextCentered(std::string const&);

    // add an application-level frame annotation to the last-drawn ImGui item
    void AddFrameAnnotationToLastItem(std::string_view);
}
