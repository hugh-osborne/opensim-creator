#include "ModelEditorViewerPanel.hpp"

#include "src/Bindings/ImGuiHelpers.hpp"
#include "src/Graphics/IconCache.hpp"
#include "src/Graphics/MeshCache.hpp"
#include "src/Graphics/ShaderCache.hpp"
#include "src/Maths/MathHelpers.hpp"
#include "src/Maths/Rect.hpp"
#include "src/OpenSimBindings/MiddlewareAPIs/EditorAPI.hpp"
#include "src/OpenSimBindings/Rendering/CachedModelRenderer.hpp"
#include "src/OpenSimBindings/Rendering/ModelRendererParams.hpp"
#include "src/OpenSimBindings/Widgets/BasicWidgets.hpp"
#include "src/OpenSimBindings/Widgets/ComponentContextMenu.hpp"
#include "src/OpenSimBindings/OpenSimHelpers.hpp"
#include "src/OpenSimBindings/UndoableModelStatePair.hpp"
#include "src/Panels/StandardPanel.hpp"
#include "src/Platform/App.hpp"
#include "src/Widgets/GuiRuler.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>
#include <ImGuizmo.h>
#include <OpenSim/Common/Component.h>
#include <OpenSim/Common/ComponentPath.h>

#include <memory>
#include <string_view>
#include <utility>

class osc::ModelEditorViewerPanel::Impl final : public osc::StandardPanel {
public:

    Impl(
        std::string_view panelName,
        MainUIStateAPI* mainUIStateAPI,
        EditorAPI* editorAPI,
        std::shared_ptr<UndoableModelStatePair> model) :

        StandardPanel{std::move(panelName)},
        m_MainUIStateAPI{mainUIStateAPI},
        m_EditorAPI{editorAPI},
        m_Model{std::move(model)}
    {
    }

private:
    void implBeforeImGuiBegin() final
    {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0.0f, 0.0f});
    }

    void implAfterImGuiBegin() final
    {
        ImGui::PopStyleVar();
    }

    void implDrawContent() final
    {
        // compute viewer size (all available space)
        Rect const viewportRect = ContentRegionAvailScreenRect();

        // if this is the first frame being rendered, auto-focus the scene
        if (!m_MaybeLastHittest)
        {
            m_CachedModelRenderer.autoFocusCamera(
                *m_Model,
                m_Params,
                AspectRatio(viewportRect)
            );
        }

        // if hovering the panel, and not using an overlay, process mouse+keyboard inputs
        if (m_MaybeLastHittest &&
            m_MaybeLastHittest->isHovered &&
            !isUsingAnOverlay())
        {
            UpdatePolarCameraFromImGuiInputs(
                m_Params.camera,
                viewportRect,
                m_CachedModelRenderer.getRootAABB()
            );
        }

        // render the 3D scene to a texture and blit it via ImGui::Image
        {
            osc::RenderTexture& sceneTexture = m_CachedModelRenderer.draw(
                *m_Model,
                m_Params,
                Dimensions(viewportRect),
                App::get().getMSXAASamplesRecommended()
            );
            DrawTextureAsImGuiImage(sceneTexture, Dimensions(viewportRect));
        }

        // item-hittest the ImGui::Image so we know whether the user is interacting with it
        ImGuiItemHittestResult const imguiHittest = HittestLastImguiItem();
        m_MaybeLastHittest = imguiHittest;

        // if hovering the image item, and not dragging the mouse around, 3D-hittest the
        // scene so we know whether the user's mouse hits something in 3D
        std::optional<SceneCollision> maybeSceneCollision;
        if (imguiHittest.isHovered &&
            !IsDraggingWithAnyMouseButtonDown())
        {
            maybeSceneCollision = m_CachedModelRenderer.getClosestCollision(
                m_Params,
                ImGui::GetMousePos(),
                viewportRect
            );
        }

        // if the mouse hits something in 3D, and the mouse isn't busy doing something
        // else (e.g. dragging around) then lookup the 3D hit in the model, so we know
        // whether the user is interacting with a component in the model
        OpenSim::Component const* maybeHover = nullptr;
        if (maybeSceneCollision && !isUsingAnOverlay())
        {
            maybeHover = osc::FindComponent(
                m_Model->getModel(),
                maybeSceneCollision->decorationID
            );
        }

        // draw 2D overlays over the 3D scene image
        draw2DImguiOverlays(
            viewportRect,
            imguiHittest,
            maybeSceneCollision,
            maybeHover
        );

        // handle any other model/state mutations as a result of interaction
        handleInteractionRelatedModelSideEffects(
            imguiHittest,
            maybeHover
        );
    }

    bool isUsingAnOverlay() const
    {
        return m_Ruler.isMeasuring() || ImGuizmo::IsUsing();
    }

    bool isHoveringAnOverlay() const
    {
        return ImGuizmo::IsOver();
    }

    // uses ImGui's 2D drawlist to draw interactive widgets/overlays on the panel
    void draw2DImguiOverlays(
        Rect const& viewportRect,
        ImGuiItemHittestResult const& imguiHittest,
        std::optional<SceneCollision> const& maybeSceneHittest,
        OpenSim::Component const* maybeHover)
    {
        // draw generic overlays (i.e. the buttons for toggling things)
        DrawViewerImGuiOverlays(
            m_Params,
            m_CachedModelRenderer.getDrawlist(),
            m_CachedModelRenderer.getRootAABB(),
            viewportRect,
            *m_IconCache,
            m_Ruler
        );

        // if applicable, draw the ruler
        m_Ruler.draw(m_Params.camera, viewportRect, maybeSceneHittest);

        // draw gizmo manipulators over the top
        // drawGizmoOverlay(viewportRect);

        if (maybeHover)
        {
            DrawComponentHoverTooltip(*maybeHover);
        }

        // right-click: open context menu
        if (imguiHittest.isRightClickReleasedWithoutDragging)
        {
            // right-click: draw a context menu
            std::string const menuName = std::string{getName()} + "_contextmenu";
            OpenSim::ComponentPath const path = osc::GetAbsolutePathOrEmpty(maybeHover);
            m_EditorAPI->pushPopup(std::make_unique<ComponentContextMenu>(menuName, m_MainUIStateAPI, m_EditorAPI, m_Model, path));
        }
    }

    // draws 3D manipulation gizmo overlay
    void drawGizmoOverlay(Rect const& viewportRect)
    {
        // - get current selection
        // - if selection empty, don't draw gizmos
        // - if selection not empty, figure out whether gizmos make sense for the selection:
        //
        //     - muscle point
        //     - marker
        //     - body
        //     - joint parent/child
        //
        // - if the selection makes sense, get the selection's transform matrix
        // - manipulate the transform matrix via ImGuizmo
        // - continuously apply manipulations to the model **but don't save them**
        // - once the user stops manipulating, save the changes

        glm::vec2 const viewportDims = osc::Dimensions(viewportRect);

        ImGuizmo::SetRect(
            viewportRect.p1.x,
            viewportRect.p1.y,
            viewportDims.x,
            viewportDims.y
        );
        ImGuizmo::SetDrawlist(ImGui::GetWindowDrawList());
        ImGuizmo::AllowAxisFlip(false);

        glm::mat4 imguizmoMtx{1.0};
        glm::mat4 delta;
        bool manipulated = ImGuizmo::Manipulate(
            glm::value_ptr(m_Params.camera.getViewMtx()),
            glm::value_ptr(m_Params.camera.getProjMtx(AspectRatio(viewportRect))),
            ImGuizmo::TRANSLATE,
            ImGuizmo::WORLD,
            glm::value_ptr(imguizmoMtx),
            glm::value_ptr(delta),
            nullptr,
            nullptr,
            nullptr
        );
    }

    // handles any interactions that change the model (e.g. what's selected)
    void handleInteractionRelatedModelSideEffects(
        ImGuiItemHittestResult const& imguiHittest,
        OpenSim::Component const* maybeHover)
    {
        // handle hover mutations
        if (isHoveringAnOverlay() || isUsingAnOverlay())
        {
            m_Model->setHovered(nullptr);
        }
        else if (imguiHittest.isHovered && maybeHover != m_Model->getHovered())
        {
            // care: this code must check whether the hover != current hover
            // (even if null), because there might be multiple viewports open
            // (#582)
            m_Model->setHovered(maybeHover);
        }

        // left-click: set model selection to (potentially empty) hover
        if (imguiHittest.isLeftClickReleasedWithoutDragging &&
            !(isHoveringAnOverlay() || isUsingAnOverlay()))
        {
            m_Model->setSelected(maybeHover);
        }
    }

    // tab/model state
    MainUIStateAPI* m_MainUIStateAPI;
    EditorAPI* m_EditorAPI;
    std::shared_ptr<UndoableModelStatePair> m_Model;

    // 3D render/image state
    ModelRendererParams m_Params;
    CachedModelRenderer m_CachedModelRenderer
    {
        App::get().getConfig(),
        App::singleton<MeshCache>(),
        *App::singleton<ShaderCache>(),
    };
    std::optional<ImGuiItemHittestResult> m_MaybeLastHittest;

    // overlay state
    std::shared_ptr<IconCache> m_IconCache = App::singleton<osc::IconCache>(
        App::resource("icons/"),
        ImGui::GetTextLineHeight()/128.0f
    );
    GuiRuler m_Ruler;
};


// public API (PIMPL)

osc::ModelEditorViewerPanel::ModelEditorViewerPanel(
    std::string_view panelName,
    MainUIStateAPI* mainUIStateAPI,
    EditorAPI* editorAPI,
    std::shared_ptr<UndoableModelStatePair> model) :

    m_Impl{std::make_unique<Impl>(std::move(panelName), mainUIStateAPI, editorAPI, std::move(model))}
{
}
osc::ModelEditorViewerPanel::ModelEditorViewerPanel(ModelEditorViewerPanel&&) noexcept = default;
osc::ModelEditorViewerPanel& osc::ModelEditorViewerPanel::operator=(ModelEditorViewerPanel&&) noexcept = default;
osc::ModelEditorViewerPanel::~ModelEditorViewerPanel() noexcept = default;

osc::CStringView osc::ModelEditorViewerPanel::implGetName() const
{
    return m_Impl->getName();
}

bool osc::ModelEditorViewerPanel::implIsOpen() const
{
    return m_Impl->isOpen();
}

void osc::ModelEditorViewerPanel::implOpen()
{
    m_Impl->open();
}

void osc::ModelEditorViewerPanel::implClose()
{
    m_Impl->close();
}

void osc::ModelEditorViewerPanel::implDraw()
{
    m_Impl->draw();
}