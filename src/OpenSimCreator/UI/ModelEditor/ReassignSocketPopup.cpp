#include "ReassignSocketPopup.h"

#include <OpenSimCreator/Documents/Model/UndoableModelActions.h>
#include <OpenSimCreator/Documents/Model/IModelStatePair.h>
#include <OpenSimCreator/UI/Shared/BasicWidgets.h>
#include <OpenSimCreator/Utils/OpenSimHelpers.h>

#include <OpenSim/Common/Component.h>
#include <OpenSim/Common/ComponentList.h>
#include <OpenSim/Common/ComponentPath.h>
#include <OpenSim/Common/ComponentSocket.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <oscar/UI/oscimgui.h>
#include <oscar/UI/Popups/StandardPopup.h>
#include <oscar/Utils/StringHelpers.h>

#include <algorithm>
#include <compare>
#include <memory>
#include <optional>
#include <ranges>
#include <sstream>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>

using namespace osc;
namespace rgs = std::ranges;

namespace
{
    // parameters that affect which sockets are displayed
    struct PopupParams final {

        PopupParams(
            UID modelVersion_,
            OpenSim::ComponentPath path_,
            std::string socketName_) :

            modelVersion{modelVersion_},
            componentPath{std::move(path_)},
            socketName{std::move(socketName_)}
        {
        }

        friend bool operator==(const PopupParams&, const PopupParams&) = default;

        UID modelVersion;
        OpenSim::ComponentPath componentPath;
        std::string socketName;
        std::string search;
    };

    // a single user-selectable connectee option
    struct ConnecteeOption final {

        explicit ConnecteeOption(const OpenSim::Component& c) :
            absPath{GetAbsolutePath(c)},
            name{c.getName()}
        {
        }

        friend auto operator<=>(const ConnecteeOption& lhs, const ConnecteeOption& rhs)
        {
            return std::tie(lhs.name, lhs.absPath.toString()) <=> std::tie(rhs.name, rhs.absPath.toString());
        }

        [[maybe_unused]]  // TODO: Ubuntu20 doesn't use this function
        friend bool operator==(const ConnecteeOption& lhs, const ConnecteeOption& rhs)
        {
            return std::tie(lhs.name, lhs.absPath.toString()) == std::tie(rhs.name, rhs.absPath.toString());
        }

        OpenSim::ComponentPath absPath;
        std::string name;
    };

    // generate a list of possible connectee options, given a set of popup parameters
    std::vector<ConnecteeOption> GenerateSelectionOptions(
        const OpenSim::Model& model,
        const PopupParams& params)
    {
        std::vector<ConnecteeOption> rv;

        const OpenSim::Component* component = FindComponent(model, params.componentPath);
        if (!component)
        {
            return rv;   // component isn't in model?
        }

        const OpenSim::AbstractSocket* socket = FindSocket(*component, params.socketName);
        if (!socket)
        {
            return rv;  // socket isn't in model?
        }

        for (const OpenSim::Component& other : model.getComponentList())
        {
            if (&other == component)
            {
                continue;  // hide redundant reconnnections
            }

            if (!contains(other.getName(), params.search))
            {
                continue;  // filtered out by search string
            }

            if (!IsAbleToConnectTo(*socket, other))
            {
                continue;  // connection would be rejected anyway
            }

            rv.emplace_back(other);
        }

        rgs::sort(rv);

        return rv;
    }
}

class osc::ReassignSocketPopup::Impl final : public StandardPopup {
public:
    Impl(std::string_view popupName,
         std::shared_ptr<IModelStatePair> model,
         std::string_view componentAbsPath,
         std::string_view socketName) :

        StandardPopup{popupName},
        m_Model{std::move(model)},
        m_Params{m_Model->getModelVersion(), std::string{componentAbsPath}, std::string{socketName}}
    {
    }

private:
    void impl_draw_content() final
    {
        // caching: regenerate cached socket list, if necessary
        //
        // we cache the list because searching+filtering all possible connectees is
        // very slow in OpenSim (#384)
        m_EditedParams.modelVersion = m_Model->getModelVersion();
        if (m_EditedParams != m_Params) {
            m_Options = GenerateSelectionOptions(m_Model->getModel(), m_EditedParams);
            m_Params = m_EditedParams;
        }

        // check: ensure the "from" side of the socket still exists
        const OpenSim::Component* component = FindComponent(m_Model->getModel(), m_Params.componentPath);
        if (not component) {
            request_close();
            return;
        }

        // check: ensure the socket still exists
        const OpenSim::AbstractSocket* socket = FindSocket(*component, m_Params.socketName);
        if (not socket) {
            request_close();
            return;
        }

        // draw UI

        ui::draw_text("%s's new connectee:", socket->getName().c_str());

        DrawSearchBar(m_EditedParams.search);

        ui::begin_child_panel(
            "##componentlist",
            Vec2{-1.0f, 16.0f*ui::get_text_line_height()},
            ui::ChildPanelFlag::Border
        );

        int id = 0;  // care: necessary because multiple connectees may have the same name
        for (const ConnecteeOption& option : m_Options) {
            ui::push_id(id++);
            if (ui::draw_selectable(option.name, option.absPath == m_UserSelectionAbsPath)) {
                m_UserSelectionAbsPath = option.absPath;
            }
            ui::draw_tooltip_if_item_hovered(option.absPath.toString());
            ui::pop_id();
        }
        ui::end_child_panel();

        if (not m_Error.empty()) {
            ui::set_next_item_width(ui::get_content_region_available().x);
            ui::draw_text_wrapped(m_Error);
        }

        ui::start_new_line();  // breathing room

        if (not m_UserSelectionAbsPath) {
            ui::begin_disabled();
        }
        if (ui::draw_button("Ok") and m_UserSelectionAbsPath) {
            const SocketReassignmentFlags flags = m_TryReexpressInDifferentFrame ?
                SocketReassignmentFlags::TryReexpressComponentInNewConnectee :
                SocketReassignmentFlags::None;

            const OpenSim::Component* selected = FindComponent(m_Model->getModel(), *m_UserSelectionAbsPath);

            if (selected && ActionReassignComponentSocket(*m_Model, m_Params.componentPath, m_Params.socketName, *selected, flags, m_Error))
            {
                request_close();
                return;
            }
        }
        if (ui::is_item_hovered(ui::HoveredFlag::AllowWhenDisabled)) {
            if (not m_UserSelectionAbsPath) {
                ui::draw_tooltip("Disabled", "A new connectee hasn't been selected.");
            }
        }
        if (not m_UserSelectionAbsPath) {
            ui::end_disabled();
        }
        ui::same_line();
        if (ui::draw_button("Cancel")) {
            request_close();
            return;
        }
        // Add a checkbox that lets the user re-express a component in a new frame (#326),
        // make sure the checkbox is hard to miss (#959).
        ui::same_line();
        ui::draw_vertical_separator();
        ui::same_line();
        tryDrawReexpressPropertyInFrameCheckbox(*component, *socket);
    }

    void impl_on_close() final
    {
        m_EditedParams.search.clear();
        m_Error.clear();
    }

    void tryDrawReexpressPropertyInFrameCheckbox(
        const OpenSim::Component& component,
        const OpenSim::AbstractSocket& abstractSocket)
    {
        const std::string label = [&component]()
        {
            std::stringstream ss;
            ss << "Re-express " << component.getName() << " in chosen frame";
            return std::move(ss).str();
        }();

        const auto* const physFrameSocket =
            dynamic_cast<const OpenSim::Socket<OpenSim::PhysicalFrame>*>(&abstractSocket);
        if (!physFrameSocket)
        {
            bool v = false;
            ui::draw_checkbox(label, &v);
            ui::draw_tooltip_body_only_if_item_hovered("Disabled: the socket doesn't connect to a physical frame");
            return;
        }

        const auto componentSpatialRepresentation =
            TryGetSpatialRepresentation(component, m_Model->getState());
        if (!componentSpatialRepresentation)
        {
            bool v = false;
            ui::draw_checkbox(label, &v);
            ui::draw_tooltip_body_only_if_item_hovered("Disabled: the component doesn't have a spatial representation that OSC knows how to re-express");
            return;
        }

        ui::draw_checkbox(label, &m_TryReexpressInDifferentFrame);
        ui::same_line();
        ui::draw_help_marker("Component Re-Expression", "This will recalculate the socket owner's appropriate spatial property such that it remains in the same location in ground after changing this socket.");
    }

    std::shared_ptr<IModelStatePair> m_Model;
    PopupParams m_Params;
    PopupParams m_EditedParams = m_Params;
    std::vector<ConnecteeOption> m_Options = GenerateSelectionOptions(m_Model->getModel(), m_EditedParams);
    std::optional<OpenSim::ComponentPath> m_UserSelectionAbsPath;
    std::string m_Error;
    bool m_TryReexpressInDifferentFrame = false;
};


// public API (PIMPL)

osc::ReassignSocketPopup::ReassignSocketPopup(
    std::string_view popupName,
    std::shared_ptr<IModelStatePair> model,
    std::string_view componentAbsPath,
    std::string_view socketName) :

    m_Impl{std::make_unique<Impl>(popupName, std::move(model), componentAbsPath, socketName)}
{
}

osc::ReassignSocketPopup::ReassignSocketPopup(ReassignSocketPopup&&) noexcept = default;
osc::ReassignSocketPopup& osc::ReassignSocketPopup::operator=(ReassignSocketPopup&&) noexcept = default;
osc::ReassignSocketPopup::~ReassignSocketPopup() noexcept = default;

bool osc::ReassignSocketPopup::impl_is_open() const
{
    return m_Impl->is_open();
}

void osc::ReassignSocketPopup::impl_open()
{
    m_Impl->open();
}

void osc::ReassignSocketPopup::impl_close()
{
    m_Impl->close();
}

bool osc::ReassignSocketPopup::impl_begin_popup()
{
    return m_Impl->begin_popup();
}

void osc::ReassignSocketPopup::impl_on_draw()
{
    m_Impl->on_draw();
}

void osc::ReassignSocketPopup::impl_end_popup()
{
    m_Impl->end_popup();
}
