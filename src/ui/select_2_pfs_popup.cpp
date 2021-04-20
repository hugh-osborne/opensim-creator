#include "select_2_pfs_popup.hpp"

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

#include <imgui.h>

std::optional<osc::ui::select_2_pfs::Response> osc::ui::select_2_pfs::draw(
    State& st, char const* modal_name, OpenSim::Model const& model, char const* first_label, char const* second_label) {

    // center the modal
    {
        ImVec2 center = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowSize(ImVec2(512, 0));
    }

    // try to show modal
    if (!ImGui::BeginPopupModal(modal_name, nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        // modal not showing
        return std::nullopt;
    }

    ImGui::Columns(2);

    ImGui::Text("%s", first_label);
    ImGui::BeginChild("first", ImVec2(256, 256), true, ImGuiWindowFlags_HorizontalScrollbar);
    for (auto const& b : model.getComponentList<OpenSim::PhysicalFrame>()) {
        if (&b == st.second) {
            continue;  // don't allow circular connections
        }

        int styles_pushed = 0;
        if (&b == st.first) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4{0.3f, 1.0f, 0.3f, 1.0f});
            ++styles_pushed;
        }
        if (ImGui::Selectable(b.getName().c_str())) {
            st.first = &b;
        }
        ImGui::PopStyleColor(styles_pushed);
    }
    ImGui::EndChild();
    ImGui::NextColumn();

    ImGui::Text("%s", second_label);
    ImGui::BeginChild("second", ImVec2(256, 256), true, ImGuiWindowFlags_HorizontalScrollbar);
    for (auto const& b : model.getComponentList<OpenSim::PhysicalFrame>()) {
        if (&b == st.first) {
            continue;  // don't allow circular connections
        }

        int styles_pushed = 0;
        if (&b == st.second) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4{0.3f, 1.0f, 0.3f, 1.0f});
            ++styles_pushed;
        }
        if (ImGui::Selectable(b.getName().c_str())) {
            st.second = &b;
        }
        ImGui::PopStyleColor(styles_pushed);
    }
    ImGui::EndChild();
    ImGui::NextColumn();

    ImGui::Columns(1);

    std::optional<Response> rv = std::nullopt;

    if (st.first && st.second) {
        if (ImGui::Button("OK")) {
            rv.emplace(*st.first, *st.second);
            st = {};  // reset user inputs
            ImGui::CloseCurrentPopup();
        }
        ImGui::SetItemDefaultFocus();
        ImGui::SameLine();
    }

    if (ImGui::Button("cancel")) {
        st = {};  // reset user inputs
        ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();

    return rv;
}
