#include "ComponentHierarchy.hpp"

#include "src/Assertions.hpp"
#include "src/Log.hpp"

#include <OpenSim/Common/Component.h>
#include <OpenSim/Simulation/Model/Geometry.h>
#include <OpenSim/Simulation/Wrap/WrapObjectSet.h>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <string>

namespace {
    size_t const g_FrameGeometryHash = typeid(OpenSim::FrameGeometry).hash_code();
    size_t const g_WrapObjectSetHash = typeid(OpenSim::WrapObjectSet).hash_code();

    // poor-man's abstraction for a constant-sized array
    template<typename T, size_t N>
    struct SizedArray final {
        static_assert(std::is_trivial_v<T>);
        static_assert(N <= std::numeric_limits<int>::max());

        std::array<T, N> els;
        int n = 0;

        SizedArray& operator=(SizedArray const& rhs) {
            std::copy(rhs.els.begin(), rhs.els.begin() + rhs.size(), els.begin());
            n = rhs.n;
            return *this;
        }

        void push_back(T v) {
            if (static_cast<size_t>(n) >= N) {
                throw std::runtime_error{"cannot render a component_hierarchy widget: the Model/Component tree is too deep"};
            }
            els[n++] = v;
        }

        T const* begin() const noexcept {
            return els.data();
        }

        T* begin() noexcept {
            return els.data();
        }

        T const* end() const noexcept {
            return els.data() + static_cast<size_t>(n);
        }

        T* end() noexcept {
            return els.data() + static_cast<size_t>(n);
        }

        size_t size() const noexcept {
            return static_cast<size_t>(n);
        }

        int sizei() const noexcept {
            return n;
        }

        bool empty() const noexcept {
            return n == 0;
        }

        void resize(size_t newsize) noexcept {
            n = newsize;
        }

        void clear() noexcept {
            n = 0;
        }

        T& operator[](size_t idx) {
            return els[idx];
        }

        T const& operator[](size_t i) const noexcept {
            return els[i];
        }
    };

    using ComponentPath = SizedArray<OpenSim::Component const*, 16>;

    // populates `out` with the sequence of nodes between (ancestor..child]
    void computeComponentPath(
            OpenSim::Component const* ancestor,
            OpenSim::Component const* child,
            ComponentPath& out) {

        out.clear();

        // populate child --> parent
        while (child) {
            out.push_back(child);

            if (!child->hasOwner()) {
                break;
            }

            if (child == ancestor) {
                break;
            }

            child = &child->getOwner();
        }

        // reverse to yield parent --> child
        std::reverse(out.begin(), out.end());
    }

    bool pathContains(ComponentPath const& p, OpenSim::Component const* c) {
        return std::find(p.begin(), p.end(), c) != p.end();
    }

    bool shouldRender(OpenSim::Component const& c) {
        auto hc = typeid(c).hash_code();
        return hc != g_FrameGeometryHash && hc != g_WrapObjectSetHash;
    }
}

static bool isSearchHit(char const* searchStr, ComponentPath const& cp) {
    for (auto const& c : cp) {
        std::string const& name = c->getName();
        if (name.find(searchStr) != std::string::npos) {
            return true;
        }
    }
    return false;
}


// public API

osc::ComponentHierarchy::Response osc::ComponentHierarchy::draw(
    OpenSim::Component const* root,
    OpenSim::Component const* selection,
    OpenSim::Component const* hover) {

    ImGui::InputText("search", search, sizeof(search));

    Response response;

    if (!root) {
        return response;
    }

    ComponentPath selectionPath;
    if (selection) {
        computeComponentPath(root, selection, selectionPath);
    }

    ComponentPath hoverPath;
    if (hover) {
        computeComponentPath(root, hover, hoverPath);
    }

    // init iterators: this alg. is single-pass with a 1-token lookahead
    auto const lst = root->getComponentList();
    auto it = lst.begin();
    auto const end = lst.end();

    // initially populate lookahead (+ path)
    OpenSim::Component const* lookahead = root;
    ComponentPath lookaheadPath;
    computeComponentPath(root, root, lookaheadPath);

    // set cur path empty (first step copies lookahead into this)
    OpenSim::Component const* cur = nullptr;
    ComponentPath currentPath;

    int imguiTreeDepth = 0;
    int imguiId = 0;
    bool hasSearch = std::strlen(search) > 0;

    while (lookahead) {
        // important: ensure all nodes have a unique ID: regardess of filtering
        ++imguiId;

        // populate current (+ path) from lookahead
        cur = lookahead;
        currentPath = lookaheadPath;

        OSC_ASSERT(cur && "cur ptr should *definitely* be populated at this point");
        OSC_ASSERT(!currentPath.empty() && "current path cannot be empty (even a root element has a path)");

        // update lookahead (+ path) by stepping to the next component in the component tree
        lookahead = nullptr;
        lookaheadPath.clear();
        while (it != end) {
            OpenSim::Component const& c = *it++;

            if (shouldRender(c)) {
                lookahead = &c;
                computeComponentPath(root, &c, lookaheadPath);
                break;
            }
        }
        OSC_ASSERT((lookahead || !lookahead) && "a lookahead is not *required* at this point");

        bool searchHit = hasSearch && isSearchHit(search, currentPath);

        // skip rendering if a parent node is collapsed
        if (imguiTreeDepth < currentPath.sizei() - 1) {
            continue;
        }

        // pop tree nodes down to the current depth
        while (imguiTreeDepth >= currentPath.sizei()) {
            ImGui::TreePop();
            --imguiTreeDepth;
        }
        OSC_ASSERT(imguiTreeDepth <= currentPath.sizei() - 1);


        if (currentPath.size() < 3 || lookaheadPath.size() > currentPath.size()) {
            // render as an expandable tree node

            int styles = 0;
            if (searchHit || cur == hover) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4{0.5f, 0.5f, 0.0f, 1.0f});
                ++styles;
            }
            if (cur == selection) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4{1.0f, 1.0f, 0.0f, 1.0f});
                ++styles;
            }

            if (searchHit || currentPath.sizei() == 1 || pathContains(selectionPath, cur)) {
                ImGui::SetNextItemOpen(true);
            }

            ImGui::PushID(imguiId);
            if (ImGui::TreeNode(cur->getName().c_str())) {
                ++imguiTreeDepth;
            }
            ImGui::PopID();
            ImGui::PopStyleColor(styles);
        } else {
            // render as plain text

            int styles = 0;
            if (searchHit || pathContains(hoverPath, cur)) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4{0.5f, 0.5f, 0.0f, 1.0f});
                ++styles;
            }
            if (pathContains(selectionPath, cur)) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4{1.0f, 1.0f, 0.0f, 1.0f});
                ++styles;
            }

            ImGui::PushID(imguiId);
            ImGui::BulletText("%s", cur->getName().c_str());
            ImGui::PopID();
            ImGui::PopStyleColor(styles);
        }

        if (ImGui::IsItemHovered()) {
            response.type = HoverChanged;
            response.ptr = cur;
        }

        if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
            response.type = SelectionChanged;
            response.ptr = cur;
        }
    }

    // pop remaining dangling tree elements
    while (imguiTreeDepth-- > 0) {
        ImGui::TreePop();
    }

    return response;
}
