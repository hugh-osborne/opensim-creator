#include "SubMeshTab.hpp"

#include <oscar/Bindings/ImGuiHelpers.hpp>
#include <oscar/Graphics/Camera.hpp>
#include <oscar/Graphics/Color.hpp>
#include <oscar/Graphics/Graphics.hpp>
#include <oscar/Graphics/Material.hpp>
#include <oscar/Graphics/Mesh.hpp>
#include <oscar/Graphics/MeshGenerators.hpp>
#include <oscar/Graphics/Shader.hpp>
#include <oscar/Graphics/SubMeshDescriptor.hpp>
#include <oscar/Maths/Transform.hpp>
#include <oscar/Maths/Vec3.hpp>
#include <oscar/Platform/App.hpp>
#include <oscar/UI/Tabs/StandardTabBase.hpp>
#include <oscar/Utils/CStringView.hpp>

#include <IconsFontAwesome5.h>
#include <SDL_events.h>

#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

using osc::Mesh;
using osc::SubMeshDescriptor;
using osc::Vec3;

namespace
{
    constexpr osc::CStringView c_TabStringID = "Demos/SubMeshes";

    Mesh GenerateMeshWithSubMeshes()
    {
        auto const meshes = std::to_array(
        {
            osc::GenCube(),
            osc::GenSphere(16, 16),
            osc::GenCircle(32),
        });

        std::vector<Vec3> allVerts;
        std::vector<Vec3> allNormals;
        std::vector<uint32_t> allIndices;
        std::vector<SubMeshDescriptor> allDescriptors;

        for (auto const& mesh : meshes)
        {
            allVerts.insert(allVerts.end(), mesh.getVerts().begin(), mesh.getVerts().end());
            allNormals.insert(allNormals.end(), mesh.getNormals().begin(), mesh.getNormals().end());

            size_t firstIndex = allIndices.size();
            for (auto index : mesh.getIndices())
            {
                allIndices.push_back(static_cast<uint32_t>(firstIndex + index));
            }
            size_t nIndices = allIndices.size() - firstIndex;

            allDescriptors.emplace_back(firstIndex, nIndices, mesh.getTopology());
        }

        Mesh rv;
        rv.setVerts(allVerts);
        rv.setNormals(allNormals);
        rv.setIndices(allIndices);
        for (auto const& desc : allDescriptors)
        {
            rv.pushSubMeshDescriptor(desc);
        }
        return rv;
    }
}

class osc::SubMeshTab::Impl final : public osc::StandardTabBase {
public:
    Impl() : StandardTabBase{c_TabStringID}
    {
        m_Camera.setBackgroundColor(Color::white());
        m_Camera.setNearClippingPlane(0.1f);
        m_Camera.setFarClippingPlane(5.0f);
        m_Camera.setPosition({0.0f, 0.0f, -2.5f});
        m_Camera.setDirection({0.0f, 0.0f, 1.0f});

        m_Material.setColor("uColor", Color::red());
        m_Material.setWireframeMode(true);
    }

private:
    void implOnMount() final {}
    void implOnUnmount() final {}

    bool implOnEvent(SDL_Event const&) final
    {
        return false;
    }

    void implOnTick() final {}

    void implOnDrawMainMenu() final {}

    void implOnDraw() final
    {
        for (size_t subMeshIndex = 0; subMeshIndex < m_MeshWithSubmeshes.getSubMeshCount(); ++subMeshIndex)
        {

            osc::Graphics::DrawMesh(
                m_MeshWithSubmeshes,
                Identity<Transform>(),
                m_Material,
                m_Camera,
                std::nullopt,
                subMeshIndex
            );
        }
        m_Camera.setPixelRect(GetMainViewportWorkspaceScreenRect());
        m_Camera.renderToScreen();
    }

    Camera m_Camera;
    Material m_Material
    {
        Shader
        {
            App::slurp("oscar_demos/shaders/SolidColor.vert"),
            App::slurp("oscar_demos/shaders/SolidColor.frag"),
        },
    };
    Mesh m_MeshWithSubmeshes = GenerateMeshWithSubMeshes();
};


// public API

osc::CStringView osc::SubMeshTab::id()
{
    return c_TabStringID;
}

osc::SubMeshTab::SubMeshTab(ParentPtr<TabHost> const&) :
    m_Impl{std::make_unique<Impl>()}
{
}

osc::SubMeshTab::SubMeshTab(SubMeshTab&&) noexcept = default;
osc::SubMeshTab& osc::SubMeshTab::operator=(SubMeshTab&&) noexcept = default;
osc::SubMeshTab::~SubMeshTab() noexcept = default;

osc::UID osc::SubMeshTab::implGetID() const
{
    return m_Impl->getID();
}

osc::CStringView osc::SubMeshTab::implGetName() const
{
    return m_Impl->getName();
}

void osc::SubMeshTab::implOnMount()
{
    m_Impl->onMount();
}

void osc::SubMeshTab::implOnUnmount()
{
    m_Impl->onUnmount();
}

bool osc::SubMeshTab::implOnEvent(SDL_Event const& e)
{
    return m_Impl->onEvent(e);
}

void osc::SubMeshTab::implOnTick()
{
    m_Impl->onTick();
}

void osc::SubMeshTab::implOnDrawMainMenu()
{
    m_Impl->onDrawMainMenu();
}

void osc::SubMeshTab::implOnDraw()
{
    m_Impl->onDraw();
}
