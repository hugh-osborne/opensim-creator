#include "ThinPlateWarpTab.hpp"

#include "src/Bindings/ImGuiHelpers.hpp"
#include "src/Bindings/GlmHelpers.hpp"
#include "src/Graphics/Camera.hpp"
#include "src/Graphics/Graphics.hpp"
#include "src/Graphics/Material.hpp"
#include "src/Graphics/Mesh.hpp"
#include "src/Graphics/MeshGen.hpp"
#include "src/Graphics/ShaderCache.hpp"
#include "src/Maths/MathHelpers.hpp"
#include "src/Platform/App.hpp"
#include "src/Platform/Log.hpp"
#include "src/Utils/Algorithms.hpp"
#include "src/Widgets/LogViewerPanel.hpp"

#include <glm/mat3x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <IconsFontAwesome5.h>
#include <imgui.h>
#include <nonstd/span.hpp>
#include <SDL_events.h>
#include <Simbody.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <sstream>
#include <iostream>
#include <limits>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

// 2D TPS algorithm stuff
//
// most of the background behind this is discussed in issue #467. For redundancy's sake, here
// are some of the references used to write this implementation:
//
// - primary literature source: https://ieeexplore.ieee.org/document/24792
// - blog explanation: https://profs.etsmtl.ca/hlombaert/thinplates/
// - blog explanation #2: https://khanhha.github.io/posts/Thin-Plate-Splines-Warping/
namespace
{
    // a single source-to-destination landmark pair in 2D space
    //
    // this is typically what the user/caller defines
    struct LandmarkPair2D final {
        glm::vec2 src;
        glm::vec2 dest;
    };

    // pretty-prints `LandmarkPair2D`
    std::ostream& operator<<(std::ostream& o, LandmarkPair2D const& p)
    {
        using osc::operator<<;
        o << "LandmarkPair2D{src = " << p.src << ", dest = " << p.dest << '}';
        return o;
    }

    // this is effectviely the "U" term in the TPS algorithm literature (which is usually U(r) = r^2 * log(r^2))
    //
    // i.e. U(||pi - p||) in the literature is equivalent to `RadialBasisFunction2D(pi, p)` here
    float RadialBasisFunction2D(glm::vec2 controlPoint, glm::vec2 p)
    {
        glm::vec2 const diff = controlPoint - p;
        float const r2 = glm::dot(diff, diff);

        if (r2 == 0.0f)
        {
            // this ensures that the result is always non-zero and non-NaN (this might be
            // necessary for some types of linear solvers?)
            return std::numeric_limits<float>::min();
        }
        else
        {
            return r2 * std::log(r2);
        }
    }

    // a single non-affine term of the 2D TPS equation
    //
    // i.e. in `f(p) = a1 + a2*p.x + a3*p.y + SUM{ wi * U(||controlPoint - p||) }` this encodes
    //      the `wi` and `controlPoint` parts of that equation
    struct TPSNonAffineTerm2D final {
        glm::vec2 weight;
        glm::vec2 controlPoint;

        TPSNonAffineTerm2D(glm::vec2 weight_, glm::vec2 controlPoint_) :
            weight{weight_},
            controlPoint{controlPoint_}
        {
        }
    };

    // pretty-prints `TPSNonAffineTerm2D`
    std::ostream& operator<<(std::ostream& o, TPSNonAffineTerm2D const& wt)
    {
        using osc::operator<<;
        return o << "TPSNonAffineTerm2D{weight = " << wt.weight << ", controlPoint = " << wt.controlPoint << '}';
    }

    // all coefficients in the 2D TPS equation
    //
    // i.e. these are the a1, a2, a3, and w's (+ control points) terms of the equation
    struct TPSCoefficients2D final {
        glm::vec2 a1 = {0.0f, 0.0f};
        glm::vec2 a2 = {1.0f, 0.0f};
        glm::vec2 a3 = {0.0f, 1.0f};
        std::vector<TPSNonAffineTerm2D> weights;
    };

    // pretty-prints TPSCoefficients2D
    std::ostream& operator<<(std::ostream& o, TPSCoefficients2D const& coefs)
    {
        using osc::operator<<;
        o << "TPSCoefficients2D{a1 = " << coefs.a1 << ", a2 = " << coefs.a2 << ", a3 = " << coefs.a3;
        for (size_t i = 0; i < coefs.weights.size(); ++i)
        {
            o << ", w" << i << " = " << coefs.weights[i];
        }
        o << '}';
        return o;
    }

    // evaluates the TPS equation with the given coefficients and input point
    glm::vec2 Evaluate(TPSCoefficients2D const& coefs, glm::vec2 p)
    {
        // this implementation effectively evaluates both `fx(x, y)` and `fy(x, y)` at
        // the same time, because `TPSCoefficients2D` stores the X and Y variants of the
        // coefficients together in memory (as `vec2`s)

        // compute affine terms (a1 + a2*x + a3*y)
        glm::vec2 rv = coefs.a1 + coefs.a2*p.x + coefs.a3*p.y;

        // accumulate non-affine terms (effectively: wi * U(||controlPoint - p||))
        for (TPSNonAffineTerm2D const& wt : coefs.weights)
        {
            rv += wt.weight * RadialBasisFunction2D(wt.controlPoint, p);
        }

        return rv;
    }

    // computes all coefficients of the TPS equation (a1, a2, a3, and all the w's)
    TPSCoefficients2D CalcCoefficients(nonstd::span<LandmarkPair2D const> landmarkPairs)
    {
        // this is based on the Bookstein Thin Plate Sline (TPS) warping algorithm
        //
        // 1. A TPS warp is (simplifying here) a linear combination:
        //
        //     f(p) = a1 + a2*p.x + a3*p.y + SUM{ wi * U(||controlPoint_i - p||) }
        //
        //    which can be represented as a matrix multiplication between the terms (1, p.x, p.y,
        //    U(||cpi - p||)) and the coefficients (a1, a2, a3, wi..)
        //
        // 2. The caller provides "landmark pairs": these are (effectively) the input
        //    arguments and the expected output
        //
        // 3. This algorithm uses the input + output to solve for the linear coefficients.
        //    Once those coefficients are known, we then have a linear equation that we
        //    we can pump new inputs into (e.g. mesh points, muscle points)
        //
        // 4. So, given the equation L * [w a] = [v o], where L is a matrix of linear terms,
        //    [w a] is a vector of the linear coefficients (we're solving for these), and [v o]
        //    is the expected output (v), with some (padding) zero elements (o)
        //
        // 5. Create matrix L:
        //
        //   |K  P|
        //   |PT 0|
        //
        //     where:
        //
        //     - K is a symmetric matrix of each *input* landmark pair evaluated via the
        //       basis function:
        //
        //        |U(p00) U(p01) U(p02)  ...  |
        //        |U(p10) U(p11) U(p12)  ...  |
        //        | ...    ...    ...   U(pnn)|
        //
        //     - P is a n-row 3-column matrix containing the number 1 (the constant term),
        //       x, and y (effectively, the p term):
        //
        //       |1 x1 y1|
        //       |1 x2 y2|
        //
        //     - PT is the transpose of P
        //     - 0 is the zero matrix (padding)
        //
        // 6. Use a linear solver to solve L * [w a] = [v o] to yield [w a]
        // 8. Return the coefficients, [w a]

        int const numPairs = static_cast<int>(landmarkPairs.size());

        if (numPairs == 0)
        {
            // edge-case: there are no pairs, so return an identity-like transform
            return TPSCoefficients2D{};
        }

        // construct matrix L
        SimTK::Matrix L(numPairs + 3, numPairs + 3);

        // populate the K part of matrix L (upper-left)
        for (int row = 0; row < numPairs; ++row)
        {
            for (int col = 0; col < numPairs; ++col)
            {
                glm::vec2 const& pi = landmarkPairs[row].src;
                glm::vec2 const& pj = landmarkPairs[col].src;

                L(row, col) = RadialBasisFunction2D(pi, pj);
            }
        }

        // populate the P part of matrix L (upper-right)
        {
            int const pStartColumn = numPairs;

            for (int row = 0; row < numPairs; ++row)
            {
                L(row, pStartColumn)     = 1.0;
                L(row, pStartColumn + 1) = landmarkPairs[row].src.x;
                L(row, pStartColumn + 2) = landmarkPairs[row].src.y;
            }
        }

        // populate the PT part of matrix L (bottom-left)
        {
            int const ptStartRow = numPairs;

            for (int col = 0; col < numPairs; ++col)
            {
                L(ptStartRow, col)     = 1.0;
                L(ptStartRow + 1, col) = landmarkPairs[col].src.x;
                L(ptStartRow + 2, col) = landmarkPairs[col].src.y;
            }
        }

        // populate the 0 part of matrix L (bottom-right)
        {
            int const zeroStartRow = numPairs;
            int const zeroStartCol = numPairs;

            for (int row = 0; row < 3; ++row)
            {
                for (int col = 0; col < 3; ++col)
                {
                    L(zeroStartRow + row, zeroStartCol + col) = 0.0;
                }
            }
        }

        // construct "result" vectors Vx and Vy (these hold the landmark destinations)
        SimTK::Vector Vx(numPairs + 3, 0.0);
        SimTK::Vector Vy(numPairs + 3, 0.0);
        for (int row = 0; row < numPairs; ++row)
        {
            Vx[row] = landmarkPairs[row].dest.x;
            Vy[row] = landmarkPairs[row].dest.y;
        }

        // construct coefficient vectors that will receive the solver's result
        SimTK::Vector Cx(numPairs + 3, 0.0);
        SimTK::Vector Cy(numPairs + 3, 0.0);

        // solve `L*Cx = Vx` and `L*Cy = Vy` for `Cx` and `Cy` (the coefficients)
        SimTK::FactorQTZ F(L);
        F.solve(Vx, Cx);
        F.solve(Vy, Cy);

        // the coefficient vectors now contain (e.g. for X): [w1, w2, ... wx, a0, a1x, a1y]
        //
        // extract them into the return value

        TPSCoefficients2D rv;

        // populate affine a1, a2, a3 terms
        rv.a1 = {Cx[numPairs],   Cy[numPairs]  };
        rv.a2 = {Cx[numPairs+1], Cy[numPairs+1]};
        rv.a3 = {Cx[numPairs+2], Cy[numPairs+2]};

        // populate `wi` coefficients (+ control points, needed at evaluation-time)
        rv.weights.reserve(numPairs);
        for (int i = 0; i < numPairs; ++i)
        {
            glm::vec2 weight = {Cx[i], Cy[i]};
            glm::vec2 controlPoint = landmarkPairs[i].src;
            rv.weights.emplace_back(weight, controlPoint);
        }

        return rv;
    }

    // a class that wraps the 2D TPS algorithm with a basic interface for transforming
    // points
    class ThinPlateWarper2D final {
    public:
        ThinPlateWarper2D(nonstd::span<LandmarkPair2D const> landmarkPairs) :
            m_Coefficients{CalcCoefficients(landmarkPairs)}
        {
        }

        glm::vec2 transform(glm::vec2 p) const
        {
            return Evaluate(m_Coefficients, p);
        }

    private:
        TPSCoefficients2D m_Coefficients;
    };

    // returns a mesh that is the equivalent of applying the 2D TPS warp to all
    // vertices of the input mesh
    osc::Mesh ApplyThinPlateWarpToMesh(ThinPlateWarper2D const& t, osc::Mesh const& mesh)
    {
        // load source points
        nonstd::span<glm::vec3 const> srcPoints = mesh.getVerts();

        // map each source point via the warper
        std::vector<glm::vec3> destPoints;
        destPoints.reserve(srcPoints.size());
        for (glm::vec3 const& srcPoint : srcPoints)
        {
            destPoints.emplace_back(t.transform(srcPoint), srcPoint.z);
        }

        // upload the new points into the returned mesh
        osc::Mesh rv = mesh;
        rv.setVerts(destPoints);
        return rv;
    }
}

// GUI stuff
namespace
{
    // holds the user's current mouse click state:
    //
    // - initial (the user did nothing with their mouse yet)
    // - first click (the user clicked the source of a landmark pair and the UI is waiting for the destination)
    struct GUIInitialMouseState final {};
    struct GUIFirstClickMouseState final { glm::vec2 srcNDCPos; };
    using GUIMouseState = std::variant<GUIInitialMouseState, GUIFirstClickMouseState>;
}

class osc::ThinPlateWarpTab::Impl final {
public:

    Impl(TabHost* parent) : m_Parent{std::move(parent)}
    {
        m_Material.setVec4("uColor", {0.0f, 0.0f, 0.0f, 1.0f});
        m_Camera.setViewMatrix(glm::mat4{1.0f});
        m_Camera.setProjectionMatrix(glm::mat4{1.0f});
        m_Camera.setBackgroundColor({1.0f, 1.0f, 1.0f, 1.0f});
    }

    UID getID() const
    {
        return m_ID;
    }

    CStringView getName() const
    {
        return m_Name;
    }

    TabHost* parent()
    {
        return m_Parent;
    }

    void onMount()
    {

    }

    void onUnmount()
    {

    }

    bool onEvent(SDL_Event const&)
    {
        return false;
    }

    void onTick()
    {

    }

    void onDrawMainMenu()
    {

    }

    void onDraw()
    {
        ImGui::DockSpaceOverViewport(ImGui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode);

        ImGui::Begin("Input");
        {
            glm::vec2 const windowDims = ImGui::GetContentRegionAvail();
            float const minDim = glm::min(windowDims.x, windowDims.y);
            glm::ivec2 const texDims = glm::ivec2{minDim, minDim};

            renderMesh(m_InputGrid, texDims, m_InputRender);

            // draw rendered texture via ImGui
            ImGuiImageHittestResult const ht = osc::DrawTextureAsImGuiImageAndHittest(*m_InputRender, texDims);

            // draw any 2D overlays etc.
            renderOverlayElements(ht);
            if (ht.isHovered)
            {
                renderMouseUIElements(ht);
            }
        }
        
        ImGui::End();

        ImGui::Begin("Output");
        {
            glm::vec2 const windowDims = ImGui::GetContentRegionAvail();
            float const minDim = glm::min(windowDims.x, windowDims.y);
            glm::ivec2 const texDims = glm::ivec2{minDim, minDim};

            ThinPlateWarper2D warper{m_LandmarkPairs};
            m_OutputGrid = ApplyThinPlateWarpToMesh(warper, m_InputGrid);

            renderMesh(m_OutputGrid, texDims, m_OutputRender);

            // draw rendered texture via ImGui
            osc::DrawTextureAsImGuiImage(*m_OutputRender, texDims);
        }
        ImGui::End();

        // draw log panel (debugging)
        m_LogViewerPanel.draw();
    }


private:

    // render the given mesh as-is to the given output render texture
    void renderMesh(Mesh const& mesh, glm::ivec2 dims, std::optional<RenderTexture>& out)
    {
        RenderTextureDescriptor desc{dims};
        desc.setAntialiasingLevel(App::get().getMSXAASamplesRecommended());
        out.emplace(desc);
        osc::Graphics::DrawMesh(mesh, osc::Transform{}, m_Material, m_Camera);
        m_Camera.swapTexture(out);
        m_Camera.render();
        m_Camera.swapTexture(out);

        OSC_ASSERT(out.has_value() && "the camera should've given the render texture back to the caller");
    }

    // render any 2D overlays
    void renderOverlayElements(ImGuiImageHittestResult const& ht)
    {
        ImDrawList* const drawlist = ImGui::GetWindowDrawList();

        // render all fully-established landmark pairs
        for (LandmarkPair2D const& p : m_LandmarkPairs)
        {
            glm::vec2 const p1 = ht.rect.p1 + (Dimensions(ht.rect) * NDCPointToTopLeftRelPos(p.src));
            glm::vec2 const p2 = ht.rect.p1 + (Dimensions(ht.rect) * NDCPointToTopLeftRelPos(p.dest));

            drawlist->AddLine(p1, p2, m_ConnectionLineColor, 5.0f);
            drawlist->AddCircleFilled(p1, 10.0f, m_SrcCircleColor);
            drawlist->AddCircleFilled(p2, 10.0f, m_DestCircleColor);
        }

        // render any currenty-placing landmark pairs in a more-faded color
        if (ht.isHovered && std::holds_alternative<GUIFirstClickMouseState>(m_MouseState))
        {
            GUIFirstClickMouseState const& st = std::get<GUIFirstClickMouseState>(m_MouseState);

            glm::vec2 const p1 = ht.rect.p1 + (Dimensions(ht.rect) * NDCPointToTopLeftRelPos(st.srcNDCPos));
            glm::vec2 const p2 = ImGui::GetMousePos();

            drawlist->AddLine(p1, p2, m_ConnectionLineColor, 5.0f);
            drawlist->AddCircleFilled(p1, 10.0f, m_SrcCircleColor);
            drawlist->AddCircleFilled(p2, 10.0f, m_DestCircleColor);
        }
    }

    // render any mouse-related overlays
    void renderMouseUIElements(ImGuiImageHittestResult const& ht)
    {
        std::visit(osc::Overload
        {
            [this, &ht](GUIInitialMouseState const& st) { renderMouseUIElements(ht, st); },
            [this, &ht](GUIFirstClickMouseState const& st) { renderMouseUIElements(ht, st); },
        }, m_MouseState);
    }

    // render any mouse-related overlays for when the user hasn't clicked yet
    void renderMouseUIElements(ImGuiImageHittestResult const& ht, GUIInitialMouseState st)
    {
        glm::vec2 const mouseScreenPos = ImGui::GetMousePos();
        glm::vec2 const mouseImagePos = mouseScreenPos - ht.rect.p1;
        glm::vec2 const mouseImageRelPos = mouseImagePos / Dimensions(ht.rect);
        glm::vec2 const mouseImageNDCPos = TopleftRelPosToNDCPoint(mouseImageRelPos);

        osc::DrawTooltipBodyOnly(StreamToString(mouseImageNDCPos).c_str());

        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        {
            m_MouseState = GUIFirstClickMouseState{mouseImageNDCPos};
        }
    }

    // render any mouse-related overlays for when the user has clicked once
    void renderMouseUIElements(ImGuiImageHittestResult const& ht, GUIFirstClickMouseState st)
    {
        glm::vec2 const mouseScreenPos = ImGui::GetMousePos();
        glm::vec2 const mouseImagePos = mouseScreenPos - ht.rect.p1;
        glm::vec2 const mouseImageRelPos = mouseImagePos / Dimensions(ht.rect);
        glm::vec2 const mouseImageNDCPos = TopleftRelPosToNDCPoint(mouseImageRelPos);

        osc::DrawTooltipBodyOnly((StreamToString(mouseImageNDCPos) + "*").c_str());

        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        {
            m_LandmarkPairs.push_back({st.srcNDCPos, mouseImageNDCPos});
            m_MouseState = GUIInitialMouseState{};
        }
    }

    // tab data
    UID m_ID;
    std::string m_Name = ICON_FA_BEZIER_CURVE " ThinPlateWarpTab";
    TabHost* m_Parent;

    // TPS algorithm state
    GUIMouseState m_MouseState = GUIInitialMouseState{};
    std::vector<LandmarkPair2D> m_LandmarkPairs;

    // GUI state (rendering, colors, etc.)
    Mesh m_InputGrid = GenNxMPoint2DGridWithConnectingLines({-1.0f, -1.0f}, {1.0f, 1.0f}, {50, 50});
    Mesh m_OutputGrid = m_InputGrid;
    Material m_Material = Material{App::shaders().get("shaders/SolidColor.vert", "shaders/SolidColor.frag")};
    Camera m_Camera;
    std::optional<RenderTexture> m_InputRender;
    std::optional<RenderTexture> m_OutputRender;
    ImU32 m_SrcCircleColor = ImGui::ColorConvertFloat4ToU32({1.0f, 0.0f, 0.0f, 1.0f});
    ImU32 m_DestCircleColor = ImGui::ColorConvertFloat4ToU32({0.0f, 1.0f, 0.0f, 1.0f});
    ImU32 m_ConnectionLineColor = ImGui::ColorConvertFloat4ToU32({0.0f, 0.0f, 0.0f, 0.6f});

    // log panel (handy for debugging)
    LogViewerPanel m_LogViewerPanel{"Log"};
};


// public API (PIMPL)

osc::ThinPlateWarpTab::ThinPlateWarpTab(TabHost* parent) :
    m_Impl{new Impl{std::move(parent)}}
{
}

osc::ThinPlateWarpTab::ThinPlateWarpTab(ThinPlateWarpTab&& tmp) noexcept :
    m_Impl{std::exchange(tmp.m_Impl, nullptr)}
{
}

osc::ThinPlateWarpTab& osc::ThinPlateWarpTab::operator=(ThinPlateWarpTab&& tmp) noexcept
{
    std::swap(m_Impl, tmp.m_Impl);
    return *this;
}

osc::ThinPlateWarpTab::~ThinPlateWarpTab() noexcept
{
    delete m_Impl;
}

osc::UID osc::ThinPlateWarpTab::implGetID() const
{
    return m_Impl->getID();
}

osc::CStringView osc::ThinPlateWarpTab::implGetName() const
{
    return m_Impl->getName();
}

osc::TabHost* osc::ThinPlateWarpTab::implParent() const
{
    return m_Impl->parent();
}

void osc::ThinPlateWarpTab::implOnMount()
{
    m_Impl->onMount();
}

void osc::ThinPlateWarpTab::implOnUnmount()
{
    m_Impl->onUnmount();
}

bool osc::ThinPlateWarpTab::implOnEvent(SDL_Event const& e)
{
    return m_Impl->onEvent(e);
}

void osc::ThinPlateWarpTab::implOnTick()
{
    m_Impl->onTick();
}

void osc::ThinPlateWarpTab::implOnDrawMainMenu()
{
    m_Impl->onDrawMainMenu();
}

void osc::ThinPlateWarpTab::implOnDraw()
{
    m_Impl->onDraw();
}
