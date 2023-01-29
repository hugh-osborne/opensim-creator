#include "OpenSimHelpers.hpp"

#include "src/Maths/MathHelpers.hpp"
#include "src/OpenSimBindings/SimTKHelpers.hpp"
#include "src/OpenSimBindings/UndoableModelStatePair.hpp"
#include "src/Platform/Log.hpp"
#include "src/Utils/Algorithms.hpp"
#include "src/Utils/Cpp20Shims.hpp"
#include "src/Utils/CStringView.hpp"
#include "src/Utils/Perf.hpp"

#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>
#include <nonstd/span.hpp>
#include <OpenSim/Common/Component.h>
#include <OpenSim/Common/ComponentList.h>
#include <OpenSim/Common/ComponentPath.h>
#include <OpenSim/Common/ComponentSocket.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Property.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/Model/AbstractPathPoint.h>
#include <OpenSim/Simulation/Model/Appearance.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/ConstraintSet.h>
#include <OpenSim/Simulation/Model/ContactGeometry.h>
#include <OpenSim/Simulation/Model/ContactGeometrySet.h>
#include <OpenSim/Simulation/Model/ControllerSet.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/Frame.h>
#include <OpenSim/Simulation/Model/Geometry.h>
#include <OpenSim/Simulation/Model/GeometryPath.h>
#include <OpenSim/Simulation/Model/JointSet.h>
#include <OpenSim/Simulation/Model/Marker.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/PathPoint.h>
#include <OpenSim/Simulation/Model/PathPointSet.h>
#include <OpenSim/Simulation/Model/Probe.h>
#include <OpenSim/Simulation/Model/ProbeSet.h>
#include <OpenSim/Simulation/Model/Station.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/SimbodyEngine/Constraint.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include <OpenSim/Simulation/Wrap/PathWrap.h>
#include <OpenSim/Simulation/Wrap/PathWrapPoint.h>
#include <OpenSim/Simulation/Wrap/WrapObject.h>
#include <OpenSim/Simulation/Wrap/WrapObjectSet.h>
#include <SimTKcommon.h>
#include <SimTKcommon/SmallMatrix.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ostream>
#include <sstream>
#include <utility>
#include <vector>

// helpers
namespace
{
    // try to delete an item from an OpenSim::Set
    //
    // returns `true` if the item was found and deleted; otherwise, returns `false`
    template<typename T, typename TSetBase = OpenSim::Object>
    bool TryDeleteItemFromSet(OpenSim::Set<T, TSetBase>& set, T const* item)
    {
        for (int i = 0; i < set.getSize(); ++i)
        {
            if (&set.get(i) == item)
            {
                set.remove(i);
                return true;
            }
        }
        return false;
    }
}

static bool IsConnectedViaSocketTo(OpenSim::Component& c, OpenSim::Component const& other)
{
    for (std::string const& socketName : c.getSocketNames())
    {
        OpenSim::AbstractSocket const& sock = c.getSocket(socketName);
        if (sock.isConnected() && &sock.getConnecteeAsObject() == &other)
        {
            return true;
        }
    }
    return false;
}

static std::vector<OpenSim::Component*> GetAnyComponentsConnectedViaSocketTo(OpenSim::Component& root, OpenSim::Component const& other)
{
    std::vector<OpenSim::Component*> rv;

    if (IsConnectedViaSocketTo(root, other))
    {
        rv.push_back(&root);
    }

    for (OpenSim::Component& c : root.updComponentList())
    {
        if (IsConnectedViaSocketTo(c, other))
        {
            rv.push_back(&c);
        }
    }

    return rv;
}


// public API

bool osc::IsConcreteClassNameLexographicallyLowerThan(OpenSim::Component const& a, OpenSim::Component const& b)
{
    return a.getConcreteClassName() < b.getConcreteClassName();
}

bool osc::IsNameLexographicallyLowerThan(OpenSim::Component const& a, OpenSim::Component const& b)
{
    return a.getName() < b.getName();
}

OpenSim::Component* osc::UpdOwner(OpenSim::Component& c)
{
    return c.hasOwner() ? const_cast<OpenSim::Component*>(&c.getOwner()) : nullptr;
}

OpenSim::Component const* osc::GetOwner(OpenSim::Component const& c)
{
    return c.hasOwner() ? &c.getOwner() : nullptr;
}


int osc::DistanceFromRoot(OpenSim::Component const& c)
{
    OpenSim::Component const* p = &c;
    int dist = 0;

    while (p->hasOwner())
    {
        ++dist;
        p = &p->getOwner();
    }

    return dist;
}

OpenSim::ComponentPath const& osc::GetEmptyComponentPath()
{
    static OpenSim::ComponentPath p;
    return p;
}

OpenSim::ComponentPath const& osc::GetRootComponentPath()
{
    static OpenSim::ComponentPath p{"/"};
    return p;
}

bool osc::IsEmpty(OpenSim::ComponentPath const& cp)
{
    return cp == GetEmptyComponentPath();
}

void osc::Clear(OpenSim::ComponentPath& cp)
{
    cp = GetEmptyComponentPath();
}

std::vector<OpenSim::Component const*> osc::GetPathElements(OpenSim::Component const& c)
{
    std::vector<OpenSim::Component const*> rv;
    rv.reserve(DistanceFromRoot(c));

    OpenSim::Component const* p = &c;
    rv.push_back(p);

    while (p->hasOwner())
    {
        p = &p->getOwner();
        rv.push_back(p);
    }

    std::reverse(rv.begin(), rv.end());

    return rv;
}

bool osc::IsInclusiveChildOf(OpenSim::Component const* parent, OpenSim::Component const* c)
{
    if (!c)
    {
        return false;
    }

    if (!parent)
    {
        return false;
    }

    for (;;)
    {
        if (c == parent)
        {
            return true;
        }

        if (!c->hasOwner())
        {
            return false;
        }

        c = &c->getOwner();
    }
}

OpenSim::Component const* osc::IsInclusiveChildOf(nonstd::span<OpenSim::Component const*> parents, OpenSim::Component const* c)
{
    while (c)
    {
        for (size_t i = 0; i < parents.size(); ++i)
        {
            if (c == parents[i])
            {
                return parents[i];
            }
        }
        c = c->hasOwner() ? &c->getOwner() : nullptr;
    }
    return nullptr;
}

OpenSim::Component const* osc::FindFirstAncestorInclusive(OpenSim::Component const* c, bool(*pred)(OpenSim::Component const*))
{
    while (c)
    {
        if (pred(c))
        {
            return c;
        }
        c = c->hasOwner() ? &c->getOwner() : nullptr;
    }
    return nullptr;
}

std::vector<OpenSim::Coordinate const*> osc::GetCoordinatesInModel(OpenSim::Model const& model)
{
    std::vector<OpenSim::Coordinate const*> rv;
    GetCoordinatesInModel(model, rv);
    return rv;
}

void osc::GetCoordinatesInModel(OpenSim::Model const& m,
                                std::vector<OpenSim::Coordinate const*>& out)
{
    OpenSim::CoordinateSet const& s = m.getCoordinateSet();
    int len = s.getSize();
    out.reserve(out.size() + static_cast<size_t>(len));

    for (int i = 0; i < len; ++i)
    {
        out.push_back(&s[i]);
    }
}

float osc::ConvertCoordValueToDisplayValue(OpenSim::Coordinate const& c, double v)
{
    float rv = static_cast<float>(v);

    if (c.getMotionType() == OpenSim::Coordinate::MotionType::Rotational)
    {
        rv = glm::degrees(rv);
    }

    return rv;
}

double osc::ConvertCoordDisplayValueToStorageValue(OpenSim::Coordinate const& c, float v)
{
    double rv = static_cast<double>(v);

    if (c.getMotionType() == OpenSim::Coordinate::MotionType::Rotational)
    {
        rv = glm::radians(rv);
    }

    return rv;
}

osc::CStringView osc::GetCoordDisplayValueUnitsString(OpenSim::Coordinate const& c)
{
    switch (c.getMotionType()) {
    case OpenSim::Coordinate::MotionType::Translational:
        return "m";
    case OpenSim::Coordinate::MotionType::Rotational:
        return "deg";
    default:
        return "";
    }
}

std::vector<std::string> osc::GetSocketNames(OpenSim::Component const& c)
{
    // const_cast is necessary because `getSocketNames` is somehow not-`const`
    return const_cast<OpenSim::Component&>(c).getSocketNames();
}

std::vector<OpenSim::AbstractSocket const*> osc::GetAllSockets(OpenSim::Component const& c)
{
    std::vector<OpenSim::AbstractSocket const*> rv;

    for (std::string const& name : GetSocketNames(c))
    {
        OpenSim::AbstractSocket const& sock = c.getSocket(name);
        rv.push_back(&sock);
    }

    return rv;
}

OpenSim::Component const* osc::FindComponent(OpenSim::Component const& c, OpenSim::ComponentPath const& cp)
{
    if (IsEmpty(cp))
    {
        return nullptr;
    }

    try
    {
        return &c.getComponent(cp);
    }
    catch (OpenSim::Exception const&)
    {
        return nullptr;
    }
}

OpenSim::Component* osc::FindComponentMut(OpenSim::Component& c, OpenSim::ComponentPath const& cp)
{
    return const_cast<OpenSim::Component*>(FindComponent(c, cp));
}

bool osc::ContainsComponent(OpenSim::Component const& root, OpenSim::ComponentPath const& cp)
{
    return FindComponent(root, cp);
}

OpenSim::AbstractSocket const* osc::FindSocket(OpenSim::Component const& c, std::string const& name)
{
    try
    {
        return &c.getSocket(name);
    }
    catch (OpenSim::SocketNotFound const&)
    {
        return nullptr;  // :(
    }
}

OpenSim::AbstractSocket* osc::FindSocketMut(OpenSim::Component& c, std::string const& name)
{
    try
    {
        return &c.updSocket(name);
    }
    catch (OpenSim::SocketNotFound const&)
    {
        return nullptr;  // :(
    }
}

bool osc::IsAbleToConnectTo(OpenSim::AbstractSocket const& s, OpenSim::Component const& c)
{
    // yes, this is very very bad

    std::unique_ptr<OpenSim::AbstractSocket> copy{s.clone()};
    try
    {
        copy->connect(c);
        return true;
    }
    catch (OpenSim::Exception const&)
    {
        return false;
    }
}

OpenSim::AbstractProperty* osc::FindPropertyMut(OpenSim::Component& c, std::string const& name)
{
    return c.hasProperty(name) ? &c.updPropertyByName(name) : nullptr;
}

OpenSim::AbstractOutput const* osc::FindOutput(OpenSim::Component const& c, std::string const& outputName)
{
    OpenSim::AbstractOutput const* rv = nullptr;
    try
    {
        rv = &c.getOutput(outputName);
    }
    catch (OpenSim::Exception const&)
    {
        // OpenSim, innit
    }
    return rv;
}

OpenSim::AbstractOutput const* osc::FindOutput(OpenSim::Component const& root,
                                               OpenSim::ComponentPath const& path,
                                               std::string const& outputName)
{
    OpenSim::Component const* c = FindComponent(root, path);
    return c ? FindOutput(*c, outputName) : nullptr;
}

bool osc::HasInputFileName(OpenSim::Model const& m)
{
    std::string const& name = m.getInputFileName();
    return !name.empty() && name != "Unassigned";
}

std::filesystem::path osc::TryFindInputFile(OpenSim::Model const& m)
{
    if (!HasInputFileName(m))
    {
        return {};
    }

    std::filesystem::path p{m.getInputFileName()};

    if (!std::filesystem::exists(p))
    {
        return {};
    }

    return p;
}

std::optional<std::filesystem::path> osc::FindGeometryFileAbsPath(
    OpenSim::Model const& model,
    OpenSim::Mesh const& mesh)
{
    // this implementation is designed to roughly mimic how OpenSim::Mesh::extendFinalizeFromProperties works

    std::string const& fileProp = mesh.get_mesh_file();
    std::filesystem::path const filePropPath{fileProp};

    bool isAbsolute = filePropPath.is_absolute();
    SimTK::Array_<std::string> attempts;
    bool const found = OpenSim::ModelVisualizer::findGeometryFile(
        model,
        fileProp,
        isAbsolute,
        attempts
    );

    if (!found || attempts.empty())
    {
        return std::nullopt;
    }

    return std::optional<std::filesystem::path>{std::filesystem::absolute({attempts.back()})};
}

bool osc::ShouldShowInUI(OpenSim::Component const& c)
{
    if (Is<OpenSim::PathWrapPoint>(c))
    {
        return false;
    }
    else if (Is<OpenSim::Station>(c) && c.hasOwner() && DerivesFrom<OpenSim::PathPoint>(c.getOwner()))
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool osc::TryDeleteComponentFromModel(OpenSim::Model& m, OpenSim::Component& c)
{
    OpenSim::Component* owner = osc::UpdOwner(c);

    if (!owner)
    {
        log::error("cannot delete %s: it has no owner", c.getName().c_str());
        return false;
    }

    if (&c.getRoot() != &m)
    {
        log::error("cannot delete %s: it is not owned by the provided model");
        return false;
    }

    // check if anything connects to the component via a socket
    if (auto connectees = GetAnyComponentsConnectedViaSocketTo(m, c); !connectees.empty())
    {
        std::stringstream ss;
        char const* delim = "";
        for (OpenSim::Component const* connectee : connectees)
        {
            ss << delim << connectee->getName();
            delim = ", ";
        }
        log::error("cannot delete %s: the following components connect to it via sockets: %s", c.getName().c_str(), std::move(ss).str().c_str());
        return false;
    }

    // BUG/HACK: check if any path wraps connect to the component
    //
    // this is because the wrapping code isn't using sockets :< - this should be
    // fixed in OpenSim itself
    for (OpenSim::PathWrap const& pw : m.getComponentList<OpenSim::PathWrap>())
    {
        if (pw.getWrapObject() == &c)
        {
            log::error("cannot delete %s: it is used in a path wrap (%s)", c.getName().c_str(), osc::GetAbsolutePathString(pw).c_str());
            return false;
        }
    }

    // at this point we know that it's *technically* feasible to delete the component
    // from the model without breaking sockets etc., so now we use heuristics to figure
    // out how to do that

    bool rv = false;

    if (auto* js = dynamic_cast<OpenSim::JointSet*>(owner))
    {
        rv = TryDeleteItemFromSet(*js, dynamic_cast<OpenSim::Joint*>(&c));
    }
    else if (auto* bs = dynamic_cast<OpenSim::BodySet*>(owner))
    {
        rv = TryDeleteItemFromSet(*bs, dynamic_cast<OpenSim::Body*>(&c));
    }
    else if (auto* wos = dynamic_cast<OpenSim::WrapObjectSet*>(owner))
    {
        rv = TryDeleteItemFromSet(*wos, dynamic_cast<OpenSim::WrapObject*>(&c));
    }
    else if (auto* cs = dynamic_cast<OpenSim::ControllerSet*>(owner))
    {
        rv = TryDeleteItemFromSet(*cs, dynamic_cast<OpenSim::Controller*>(&c));
    }
    else if (auto* conss = dynamic_cast<OpenSim::ConstraintSet*>(owner))
    {
        rv = TryDeleteItemFromSet(*conss, dynamic_cast<OpenSim::Constraint*>(&c));
    }
    else if (auto* fs = dynamic_cast<OpenSim::ForceSet*>(owner))
    {
        rv = TryDeleteItemFromSet(*fs, dynamic_cast<OpenSim::Force*>(&c));
    }
    else if (auto* ms = dynamic_cast<OpenSim::MarkerSet*>(owner))
    {
        rv = TryDeleteItemFromSet(*ms, dynamic_cast<OpenSim::Marker*>(&c));
    }
    else if (auto* cgs = dynamic_cast<OpenSim::ContactGeometrySet*>(owner); cgs)
    {
        rv = TryDeleteItemFromSet(*cgs, dynamic_cast<OpenSim::ContactGeometry*>(&c));
    }
    else if (auto* ps = dynamic_cast<OpenSim::ProbeSet*>(owner))
    {
        rv = TryDeleteItemFromSet(*ps, dynamic_cast<OpenSim::Probe*>(&c));
    }
    else if (auto* gp = dynamic_cast<OpenSim::GeometryPath*>(owner))
    {
        if (auto* app = dynamic_cast<OpenSim::AbstractPathPoint*>(&c))
        {
            rv = TryDeleteItemFromSet(gp->updPathPointSet(), app);
        }
        else if (auto* pw = dynamic_cast<OpenSim::PathWrap*>(&c))
        {
            rv = TryDeleteItemFromSet(gp->updWrapSet(), pw);
        }
    }
    else if (auto* geom = FindAncestorWithTypeMut<OpenSim::Geometry>(&c))
    {
        // delete an OpenSim::Geometry from its owning OpenSim::Frame

        if (auto* frame = FindAncestorWithTypeMut<OpenSim::Frame>(geom))
        {
            // its owner is a frame, which holds the geometry in a list property

            // make a copy of the property containing the geometry and
            // only copy over the not-deleted geometry into the copy
            //
            // this is necessary because OpenSim::Property doesn't seem
            // to support list element deletion, but does support full
            // assignment

            OpenSim::ObjectProperty<OpenSim::Geometry>& prop =
                static_cast<OpenSim::ObjectProperty<OpenSim::Geometry>&>(frame->updProperty_attached_geometry());

            std::unique_ptr<OpenSim::ObjectProperty<OpenSim::Geometry>> copy{prop.clone()};
            copy->clear();
            for (int i = 0; i < prop.size(); ++i)
            {
                OpenSim::Geometry& g = prop[i];
                if (&g != geom)
                {
                    copy->adoptAndAppendValue(g.clone());
                }
            }

            prop.assign(*copy);

            rv = true;
        }
    }

    if (!rv)
    {
        osc::log::error("cannot delete %s: OpenSim Creator doesn't know how to delete a %s from its parent (maybe it can't?)", c.getName().c_str(), c.getConcreteClassName().c_str());
    }

    return rv;
}

void osc::CopyCommonJointProperties(OpenSim::Joint const& src, OpenSim::Joint& dest)
{
    dest.setName(src.getName());

    // copy owned frames
    dest.updProperty_frames().assign(src.getProperty_frames());

    // copy parent frame socket *path* (note: don't use connectSocket, pointers are evil in model manipulations)
    dest.updSocket("parent_frame").setConnecteePath(src.getSocket("parent_frame").getConnecteePath());

    // copy child socket *path* (note: don't use connectSocket, pointers are evil in model manipulations)
    dest.updSocket("child_frame").setConnecteePath(src.getSocket("child_frame").getConnecteePath());
}

bool osc::DeactivateAllWrapObjectsIn(OpenSim::Model& m)
{
    bool rv = false;
    for (OpenSim::WrapObjectSet& wos : m.updComponentList<OpenSim::WrapObjectSet>())
    {
        for (int i = 0; i < wos.getSize(); ++i)
        {
            OpenSim::WrapObject& wo = wos[i];
            wo.set_active(false);
            wo.upd_Appearance().set_visible(false);
            rv = rv || true;
        }
    }
    return rv;
}

bool osc::ActivateAllWrapObjectsIn(OpenSim::Model& m)
{
    bool rv = false;
    for (OpenSim::WrapObjectSet& wos : m.updComponentList<OpenSim::WrapObjectSet>())
    {
        for (int i = 0; i < wos.getSize(); ++i)
        {
            OpenSim::WrapObject& wo = wos[i];
            wo.set_active(true);
            wo.upd_Appearance().set_visible(true);
            rv = rv || true;
        }
    }
    return rv;
}

void osc::AddComponentToModel(OpenSim::Model& m, std::unique_ptr<OpenSim::Component> c)
{
    if (!c)
    {
        return;  // paranoia
    }
    else if (dynamic_cast<OpenSim::Body*>(c.get()))
    {
        m.addBody(static_cast<OpenSim::Body*>(c.release()));
    }
    else if (OpenSim::Joint* j = dynamic_cast<OpenSim::Joint*>(c.get()))
    {
        // HOTFIX: `OpenSim::Ground` should never be listed as a joint's parent, because it
        //         causes a segfault in OpenSim 4.4 (#543)
        if (dynamic_cast<OpenSim::Ground const*>(&j->getChildFrame()))
        {
            throw std::runtime_error{"cannot create a new joint with 'ground' as the child: did you mix up parent/child?"};
        }

        m.addJoint(static_cast<OpenSim::Joint*>(c.release()));
    }
    else if (dynamic_cast<OpenSim::Constraint*>(c.get()))
    {
        m.addConstraint(static_cast<OpenSim::Constraint*>(c.release()));
    }
    else if (dynamic_cast<OpenSim::Force*>(c.get()))
    {
        m.addForce(static_cast<OpenSim::Force*>(c.release()));
    }
    else if (dynamic_cast<OpenSim::Probe*>(c.get()))
    {
        m.addProbe(static_cast<OpenSim::Probe*>(c.release()));
    }
    else if (dynamic_cast<OpenSim::ContactGeometry*>(c.get()))
    {
        m.addContactGeometry(static_cast<OpenSim::ContactGeometry*>(c.release()));
    }
    else if (dynamic_cast<OpenSim::Marker*>(c.get()))
    {
        m.addMarker(static_cast<OpenSim::Marker*>(c.release()));
    }
    else if (dynamic_cast<OpenSim::Controller*>(c.get()))
    {
        m.addController(static_cast<OpenSim::Controller*>(c.release()));
    }
    else
    {
        m.addComponent(c.release());
    }

    m.finalizeConnections();  // necessary, because adding it may have created a new (not finalized) connection
}

std::unique_ptr<osc::UndoableModelStatePair> osc::LoadOsimIntoUndoableModel(std::filesystem::path const& p)
{
    return std::make_unique<osc::UndoableModelStatePair>(p);
}

void osc::InitializeModel(OpenSim::Model& model)
{
    OSC_PERF("osc::InitializeModel");
    model.finalizeFromProperties();  // clears potentially-stale member components (required for `clearConnections`)
    model.clearConnections();        // clears any potentially stale pointers that can be retained by OpenSim::Socket<T> (see #263)
    model.buildSystem();             // creates a new underlying physics system
}

SimTK::State& osc::InitializeState(OpenSim::Model& model)
{
    OSC_PERF("osc::InitializeState");
    SimTK::State& state = model.initializeState();  // creates+returns a new working state
    model.equilibrateMuscles(state);
    model.realizeDynamics(state);
    return state;
}

std::optional<int> osc::FindJointInParentJointSet(OpenSim::Joint const& joint)
{
    auto const* parentJointset =
        joint.hasOwner() ? dynamic_cast<OpenSim::JointSet const*>(&joint.getOwner()) : nullptr;

    if (!parentJointset)
    {
        // it's a joint, but it's not owned by a JointSet, so the implementation cannot switch
        // the joint type
        return std::nullopt;
    }

    OpenSim::JointSet const& js = *parentJointset;

    for (int i = 0; i < js.getSize(); ++i)
    {
        OpenSim::Joint const* j = &js[i];
        if (j == &joint)
        {
            return i;
        }
    }
    return std::nullopt;
}

std::string osc::GetRecommendedDocumentName(osc::UndoableModelStatePair const& uim)
{
    if (uim.hasFilesystemLocation())
    {
        return uim.getFilesystemPath().filename().string();
    }
    else
    {
        return "untitled.osim";
    }
}

std::string osc::GetDisplayName(OpenSim::Geometry const& g)
{
    if (OpenSim::Mesh const* mesh = dynamic_cast<OpenSim::Mesh const*>(&g); mesh)
    {
        return mesh->getGeometryFilename();
    }
    else
    {
        return g.getConcreteClassName();
    }
}

char const* osc::GetMotionTypeDisplayName(OpenSim::Coordinate const& c)
{
    switch (c.getMotionType()) {
    case OpenSim::Coordinate::MotionType::Rotational:
        return "Rotational";
    case OpenSim::Coordinate::MotionType::Translational:
        return "Translational";
    case OpenSim::Coordinate::MotionType::Coupled:
        return "Coupled";
    default:
        return "Unknown";
    }
}

bool osc::TrySetAppearancePropertyIsVisibleTo(OpenSim::Component& c, bool v)
{
    if (!c.hasProperty("Appearance"))
    {
        return false;
    }
    OpenSim::AbstractProperty& p = c.updPropertyByName("Appearance");
    auto* maybeAppearance = dynamic_cast<OpenSim::Property<OpenSim::Appearance>*>(&p);
    if (!maybeAppearance)
    {
        return false;
    }
    maybeAppearance->updValue().set_visible(v);
    return true;
}

glm::vec4 osc::GetSuggestedBoneColor() noexcept
{
    glm::vec4 usualDefault = {232.0f / 255.0f, 216.0f / 255.0f, 200.0f/255.0f, 1.0f};
    glm::vec4 white = {1.0f, 1.0f, 1.0f, 1.0f};
    float brightenAmount = 0.1f;
    return glm::mix(usualDefault, white, brightenAmount);
}

bool osc::IsShowingFrames(OpenSim::Model const& model)
{
    return model.getDisplayHints().get_show_frames();
}

bool osc::ToggleShowingFrames(OpenSim::Model& model)
{
    bool const newValue = !IsShowingFrames(model);
    model.updDisplayHints().set_show_frames(newValue);
    return newValue;
}

bool osc::IsShowingMarkers(OpenSim::Model const& model)
{
    return model.getDisplayHints().get_show_markers();
}

bool osc::ToggleShowingMarkers(OpenSim::Model& model)
{
    bool const newValue = !IsShowingMarkers(model);
    model.updDisplayHints().set_show_markers(newValue);
    return newValue;
}

bool osc::IsShowingWrapGeometry(OpenSim::Model const& model)
{
    return model.getDisplayHints().get_show_wrap_geometry();
}

bool osc::ToggleShowingWrapGeometry(OpenSim::Model& model)
{
    bool const newValue = !IsShowingWrapGeometry(model);
    model.updDisplayHints().set_show_wrap_geometry(newValue);
    return newValue;
}

bool osc::IsShowingContactGeometry(OpenSim::Model const& model)
{
    return model.getDisplayHints().get_show_contact_geometry();
}

bool osc::ToggleShowingContactGeometry(OpenSim::Model& model)
{
    bool const newValue = !IsShowingContactGeometry(model);
    model.updDisplayHints().set_show_contact_geometry(newValue);
    return newValue;
}

void osc::GetAbsolutePathString(OpenSim::Component const& c, std::string& out)
{
    static constexpr ptrdiff_t c_MaxEls = 16;

    ptrdiff_t nEls = 0;
    std::array<OpenSim::Component const*, c_MaxEls> els;
    OpenSim::Component const* cur = &c;
    OpenSim::Component const* next = osc::GetOwner(*cur);

    if (!next)
    {
        // edge-case: caller provided a root
        out = '/';
        return;
    }

    while (cur && next && nEls < c_MaxEls)
    {
        els[nEls++] = cur;
        cur = next;
        next = osc::GetOwner(*cur);
    }

    if (nEls >= c_MaxEls)
    {
        // edge-case: component is too deep: fallback to OpenSim impl.
        out = c.getAbsolutePathString();
        return;
    }

    // else: construct the path piece-by-piece

    // precompute path length (memory allocation)
    size_t pathlen = nEls;
    for (ptrdiff_t i = 0; i < nEls; ++i)
    {
        pathlen += els[i]->getName().size();
    }

    // then preallocate the string
    out.resize(pathlen);

    // and assign it
    size_t loc = 0;
    for (ptrdiff_t i = nEls-1; i >= 0; --i)
    {
        out[loc++] = '/';
        std::string const& name = els[i]->getName();
        std::copy(name.begin(), name.end(), out.begin() + loc);
        loc += name.size();
    }
}

std::string osc::GetAbsolutePathString(OpenSim::Component const& c)
{
    std::string rv;
    GetAbsolutePathString(c, rv);
    return rv;
}

OpenSim::ComponentPath osc::GetAbsolutePath(OpenSim::Component const& c)
{
    return OpenSim::ComponentPath{GetAbsolutePathString(c)};
}

OpenSim::ComponentPath osc::GetAbsolutePathOrEmpty(OpenSim::Component const* c)
{
    if (c)
    {
        return osc::GetAbsolutePath(*c);
    }
    else
    {
        return OpenSim::ComponentPath{};
    }
}