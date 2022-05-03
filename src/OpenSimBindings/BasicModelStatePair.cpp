#include "BasicModelStatePair.hpp"

#include "src/OpenSimBindings/OpenSimHelpers.hpp"
#include "src/Utils/Assertions.hpp"
#include "src/Utils/Perf.hpp"

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>

static void InitModel(OpenSim::Model& m)
{
    OSC_PERF("model update");
    m.buildSystem();
    m.initializeState();
}

class osc::BasicModelStatePair::Impl final {
public:

    Impl(OpenSim::Model const& m, SimTK::State const& st) :
        m_Model(std::make_unique<OpenSim::Model>(m))
    {
        InitModel(*m_Model);
        OSC_ASSERT(m_Model->getWorkingState().isConsistent(st));
        m_Model->updWorkingState() = st;
    }

    Impl(Impl const& o) :
        m_Model{std::make_unique<OpenSim::Model>(*o.m_Model)}
    {
        InitModel(*m_Model);
        OSC_ASSERT(m_Model->getWorkingState().isConsistent(o.m_Model->getWorkingState()));
        m_Model->updWorkingState() = o.m_Model->getWorkingState();
    }

    std::unique_ptr<Impl> clone()
    {
        return std::make_unique<Impl>(*this);
    }

    OpenSim::Model const& getModel() const
    {
        return *m_Model;
    }

    OpenSim::Model& updModel()
    {
        return *m_Model;
    }

    SimTK::State const& getState() const
    {
        return m_Model->getWorkingState();
    }

private:
    std::unique_ptr<OpenSim::Model> m_Model;
};


// public API

osc::BasicModelStatePair::BasicModelStatePair(OpenSim::Model const& model, SimTK::State const& state) :
    m_Impl{new Impl{model, state}}
{
}
osc::BasicModelStatePair::BasicModelStatePair(BasicModelStatePair const&) = default;
osc::BasicModelStatePair::BasicModelStatePair(BasicModelStatePair&&) noexcept = default;
osc::BasicModelStatePair& osc::BasicModelStatePair::operator=(BasicModelStatePair const&) = default;
osc::BasicModelStatePair& osc::BasicModelStatePair::operator=(BasicModelStatePair&&) noexcept = default;
osc::BasicModelStatePair::~BasicModelStatePair() noexcept = default;

OpenSim::Model const& osc::BasicModelStatePair::getModel() const
{
    return m_Impl->getModel();
}

OpenSim::Model& osc::BasicModelStatePair::updModel()
{
    return m_Impl->updModel();
}

SimTK::State const& osc::BasicModelStatePair::getState() const
{
    return m_Impl->getState();
}
