//=============================================================================
// INCLUDES
//=============================================================================
#include "SynapseConnection.h"
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
SynapseConnection::SynapseConnection() :
    ModelComponent{} {
    constructProperties();
}

SynapseConnection::~SynapseConnection() = default;

void SynapseConnection::constructProperties() {
    setAuthors("Hugh Osborne");
    constructProperty_enabled(true);
}

//=============================================================================
// GET AND SET
//=============================================================================
bool SynapseConnection::isEnabled() const {
    return get_enabled();
}

void SynapseConnection::setEnabled(bool aTrueFalse) {
    upd_enabled() = aTrueFalse;
}

//==============================================================================
// MODELCOMPONENT INTERFACE REQUIREMENTS
//==============================================================================

void SynapseConnection::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    this->_neurons = addCacheVariable("neurons", std::vector<LIF_Neuron>(get_num_neurons()), SimTK::Stage::Dynamics);
}

void SynapseConnection::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);

    auto& neurons = updNeurons(s);
    for (auto& n : neurons) {
        n.membrane_potential = -70.0;
        n.prev_time = 0.0;
        n.refractory_period = 0.02;
        n.resting_potential = -70.0;
        n.tau = 1.0;
        n.threshold_potential = -50.0;
    }
    markCacheVariableValid(s, _neurons);
}