//=============================================================================
// INCLUDES
//=============================================================================
#include "MotorUnitGroup.h"
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace OpenSim;
using namespace std;

//=============================================================================
// STATE VARIABLE NAMES
//=============================================================================
const std::string MotorUnitGroup::STATE_MEAN_MEMBRANE_POTENTIAL_NAME = "mean_membrane_potential";
const std::string MotorUnitGroup::STATE_MEAN_FIRING_RATE_NAME = "mean_firing_rate";
const std::string MotorUnitGroup::STATE_MUSCLE_EXCITATION_NAME= "muscle_excitation";

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
MotorUnitGroup::MotorUnitGroup() :
        ModelComponent{}{
    constructProperties();
}

MotorUnitGroup::~MotorUnitGroup() = default;

void MotorUnitGroup::constructProperties() {
    setAuthors("Hugh Osborne based on work by Ajay Seth, Frank Anderson, Chand John, Samuel Hamner");
    constructProperty_enabled(true);

    constructProperty_num_neurons(50);

    constructProperty_tau_mean(1.0);
    constructProperty_tau_var(0.0);
    constructProperty_rest_mean(-70.0);
    constructProperty_rest_var(0.0);
    constructProperty_threshold_mean(-50.0);
    constructProperty_threshold_var(0.0);
    constructProperty_refractory_mean(0.02); 
    constructProperty_refractory_var(0.0);
}

//=============================================================================
// GET AND SET
//=============================================================================
bool MotorUnitGroup::isEnabled() const {
    return get_enabled();
}

void MotorUnitGroup::setEnabled(bool aTrueFalse) {
    upd_enabled() = aTrueFalse;
}

double MotorUnitGroup::getMeanMembranePotential(const SimTK::State& s) const {
    auto neurons = getNeurons(s);
    double potential = 0.0;
    for (auto& n : neurons)
        potential += n.membrane_potential;
    return potential / neurons.size();
}

double MotorUnitGroup::getMeanFiringRate(const SimTK::State& s) const {
    return getMeanMembranePotential(s) * 0.0;
}

double MotorUnitGroup::getMuscleExcitation(const SimTK::State& s) const {
    return (getMeanMembranePotential(s) - get_rest_mean()) * 5.0;
}

void MotorUnitGroup::updateNeurons(const SimTK::State& s, std::vector<MotorUnitGroup::LIF_Neuron>&) const {
    auto neurons = updCacheVariableValue(s, _neurons);
    for (auto& n : neurons) {
        double dt = s.getTime() - n.prev_time;
        n.prev_time = s.getTime();
        n.membrane_potential += dt * (n.membrane_potential - n.resting_potential);
        n.membrane_potential += dt * 1.0;
        if (n.membrane_potential > n.threshold_potential) {
            n.membrane_potential = n.resting_potential;
        }
    }
}

const std::vector<MotorUnitGroup::LIF_Neuron>& MotorUnitGroup::getNeurons(const SimTK::State& s) const
{
    if (isCacheVariableValid(s, _neurons)) {
        return getCacheVariableValue(s, _neurons);
    }

    std::vector<MotorUnitGroup::LIF_Neuron>& neurons = updCacheVariableValue(s, _neurons);
    updateNeurons(s, neurons);
    markCacheVariableValid(s, _neurons);
    return neurons;
}

std::vector<MotorUnitGroup::LIF_Neuron>& MotorUnitGroup::updNeurons(const SimTK::State& s) const
{
    return updCacheVariableValue(s, _neurons);
}

//==============================================================================
// MODELCOMPONENT INTERFACE REQUIREMENTS
//==============================================================================
void MotorUnitGroup::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);
}

void MotorUnitGroup::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    this->_neurons = addCacheVariable("neurons", std::vector<LIF_Neuron>(get_num_neurons()), SimTK::Stage::Dynamics);
}

void MotorUnitGroup::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);

    auto neurons = updNeurons(s);
    for (auto& n : neurons) {
        n.membrane_potential = -70.0;
        n.prev_time = 0.0;
        n.refractory_period = 0.02;
        n.resting_potential = -70.0;
        n.tau = 1.0;
        n.threshold_potential = -50.0;
    }
    setCacheVariableValue(s, _neurons, neurons);
}

void MotorUnitGroup::extendSetPropertiesFromState(const SimTK::State& s)
{
    Super::extendSetPropertiesFromState(s);
}

void MotorUnitGroup::extendRealizeDynamics(const SimTK::State& s) const {
    Super::extendRealizeDynamics(s);

    //std::vector<MotorUnitGroup::LIF_Neuron>& neurons = updCacheVariableValue(s, _neurons);
    //updateNeurons(s, neurons);
    //markCacheVariableValid(s, _neurons);
}