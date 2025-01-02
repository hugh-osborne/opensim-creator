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
const std::string MotorUnitGroup::STATE_MEAN_MEMBRANE_POTENTIAL_NAME = "membrane_potentials";
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



    if (!get_ignore_activation_dynamics()) {
        setActivation(s, getDefaultActivation());
    }
    if (!get_ignore_tendon_compliance()) {
        setFiberLength(s, getDefaultFiberLength());
    }
}

void MotorUnitGroup::extendSetPropertiesFromState(const SimTK::State& s)
{
    Super::extendSetPropertiesFromState(s);

    if (!get_ignore_activation_dynamics()) {
        setDefaultActivation(getStateVariableValue(s, STATE_ACTIVATION_NAME));
    }
    if (!get_ignore_tendon_compliance()) {
        setDefaultFiberLength(getStateVariableValue(s, STATE_FIBER_LENGTH_NAME));
    }
}

void MotorUnitGroup::computeStateVariableDerivatives(const SimTK::State& s) const
{
    // Activation dynamics if not ignored
    if (!get_ignore_activation_dynamics()) {
        double adot = 0;
        // if not disabled or overridden then compute its derivative
        if (appliesForce(s) && !isActuationOverridden(s)) {
            adot = getActivationDerivative(s);
        }
        setStateVariableDerivativeValue(s, STATE_ACTIVATION_NAME, adot);
    }

    // Fiber length is the next state (if it is a state at all)
    if (!get_ignore_tendon_compliance()) {
        double ldot = 0;
        // if not disabled or overridden then compute its derivative
        if (appliesForce(s) && !isActuationOverridden(s)) {
            ldot = getFiberVelocity(s);
        }
        setStateVariableDerivativeValue(s, STATE_FIBER_LENGTH_NAME, ldot);
    }
}