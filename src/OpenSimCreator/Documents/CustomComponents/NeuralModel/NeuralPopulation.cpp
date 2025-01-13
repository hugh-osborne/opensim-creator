//=============================================================================
// INCLUDES
//=============================================================================
#include "NeuralPopulation.h"
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace OpenSim;
using namespace std;

//=============================================================================
// STATE VARIABLE NAMES
//=============================================================================
const std::string NeuralPopulation::STATE_MEAN_MEMBRANE_POTENTIAL_NAME = "mean_membrane_potential";
const std::string NeuralPopulation::STATE_MEAN_FIRING_RATE_NAME = "mean_firing_rate";

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
NeuralPopulation::NeuralPopulation() :
    ModelComponent{} {

    _incoming_connections = std::map<const std::string, const SynapseConnection*>();

    constructProperties();
}

NeuralPopulation::~NeuralPopulation() = default;

void NeuralPopulation::constructProperties() {
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
bool NeuralPopulation::isEnabled() const {
    return get_enabled();
}

void NeuralPopulation::setEnabled(bool aTrueFalse) {
    upd_enabled() = aTrueFalse;
}

double NeuralPopulation::getMeanMembranePotential(const SimTK::State& s) const {
    auto neurons = getNeurons(s);
    double potential = 0.0;
    for (auto& n : neurons)
        potential += n.membrane_potential;
    return potential / neurons.size();
}

double NeuralPopulation::getTotalSpikes(const SimTK::State& s) const {
    double total_spikes = 0.0;
    for (int i = 0; i < get_num_neurons(); i++) {
        total_spikes += getNeuronSpiked(s, i) ? 1.0 : 0.0;
    }
    return total_spikes;
}

double NeuralPopulation::getMeanFiringRate(const SimTK::State& s) const {
    double total_spikes = getTotalSpikes(s);
    return total_spikes / (double)get_num_neurons();
}



const std::vector<NeuralPopulation::LIF_Neuron>& NeuralPopulation::getNeurons(const SimTK::State& s) const
{
    if (isCacheVariableValid(s, _neurons)) {
        return getCacheVariableValue(s, _neurons);
    }

    std::vector<NeuralPopulation::LIF_Neuron>& neurons = updCacheVariableValue(s, _neurons);
    updateNeurons(s, neurons);
    markCacheVariableValid(s, _neurons);
    return neurons;
}

std::vector<NeuralPopulation::LIF_Neuron>& NeuralPopulation::updNeurons(const SimTK::State& s) const
{
    return updCacheVariableValue(s, _neurons);
}

//==============================================================================
// MODELCOMPONENT INTERFACE REQUIREMENTS
//==============================================================================
void NeuralPopulation::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);
}

void NeuralPopulation::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    this->_neurons = addCacheVariable("neurons", std::vector<LIF_Neuron>(get_num_neurons()), SimTK::Stage::Dynamics);
}

void NeuralPopulation::extendInitStateFromProperties(SimTK::State& s) const
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
        n.spiked = false;
        n.refractory_time_left = 0.0;
    }
    markCacheVariableValid(s, _neurons);
}

void NeuralPopulation::extendSetPropertiesFromState(const SimTK::State& s)
{
    Super::extendSetPropertiesFromState(s);
}

//==============================================================================
// FUNCTIONAL
//==============================================================================

void NeuralPopulation::updateNeurons(const SimTK::State& s, std::vector<NeuralPopulation::LIF_Neuron>&) const {
    auto& neurons = updCacheVariableValue(s, _neurons);

    // get any incoming spikes
    for (auto ic : _incoming_connections) {
        std::vector<double> psps = ic.second->getPsps(s);
        s.toString();
        // psps should be the same length as the number of neurons. 
        for (int p = 0; p < get_num_neurons(); p++) {
            if (neurons[p].refractory_time_left <= 0.0)
                neurons[p].membrane_potential += psps[p];
        }
    }

    for (auto& n : neurons) {
        double dt = s.getTime() - n.prev_time;
        n.prev_time = s.getTime();
        n.spiked = false;

        if (n.refractory_time_left > 0.0)
            n.refractory_time_left -= dt;

        n.membrane_potential += dt * -(n.membrane_potential - n.resting_potential);
        if (n.membrane_potential > n.threshold_potential) {
            n.spiked = true;
            n.membrane_potential = n.resting_potential;
            n.refractory_time_left = n.refractory_period;
        }
    }
}

void NeuralPopulation::registerIncomingConnection(const SynapseConnection* conn) const{
    _incoming_connections[conn->getName()] = conn;
}

bool NeuralPopulation::getNeuronSpiked(const SimTK::State& s, int n) const {
    return getNeurons(s)[n].spiked;
}