//=============================================================================
// INCLUDES
//=============================================================================
#include "SynapseConnectionDelta.h"
#include "NeuralPopulation.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <random>

//=============================================================================
// STATICS
//=============================================================================
using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
SynapseConnectionDelta::SynapseConnectionDelta() :
    SynapseConnection{} {

    _connections = std::vector<std::vector<int>>();
    _spike_queue = std::queue<Spike>();

    constructProperties();
}

SynapseConnectionDelta::~SynapseConnectionDelta() = default;

void SynapseConnectionDelta::constructProperties() {
    setAuthors("Hugh Osborne");

    constructProperty_default_psp(0.1);
    constructProperty_delay(0.0);
    constructProperty_num_outputs_per_input(1);
}

//=============================================================================
// GET AND SET
//=============================================================================


//==============================================================================
// MODELCOMPONENT INTERFACE REQUIREMENTS
//==============================================================================

void SynapseConnectionDelta::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
}

void SynapseConnectionDelta::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);

    _num_input_neurons = getSocket<NeuralPopulation>("InputPopulation").getConnectee().get_num_neurons();
    _num_output_neurons = getSocket<NeuralPopulation>("OutputPopulation").getConnectee().get_num_neurons();

    // Build the connections
    _connections = std::vector<std::vector<int>>(_num_input_neurons);

    // random generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    for (int i = 0; i < _num_input_neurons; i++) {
        int o = (int)(dis(gen) * _num_output_neurons);
        _connections[i].push_back(o);
    }

}

//==============================================================================
// FUNCTIONAL
//==============================================================================

std::vector<double> SynapseConnectionDelta::getPsps(const SimTK::State& s) const {
    std::vector<double> out(_num_output_neurons);

    // get incoming spikes
    for (int i = 0; i < _num_input_neurons; i++) {
        // If the neuron spiked add the psp to the queue
        if (!getSocket<NeuralPopulation>("InputPopulation").getConnectee().getNeuronSpiked(s, i))
            continue;

        for (auto c : _connections[i]) {
            Spike spike;
            spike.deadline = s.getTime() + get_delay();
            spike.target_neuron = c;

            _spike_queue.push(spike);
        }
    }

    // Process the spike queue

    if (_spike_queue.empty())
        return out;

    Spike current_spike = _spike_queue.front();
    while (current_spike.deadline <= s.getTime()) {
        out[current_spike.target_neuron] += get_default_psp(); // This should get a psp that can change (not the property)
        _spike_queue.pop();

        if (_spike_queue.empty())
            break;

        current_spike = _spike_queue.front();
    }

    return out;
}