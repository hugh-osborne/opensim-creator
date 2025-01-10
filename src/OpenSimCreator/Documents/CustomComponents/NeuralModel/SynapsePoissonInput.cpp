//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/Model/Model.h>
#include "SynapsePoissonInput.h"
#include "NeuralPopulation.h"
#include <random>

//=============================================================================
// STATICS
//=============================================================================
using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
SynapsePoissonInput::SynapsePoissonInput() :
    SynapseConnection{} {

    constructProperties();
}

SynapsePoissonInput::~SynapsePoissonInput() = default;

void SynapsePoissonInput::constructProperties() {
    setAuthors("Hugh Osborne");
    constructProperty_enabled(true);

    constructProperty_psp(0.1);
    constructProperty_lambda(100);
}

//=============================================================================
// GET AND SET
//=============================================================================
bool SynapsePoissonInput::isEnabled() const {
    return get_enabled();
}

void SynapsePoissonInput::setEnabled(bool aTrueFalse) {
    upd_enabled() = aTrueFalse;
}

double SynapsePoissonInput::getAverageNumSpikes(const SimTK::State& s) const {
    s.toString();
    return _average_psp;
}
//==============================================================================
// MODELCOMPONENT INTERFACE REQUIREMENTS
//==============================================================================

void SynapsePoissonInput::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
}

void SynapsePoissonInput::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);

    _num_output_neurons = getSocket<NeuralPopulation>("OutputPopulation").getConnectee().get_num_neurons();
    getSocket<NeuralPopulation>("OutputPopulation").getConnectee().registerIncomingConnection(this);
}

//==============================================================================
// FUNCTIONAL
//==============================================================================

std::vector<double> SynapsePoissonInput::getPsps(const SimTK::State& s) const {
    std::vector<double> out(_num_output_neurons);
    double time_elapsed = s.getTime() - _prev_time;
    _prev_time = s.getTime();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::poisson_distribution<> dis(get_lambda() * time_elapsed);

    double total_psp = 0.0;

    for (int i = 0; i < _num_output_neurons; i++) {
        double t_psp = dis(gen) * get_psp();
        total_psp = t_psp;
        out[i] += t_psp;
    }

    _average_psp = (double)(total_psp / _num_output_neurons);

    return out;
}