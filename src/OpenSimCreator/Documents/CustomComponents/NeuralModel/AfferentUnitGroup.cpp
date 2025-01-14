//=============================================================================
// INCLUDES
//=============================================================================
#include "AfferentUnitGroup.h"
#include "../Thelen2003MuscleWithAfferents/Thelen2003MuscleWithAfferents.h"
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
AfferentUnitGroup::AfferentUnitGroup() :
        NeuralPopulation{}{
    constructProperties();
}

AfferentUnitGroup::~AfferentUnitGroup() = default;

void AfferentUnitGroup::constructProperties() {
    setAuthors("Hugh Osborne");

    constructProperty_excitation_mean(1.0);
    constructProperty_excitation_var(1.0);
    constructProperty_afferent_type(0);
}

void AfferentUnitGroup::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);

    _neuron_afferent_excitabilities = std::vector<double>(get_num_neurons());
    for (int i = 0; i < get_num_neurons(); i++) {
        _neuron_afferent_excitabilities[i] = get_excitation_mean();
    }
}

void AfferentUnitGroup::updateNeurons(const SimTK::State& s) const {
    auto& neurons = updCacheVariableValue(s, _neurons);

    double activity = getSocket<Thelen2003MuscleWithAfferents>("SourceMuscle").getConnectee().getIaOutput(s);

    if (get_afferent_type() == 1) {
        activity = getSocket<Thelen2003MuscleWithAfferents>("SourceMuscle").getConnectee().getGTOout(s);
    }
    else if (get_afferent_type() == 2) {
        activity = getSocket<Thelen2003MuscleWithAfferents>("SourceMuscle").getConnectee().getIIOutput(s);
    }
    else {
        activity = 0.0;
        // Error!
    }

    // Apply the activity with the multiplier

    for (int p = 0; p < get_num_neurons(); p++) {
        double dt = s.getTime() - neurons[p].prev_time;
        if (neurons[p].refractory_time_left <= 0.0)
            neurons[p].membrane_potential += dt * activity * _neuron_afferent_excitabilities[p];
    }

    Super::updateNeurons(s);
}