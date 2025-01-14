#ifndef OPENSIM_AFFERENTUNITGROUP_H_
#define OPENSIM_AFFERENTUNITGROUP_H_

#include "NeuralPopulation.h"
#include <OpenSim/Common/Set.h>

namespace OpenSim { 

class Model;
class Thelen2003MuscleWithAfferents;

/**
 * AfferentUnitGroup
 * 
 * As with Motor units (which convert motor neuron spikes to 
 * an excitation value to be used by the muscle model), the afferent
 * signals from the muscle must be converted to a spike train.
 * 
 * @author Hugh Osborne
 *
 */
class OSIMSIMULATION_API AfferentUnitGroup : public NeuralPopulation {
    OpenSim_DECLARE_CONCRETE_OBJECT(AfferentUnitGroup, NeuralPopulation);

public:
//=============================================================================
// PROPERTIES
//=============================================================================

    // Average Excitation value for the neural population
    OpenSim_DECLARE_PROPERTY(excitation_mean, double,
        "Mean excitation of the neurons.");

    // Variance of the Excitation value for the neural population
    OpenSim_DECLARE_PROPERTY(excitation_var, double,
        "Variance of the excitation of the neurons.");

    // Type of afferent signal to receive from the muscle
    // TODO: Make this an enum type 
    OpenSim_DECLARE_PROPERTY(afferent_type, int,
        "The type of afferent: 0 for group Ia, 1 for group Ib, 2 for group II.");

    OpenSim_DECLARE_SOCKET(SourceMuscle, Thelen2003MuscleWithAfferents,
        "The Muscle providing the afferent signal.");


//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION AND DESTRUCTION
    //--------------------------------------------------------------------------
    AfferentUnitGroup();
    ~AfferentUnitGroup() override;

    // Override UpdateNeurons to pass the afferent signal as an increase in potential.
    void updateNeurons(const SimTK::State& s) const override;

    /** Initializes the state of the ModelComponent */
    void extendInitStateFromProperties(SimTK::State& s) const override;

private:

    // Construct and initialize properties.
    void constructProperties();

    mutable std::vector<double> _neuron_afferent_excitabilities;

};  // class AfferentUnitGroup

} // namespace OpenSim

#endif // OPENSIM_AFFERENTUNITGROUP_H_


