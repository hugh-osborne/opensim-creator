#ifndef OPENSIM_MOTORUNITGROUP_H_
#define OPENSIM_MOTORUNITGROUP_H_

#include "NeuralPopulation.h"
#include <OpenSim/Common/Set.h>

namespace OpenSim { 

class Model;

/**
 * MotorUnitGroup
 * 
 * Simulates the behaviour of a group of motor neurons and the activation
 * of connected muscle cells. The expectation is that the motor neurons
 * in this group are innervated by various neural populations and afferents. 
 * In this class, the resulting firing of the motor neurons are translated 
 * into a single excitation value that is then passed to the muscle model.
 * 
 * As a basic example, we will simulate motor neurons as LIF neurons but more
 * a more complex model would be preferable to capture other behaviours beyond
 * tonic spiking.
 * 
 * @author Hugh Osborne
 *
 */
class OSIMSIMULATION_API MotorUnitGroup : public NeuralPopulation {
    OpenSim_DECLARE_CONCRETE_OBJECT(MotorUnitGroup, NeuralPopulation);

public:
//=============================================================================
// PROPERTIES
//=============================================================================

    // Spike Proportion Multiplier
    OpenSim_DECLARE_PROPERTY(multiplier, double,
        "Multiplier for the excitation.");

    // Muscle Excitation Value
    OpenSim_DECLARE_OUTPUT(muscle_excitation, double, getMuscleExcitation, 
        SimTK::Stage::Dynamics);


//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION AND DESTRUCTION
    //--------------------------------------------------------------------------
    MotorUnitGroup();
    ~MotorUnitGroup() override;

    //==============================================================================
    // GETTERS/SETTERS
    //==============================================================================

    double getMuscleExcitation(const SimTK::State& s) const;

protected:
    static const std::string STATE_MUSCLE_EXCITATION_NAME;


private:

    // Construct and initialize properties.
    void constructProperties();


};  // class MotorUnitGroup

} // namespace OpenSim

#endif // OPENSIM_MOTORUNITGROUP_H_


