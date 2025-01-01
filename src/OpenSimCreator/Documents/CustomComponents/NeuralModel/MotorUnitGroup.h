#ifndef OPENSIM_MOTORUNITGROUOP_H_
#define OPENSIM_MOTORUNITGROUOP_H_

#include <OpenSim/Simulation/Model/ModelComponent.h>
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
class OSIMSIMULATION_API MotorUnitGroup : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(MotorUnitGroup, ModelComponent);

public:
//=============================================================================
// PROPERTIES
//=============================================================================
    /* Population is enabled by default.*/
    OpenSim_DECLARE_PROPERTY(enabled, bool, 
        "Flag (true or false) indicating whether or not the population is "
        "enabled." );

    // The innervated muscle
    OpenSim_DECLARE_SOCKET(ActivatedMuscle, Muscle, 
        "The muscle that will have its excitation set.");

    // Number of motoneurons
    OpenSim_DECLARE_PROPERTY(num_neurons, int,
        "The number of motoneurons.");

    // LIF tau mean
    OpenSim_DECLARE_PROPERTY(LIF_tau_mean, double,
        "Mean value for tau (time scale) for the LIF neurons.");

    // LIF tau var
    OpenSim_DECLARE_PROPERTY(LIF_tau_var, double,
        "Variance for tau (time scale) for the LIF neurons.");

    // LIF Resting Potential mean
    OpenSim_DECLARE_PROPERTY(LIF_rest_mean, double,
        "Mean value for resting potential of the LIF neurons.");

    // LIF Resting Potential var
    OpenSim_DECLARE_PROPERTY(LIF_rest_var, double,
        "Variance for resting potential of the LIF neurons.");

    // LIF Threshold mean
    OpenSim_DECLARE_PROPERTY(LIF_threshold_mean, double,
        "Mean value for spike threshold of the LIF neurons.");

    // LIF Threshold var
    OpenSim_DECLARE_PROPERTY(LIF_threshold_var, double,
        "Variance for spike threshold of the LIF neurons.");

    // Membrane Potentials
    OpenSim_DECLARE_LIST_OUTPUT(membrane_potentials, double, 
        SimTK::Stage::Dynamics);

    // Average Firing Rate
    OpenSim_DECLARE_OUTPUT(mean_firing_rate, double,
        SimTK::Stage::Dynamics);


//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION AND DESTRUCTION
    //--------------------------------------------------------------------------
    MotorUnitGroup();
    ~MotorUnitGroup() override;

    //--------------------------------------------------------------------------
    // CONTROLLER INTERFACE
    //--------------------------------------------------------------------------
    /** Get whether or not this controller is enabled.
     * @return true when controller is enabled.
     */
    bool isEnabled() const;

    /** Enable this controller.
     * @param enableFlag Enable the controller if true.
     */
    void setEnabled(bool enableFlag);


private:

    // Construct and initialize properties.
    void constructProperties();

};  // class MotorUnitGroup

} // namespace OpenSim

#endif // OPENSIM_MOTORUNITGROUOP_H_


