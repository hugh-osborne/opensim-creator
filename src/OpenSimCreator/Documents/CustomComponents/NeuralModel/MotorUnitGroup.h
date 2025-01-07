#ifndef OPENSIM_MOTORUNITGROUP_H_
#define OPENSIM_MOTORUNITGROUP_H_

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
 * The LIF model tracks membrane potential that slowly reduces over time towards
 * the resting potential. Input spikes cause instantaneous jumps in membrane
 * potential (delta synapses). When the membrane potential goes above the
 * threshold potential, the membrane potential is reset to the resting potential.
 * Additional incoming spikes are ignored during the refractory period.
 * 
 * The refractory period is useful to impose a maximum firing rate.
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

    // Number of motoneurons
    OpenSim_DECLARE_PROPERTY(num_neurons, int,
        "The number of motoneurons.");

    // LIF tau mean
    OpenSim_DECLARE_PROPERTY(tau_mean, double,
        "Mean value for tau (time scale) for the LIF neurons.");

    // LIF tau var
    OpenSim_DECLARE_PROPERTY(tau_var, double,
        "Variance for tau (time scale) for the LIF neurons.");

    // LIF Resting Potential mean
    OpenSim_DECLARE_PROPERTY(rest_mean, double,
        "Mean value for resting potential of the LIF neurons.");

    // LIF Resting Potential var
    OpenSim_DECLARE_PROPERTY(rest_var, double,
        "Variance for resting potential of the LIF neurons.");

    // LIF Threshold mean
    OpenSim_DECLARE_PROPERTY(threshold_mean, double,
        "Mean value for spike threshold of the LIF neurons.");

    // LIF Threshold var
    OpenSim_DECLARE_PROPERTY(threshold_var, double,
        "Variance for spike threshold of the LIF neurons.");

    // LIF refractory time mean
    OpenSim_DECLARE_PROPERTY(refractory_mean, double,
        "Mean value for spike threshold of the LIF neurons.");

    // LIF refractory time var
    OpenSim_DECLARE_PROPERTY(refractory_var, double,
        "Variance for spike threshold of the LIF neurons.");

    // Average Membrane Potential
    OpenSim_DECLARE_OUTPUT(mean_membrane_potential, double, getMeanMembranePotential,
        SimTK::Stage::Dynamics);

    // Average Firing Rate
    OpenSim_DECLARE_OUTPUT(mean_firing_rate, double, getMeanFiringRate,
        SimTK::Stage::Dynamics);

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

    /** Get whether or not this controller is enabled.
     * @return true when controller is enabled.
     */
    bool isEnabled() const;

    /** Enable this controller.
     * @param enableFlag Enable the controller if true.
     */
    void setEnabled(bool enableFlag);

    //==============================================================================
    // GETTERS/SETTERS
    //==============================================================================
    double getMeanMembranePotential(const SimTK::State& s) const;

    double getMeanFiringRate(const SimTK::State& s) const;

    double getMuscleExcitation(const SimTK::State& s) const;

    //==============================================================================
    // MODELCOMPONENT INTERFACE REQUIREMENTS
    //==============================================================================
    /** Sets up the ModelComponent from the model, if necessary */
    void extendConnectToModel(Model& model) override;

    /** Creates the ModelComponent so that it can be used in simulation */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    /** Initializes the state of the ModelComponent */
    void extendInitStateFromProperties(SimTK::State& s) const override;

    /** Sets the default state for the ModelComponent */
    void extendSetPropertiesFromState(const SimTK::State& s) override;

protected:
    static const std::string STATE_MEAN_MEMBRANE_POTENTIAL_NAME;
    static const std::string STATE_MEAN_FIRING_RATE_NAME;
    static const std::string STATE_MUSCLE_EXCITATION_NAME;


private:

    // Construct and initialize properties.
    void constructProperties();

protected:

    struct LIF_Neuron {                     //DIMENSION         Units      
        double tau;                         //time              s  
        double resting_potential;           //voltage           mV
        double threshold_potential;         //voltage           mV        
        double refractory_period;           //time              s
        double membrane_potential;          //voltage           mV
        double prev_time;                   //time              s

        LIF_Neuron() :
            tau(SimTK::NaN),
            resting_potential(SimTK::NaN),
            threshold_potential(SimTK::NaN),
            refractory_period(SimTK::NaN),
            membrane_potential(SimTK::NaN),
            prev_time(SimTK::NaN)
            {
        }
    };

    mutable CacheVariable<std::vector<LIF_Neuron>> _neurons;

    const std::vector<LIF_Neuron>& getNeurons(const SimTK::State& s) const;
    std::vector < LIF_Neuron>& updNeurons(const SimTK::State& s) const;

    //==============================================================================
    // SIMULATION
    //==============================================================================
    void updateNeurons(const SimTK::State& s, std::vector<MotorUnitGroup::LIF_Neuron>&) const;

};  // class MotorUnitGroup

} // namespace OpenSim

#endif // OPENSIM_MOTORUNITGROUP_H_


