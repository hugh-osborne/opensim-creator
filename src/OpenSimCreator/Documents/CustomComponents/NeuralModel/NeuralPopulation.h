#ifndef OPENSIM_NEURALPOPULATION_H_
#define OPENSIM_NEURALPOPULATION_H_

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/Set.h>
#include "SynapseConnection.h"

namespace OpenSim {

    class Model;

    /**
     * Neural Population
     *
     * Simulates the behaviour of a population of LIF neurons. In a move that will
     * probably make everyone cringe, the simulation is not solved using
     * the opensim solver as I couldn't see a nice way to add multiple neurons
     * without adding so many individual neurons and adding each to the model.
     * Instead, this single component holds a cache variable of all neurons and 
     * performs a simple Euler step for each neuron based on the previous state
     * time. Totally ugly but perhaps if I ever get paid for this, I'll come back
     * and do a better job...
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
    class OSIMSIMULATION_API NeuralPopulation : public ModelComponent {
        OpenSim_DECLARE_CONCRETE_OBJECT(NeuralPopulation, ModelComponent);

    public:
        //=============================================================================
        // PROPERTIES
        //=============================================================================
            /* Population is enabled by default.*/
        OpenSim_DECLARE_PROPERTY(enabled, bool,
            "Flag (true or false) indicating whether or not the population is "
            "enabled.");

        // Number of neurons
        OpenSim_DECLARE_PROPERTY(num_neurons, int,
            "The number of neurons.");

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

        // Total Spikes
        OpenSim_DECLARE_OUTPUT(total_spikes, double, getTotalSpikes,
            SimTK::Stage::Dynamics);


        //=============================================================================
        // METHODS
        //=============================================================================
            //--------------------------------------------------------------------------
            // CONSTRUCTION AND DESTRUCTION
            //--------------------------------------------------------------------------
        NeuralPopulation();
        ~NeuralPopulation() override;

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

        double getTotalSpikes(const SimTK::State& s) const;

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


    private:

        // Construct and initialize properties.
        void constructProperties();

    protected:

        struct LIF_Neuron {                     //DIMENSION         Units      
            double tau;                         //time              s  
            double resting_potential;           //voltage           mV
            double threshold_potential;         //voltage           mV        
            double refractory_period;           //time              s
            double refractory_time_left;        //time              s
            double membrane_potential;          //voltage           mV
            double prev_time;                   //time              s
            bool   spiked;                      //boolean           

            LIF_Neuron() :
                tau(SimTK::NaN),
                resting_potential(SimTK::NaN),
                threshold_potential(SimTK::NaN),
                refractory_period(SimTK::NaN),
                refractory_time_left(SimTK::NaN),
                membrane_potential(SimTK::NaN),
                prev_time(SimTK::NaN),
                spiked(false)
            {
            }
        };

        mutable CacheVariable<std::vector<LIF_Neuron>> _neurons;

        const std::vector<LIF_Neuron>& getNeurons(const SimTK::State& s) const;
        std::vector <LIF_Neuron>& updNeurons(const SimTK::State& s) const;

        //==============================================================================
        // SIMULATION
        //==============================================================================
        virtual void updateNeurons(const SimTK::State& s) const;

        // We want to be able to create a synapse connection object in Opensim creator 
        // and indicate the input and output population for each. We don't want to list
        // in the creator the incoming connections for each population. 
        // So in the output population, we "register" each connection instead behind the 
        // scenes.
        mutable std::map<const std::string, const SynapseConnection*> _incoming_connections;

        

    public:

        void registerIncomingConnection(const SynapseConnection* conn) const;

        bool getNeuronSpiked(const SimTK::State& s, int n) const;

    };  // class NeuralPopulation

} // namespace OpenSim

#endif // OPENSIM_NEURALPOPULATION_H_


