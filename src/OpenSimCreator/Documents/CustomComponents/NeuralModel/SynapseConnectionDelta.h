#ifndef OPENSIM_SYNAPSECONNECTIONDELTA_H_
#define OPENSIM_SYNAPSECONNECTIONDELTA_H_

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/Set.h>
#include <queue>
#include "SynapseConnection.h"

namespace OpenSim {

    class Model;
    class NeuralPopulation;

    /**
     * Synapse Connection
     *
     * Connection between populations...
     *
     * @author Hugh Osborne
     *
     */
    class OSIMSIMULATION_API SynapseConnectionDelta : public SynapseConnection {
        OpenSim_DECLARE_CONCRETE_OBJECT(SynapseConnectionDelta, SynapseConnection);

    public:
        //=============================================================================
        // PROPERTIES
        //=============================================================================
        
        // Default Post Synaptic Potential (Efficacy)
        OpenSim_DECLARE_PROPERTY(default_psp, double,
            "The default post synaptic potential (psp): The change in membrane potential provided by this delta synapse.");

        // Delay
        OpenSim_DECLARE_PROPERTY(delay, double,
            "The time delay of incoming spikes to reach the post synaptic neuron.");

        // Connection Ratio: Number of output neurons per input neuron.
        // TODO: make this a double and update the connection setup accordingly
        OpenSim_DECLARE_PROPERTY(num_outputs_per_input, int,
            "The number of neurons in the output population with connections from each input neuron. Randomly assigned.");

        /** input population */
        OpenSim_DECLARE_SOCKET(InputPopulation, NeuralPopulation,
            "The Input Population.");


        //=============================================================================
        // METHODS
        //=============================================================================
            //--------------------------------------------------------------------------
            // CONSTRUCTION AND DESTRUCTION
            //--------------------------------------------------------------------------
        SynapseConnectionDelta();
        ~SynapseConnectionDelta() override;

        /* Get the total PSP for each output neuron.
        */
        std::vector<double> getPsps(const SimTK::State& s) const override;

        //==============================================================================
        // MODELCOMPONENT INTERFACE REQUIREMENTS
        //==============================================================================

        /** Creates the ModelComponent so that it can be used in simulation */
        void extendAddToSystem(SimTK::MultibodySystem& system) const override;

        /** Initializes the state of the ModelComponent */
        void extendInitStateFromProperties(SimTK::State& s) const override;



    private:

        // Construct and initialize properties.
        void constructProperties();

    protected:

        struct Spike {                          //DIMENSION         Units      
            double deadline;                    //time              s  
            int    target_neuron;               //neuron id         
            // Later add efficacy and spike time (for STDP) if required

            Spike() :
                deadline(SimTK::NaN),
                target_neuron(-1)
            {
            }
        };

    private:
        mutable int _num_input_neurons;
        mutable int _num_output_neurons;

        mutable std::queue<Spike> _spike_queue;
        mutable std::vector<std::vector<int>> _connections;


    };  // class SynapseConnectionDelta

} // namespace OpenSim

#endif // OPENSIM_SYNAPSECONNECTIONDELTA_H_


