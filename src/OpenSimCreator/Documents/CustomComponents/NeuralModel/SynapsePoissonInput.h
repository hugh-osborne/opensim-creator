#ifndef OPENSIM_SYNAPSEPOISSONINPUT_H_
#define OPENSIM_SYNAPSEPOISSONINPUT_H_

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include "SynapseConnection.h"
#include <OpenSim/Common/Set.h>

namespace OpenSim {

    class Model;
    class NeuralPopulation;

    /**
     * Synapse Poisson Input
     *
     * Provides a Poisson distributed spike train to the output population.
     *
     * @author Hugh Osborne
     *
     */
    class OSIMSIMULATION_API SynapsePoissonInput : public SynapseConnection {
        OpenSim_DECLARE_CONCRETE_OBJECT(SynapsePoissonInput, SynapseConnection);

    public:
        //=============================================================================
        // PROPERTIES
        //=============================================================================
            /* Population is enabled by default.*/
        OpenSim_DECLARE_PROPERTY(enabled, bool,
            "Flag (true or false) indicating whether or not the population is "
            "enabled.");

        // Post Synaptic Potential (Efficacy)
        OpenSim_DECLARE_PROPERTY(psp, double,
            "The post synaptic potential (psp): The change in membrane potential provided by this delta synapse.");

        // Poisson firing rate, lambda
        OpenSim_DECLARE_PROPERTY(lambda, double,
            "The average spike firing rate of this input which is the lambda (rate) value of the Poisson distribution.");


        //=============================================================================
        // METHODS
        //=============================================================================
            //--------------------------------------------------------------------------
            // CONSTRUCTION AND DESTRUCTION
            //--------------------------------------------------------------------------
        SynapsePoissonInput();
        ~SynapsePoissonInput() override;

        /** Get whether or not this controller is enabled.
         * @return true when controller is enabled.
         */
        bool isEnabled() const;

        /** Enable this controller.
         * @param enableFlag Enable the controller if true.
         */
        void setEnabled(bool enableFlag);

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

    private:
        mutable int _num_output_neurons;
        mutable double _prev_time;

    };  // class SynapsePoissonInput

} // namespace OpenSim

#endif // OPENSIM_SYNAPSEPOISSONINPUT_H_


