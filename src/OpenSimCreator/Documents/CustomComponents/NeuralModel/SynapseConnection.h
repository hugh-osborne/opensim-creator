#ifndef OPENSIM_SYNAPSECONNECTION_H_
#define OPENSIM_SYNAPSECONNECTION_H_

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/Set.h>

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
    class OSIMSIMULATION_API SynapseConnection : public ModelComponent {
        OpenSim_DECLARE_ABSTRACT_OBJECT(SynapseConnection, ModelComponent);

    public:
        //=============================================================================
        // PROPERTIES
        //=============================================================================

        /** output population */
        OpenSim_DECLARE_SOCKET(OutputPopulation, NeuralPopulation,
            "The Output Population.");


        //=============================================================================
        // METHODS
        //=============================================================================
            //--------------------------------------------------------------------------
            // CONSTRUCTION AND DESTRUCTION
            //--------------------------------------------------------------------------
        SynapseConnection();
        ~SynapseConnection() override;

        /** Initializes the state of the ModelComponent */
        void extendInitStateFromProperties(SimTK::State& s) const override;

        /** Get whether or not this controller is enabled.
         * @return true when controller is enabled.

        /* Get the total PSP for each output neuron.
        */
        virtual std::vector<double> getPsps(const SimTK::State& s) const = 0;

    };  // class SynapseConnection

} // namespace OpenSim

#endif // OPENSIM_SYNAPSECONNECTION_H_


