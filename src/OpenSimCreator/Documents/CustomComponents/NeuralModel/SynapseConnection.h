#ifndef OPENSIM_SYNAPSECONNECTION_H_
#define OPENSIM_SYNAPSECONNECTION_H_

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/Set.h>

namespace OpenSim {

    class Model;

    /**
     * Synapse Connection
     *
     * Connection between populations...
     *
     * @author Hugh Osborne
     *
     */
    class OSIMSIMULATION_API SynapseConnection : public ModelComponent {
        OpenSim_DECLARE_CONCRETE_OBJECT(SynapseConnection, ModelComponent);

    public:
        //=============================================================================
        // PROPERTIES
        //=============================================================================
            /* Population is enabled by default.*/
        OpenSim_DECLARE_PROPERTY(enabled, bool,
            "Flag (true or false) indicating whether or not the population is "
            "enabled.");

        // Default Post Synaptic Potential (Efficacy)
        OpenSim_DECLARE_PROPERTY(default_psp, double,
            "The default post synaptic potential (psp): The change in membrane potential provided by this delta synapse.");

        // Delay
        OpenSim_DECLARE_PROPERTY(delay, double,
            "The time delay of incoming spikes to reach the post synaptic neuron.");

        /** input population */
        OpenSim_DECLARE_SOCKET(InputPopulation, NeuralPopulation,
            "The Input Population.");

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

        /** Get whether or not this controller is enabled.
         * @return true when controller is enabled.
         */
        bool isEnabled() const;

        /** Enable this controller.
         * @param enableFlag Enable the controller if true.
         */
        void setEnabled(bool enableFlag);

        double getFiringRate(SimTK::State& s);
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
        mutable CacheVariable<double> _psp;
        mutable CacheVariable<std::vector<double>> _queue;

        const std::vector<double>& getPsp(const SimTK::State& s) const;
        std::vector <double>& updPsp(const SimTK::State& s) const;

        const std::vector<double>& getQueue(const SimTK::State& s) const;
        std::vector <double>& updQueue(const SimTK::State& s) const;

    };  // class SynapseConnection

} // namespace OpenSim

#endif // OPENSIM_SYNAPSECONNECTION_H_


