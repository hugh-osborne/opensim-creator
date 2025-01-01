#ifndef OPENSIM_NEURALPOPULATION_H_
#define OPENSIM_NEURALPOPULATION_H_

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/Set.h>

namespace OpenSim { 

class Model;

/**
 * NeuralPopulation
 * 
 * A base class for a population of neurons. The population can have
 * connections to other populations or to motor units (motor neuron + 
 * innervated muscle cells). 
 * 
 * To start with, we'll have a very basic population of LIF neurons 
 * simulated using a Monte Carlo technique.
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
        "enabled." );

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION AND DESTRUCTION
    //--------------------------------------------------------------------------
    NeuralPopulation();
    ~NeuralPopulation() override;


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

};  // class NeuralPopulation

} // namespace OpenSim

#endif // OPENSIM_NEURALPOPULATION_H_


