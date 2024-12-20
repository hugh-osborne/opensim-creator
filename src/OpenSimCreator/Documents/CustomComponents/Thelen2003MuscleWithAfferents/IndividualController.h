#ifndef OPENSIM_INDIVIDUALCONTROLLER_H_
#define OPENSIM_INDIVIDUALCONTROLLER_H_

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/Set.h>

namespace OpenSim { 

class Model;
class Actuator;

/**
 * Code based on the Controller class in OpenSim by Ajay Seth.
 *  
 * In some cases, it's somewhat cumbersome to define a 
 * list of actuators rather than having an individual controller
 * for each - particularly as the controller becomes more complex
 * and there is interconnectedness between actuators.
 * 
 * For example, for the Thelen2003MuscleWithAfferents, there are
 * three inputs for each actuator and so four lists must be handled
 * *in the same order* to match them up. Yucky.
 * 
 * Additionally, Opensim-Creator currently doesn't support lists of
 * sockets (although I expect this will be fixed at some point). Again,
 * adding sockets and lining them all up in a list will suck in the GUI.
 * 
 * @author Hugh Osborne
 *
 */
class OSIMSIMULATION_API IndividualController : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(IndividualController, ModelComponent);

public:
//=============================================================================
// PROPERTIES
//=============================================================================
    /** Controller is enabled (active) by default.
    NOTE: Prior to OpenSim 4.0, this property was named **isDisabled**.
          If **isDisabled** is **true**, **enabled** is **false**.
          If **isDisabled** is **false**, **enabled** is **true**. */
    OpenSim_DECLARE_PROPERTY(enabled, bool, 
        "Flag (true or false) indicating whether or not the controller is "
        "enabled." );

    OpenSim_DECLARE_SOCKET(actuator, Actuator,
        "A single Actuator that this controller will control.");

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION AND DESTRUCTION
    //--------------------------------------------------------------------------
    IndividualController();
    ~IndividualController() noexcept override;

    IndividualController(const IndividualController&);
    IndividualController& operator=(IndividualController const&);

    IndividualController(IndividualController&&);
    IndividualController& operator=(IndividualController&&);

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

    /** Replace the current actuator with the one provided. */
    void setActuator(const Actuator& actuator);

    /** Compute the control for actuator
     *  This method defines the behavior for any concrete controller 
     *  and therefore must be implemented by concrete subclasses.
     *
     * @param s         system state 
     * @param controls  writable model controls (all actuators)
     */
    virtual void computeControls(const SimTK::State& s, 
        SimTK::Vector &controls) const = 0;

    /** Get the number of controls this controller computes. */
    int getNumControls() const { return _numControls; }

protected:
    /** Only a Controller can set its number of controls based on its
     * actuators. */
    void setNumControls(int numControls) { _numControls = numControls; }


private:
    // The number of controls this controller computes.
    int _numControls;

    // Construct and initialize properties.
    void constructProperties();

};  // class IndividualController

} // namespace OpenSim

#endif // OPENSIM_INDIVIDUALCONTROLLER_H_


