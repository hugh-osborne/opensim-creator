#ifndef OPENSIM_MUSCLEWITHSPINDLECONTROLLER_H_
#define OPENSIM_MUSCLEWITHSPINDLECONTROLLER_H_

#include "IndividualController.h"
#include "Thelen2003MuscleWithAfferents.h"
//#include "osimPluginDLL.h" // for registering as plugin


namespace OpenSim {

	//=============================================================================
	//=============================================================================
	/**
	 * MuscleWithSpindleController is a concrete Controller that takes three inputs
	 * to control the Excitation (alpha MN) and static and dynamic Gamma MN 
	 * excitation of a Thelen2003MuscleWithAfferents. A spindle afferent is an 
	 * object of the class Mileusnic06Spindle, and expects two inputs, 
	 * corresponding to static and dynamic gamma motoneurons.
	 *
	 * The class MuscleWithSpindleController is a version of the class Controller,
	 * where modifications have been made to allow three control functions for a
	 * single actuator (an object of the Thelen2003MuscleWithAfferents class) that
	 * contains a Mileusnic06Spindle object.
	 *
	 * @author  Hugh Osborne based on and using work by Sergio Verduzco
	 */
	 //=============================================================================

	 //class OSIMSIMULATION_API MuscleWithSpindleController : public PrescribedController {
	 // Maybe OSIMPLUGIN_API If I want to make this a plugin
	class MuscleWithSpindleController : public IndividualController {
		OpenSim_DECLARE_CONCRETE_OBJECT(MuscleWithSpindleController, IndividualController);

		//=============================================================================
		// DATA
		//=============================================================================

	public:
		/** Inputs corresponding to alpha motoneruons */
		OpenSim_DECLARE_SOCKET(AlphaInput, Thelen2003MuscleWithAfferents,
			"Inputs to the alpha motoneurons.");

		/** Inputs corresponding to static gamma motoneruons */
		OpenSim_DECLARE_SOCKET(SpindleInputStatic, Thelen2003MuscleWithAfferents, 
			"Inputs to nuclear chain fibers and static nuclear bag fibers.");

		/** Inputs corresponding to dynamic gamma motoneruons */
		OpenSim_DECLARE_SOCKET(SpindleInputDynamic, Thelen2003MuscleWithAfferents, 
			"Inputs to dynamic nuclear bag fibers.");

		//=============================================================================
		// METHODS
		//=============================================================================
			//--------------------------------------------------------------------------
			// CONSTRUCTION AND DESTRUCTION
			//--------------------------------------------------------------------------
	public:
		/** Default constructor */
		MuscleWithSpindleController();
		~MuscleWithSpindleController() override;
		//--------------------------------------------------------------------------
		// CONTROL
		//--------------------------------------------------------------------------
		/**
		 * Compute the control values for all actuators under the control of this
		 * Controller.
		 *
		 * @param s             system state
		 * @param controls      model controls
		 */
		void computeControls(const SimTK::State& s, SimTK::Vector& controls) const OVERRIDE_11;
	
		/** Model component interface */
		void extendConnectToModel(Model& model) OVERRIDE_11;

	protected:

		// This method sets all member variables to default (e.g., NULL) values.
		void setNull();

		//=============================================================================
	};	// END of class MuscleWithSpindleController

}; //namespace
//=============================================================================
//=============================================================================

#endif