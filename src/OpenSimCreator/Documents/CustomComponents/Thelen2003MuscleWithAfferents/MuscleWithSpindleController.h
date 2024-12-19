#include <OpenSim/Simulation/Control/PrescribedController.h>
#include <OpenSim/Common/FunctionSet.h>
//#include "osimPluginDLL.h" // for registering as plugin


namespace OpenSim {

	class Function;

	//=============================================================================
	//=============================================================================
	/**
	 * MuscleWithSpindleController is a concrete Controller that specifies functions to
	 * prescribe the control values of muscles with spindle afferents as a function
	 * of time. A spindle afferent is an object of the class Mileusnic06Spindle, and
	 * expects two inputs, corresponding to static and dynamic gamma motoneurons.
	 *
	 * The class MuscleWithSpindleController is a version of the class PrescribedController,
	 * where modifications have been made to allow three control functions for a
	 * single actuator (an object of the Thelen2003MuscleWithAfferents class) that
	 * contains a Mileusnic06Spindle object.
	 *
	 * Getting controls from files is not yet implemented.
	 *
	 * @author  Hugh Osborne based on and using work by Sergio Verduzco
	 */
	 //=============================================================================

	 //class OSIMSIMULATION_API MuscleWithSpindleController : public PrescribedController {
	 // Maybe OSIMPLUGIN_API If I want to make this a plugin
	class MuscleWithSpindleController : public PrescribedController {
		OpenSim_DECLARE_CONCRETE_OBJECT(MuscleWithSpindleController, PrescribedController);

		//=============================================================================
		// DATA
		//=============================================================================

	public:

		/** FunctionSet describing the inputs corresponding to static gamma motoneruons */
		OpenSim_DECLARE_PROPERTY(SpindleFunctionsStatic, FunctionSet,
			"Functions (one per control) describing the inputs to nuclear chain fibers and"
			"static nuclear bag fibers.");

		/** FunctionSet describing the inputs corresponding to dynamic gamma motoneruons */
		OpenSim_DECLARE_PROPERTY(SpindleFunctionsDynamic, FunctionSet,
			"Functions (one per control) describing the inputs to dynamic nuclear bag fibers.");

		OpenSim_DECLARE_OPTIONAL_PROPERTY(controls_file_spindle_static, std::string,
			"Controls storage (.sto) file containing controls for individual "
			"actuators in the model. Each column label must be either the name "
			"or path to an actuator in the model.");

		OpenSim_DECLARE_OPTIONAL_PROPERTY(controls_file_spindle_dynamic, std::string,
			"Controls storage (.sto) file containing controls for individual "
			"actuators in the model. Each column label must be either the name "
			"or path to an actuator in the model.");

		//=============================================================================
		// METHODS
		//=============================================================================
			//--------------------------------------------------------------------------
			// CONSTRUCTION AND DESTRUCTION
			//--------------------------------------------------------------------------
	public:
		/** Default constructor */
		MuscleWithSpindleController();

		/** Convenience constructor get controls from file
		 * @param controlsFileName  string containing the controls storage (.sto)
		 * @param interpMethodType	int 0-constant, 1-linear, 3-cubic, 5-quintic
		 *                          defaults to linear.
		 */
		MuscleWithSpindleController(const std::string& controlsFileName,
			int interpMethodType = 1);

		/** Destructor */
		virtual ~MuscleWithSpindleController();

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
		void computeControls(const SimTK::State& s,
			SimTK::Vector& controls) const OVERRIDE_11;


		/**
		 *	Assign prescribed control functions for the desired actuator identified
		 *  by its name. Controller takes ownership of the function.
		 *  @param actName                the actuator's name in the controller's set
		 *	@param prescribedFunction   the actuator's control function
		 *  @param prescribedFunctionStatic     function describing static gamma motoneurons
		 *  @param prescribedFunctionDynamic    function describing dynamic gamma motoneurons
		 */

		void prescribeSpindleStaticControlForActuator(const std::string& actName,
			const Function& prescribedFunctionStatic);

		void prescribeSpindleDynamicControlForActuator(const std::string& actName,
			const Function& prescribedFunctionDynamic);

	public:
		/** Model component interface */
		void extendConnectToModel(Model& model) OVERRIDE_11;
	protected:
		// construct and initialize properties
		void constructProperties();

		// This method sets all member variables to default (e.g., NULL) values.
		void setNull();

		// Index map for spindle static and dynamic
		std::unordered_map<std::string, int> _actuLabelsToSpindleStaticControlFunctionIndexMap;
		std::unordered_map<std::string, int> _actuLabelsToSpindleDynamicControlFunctionIndexMap;


		//=============================================================================
	};	// END of class MuscleWithSpindleController

}; //namespace
//=============================================================================
//=============================================================================
