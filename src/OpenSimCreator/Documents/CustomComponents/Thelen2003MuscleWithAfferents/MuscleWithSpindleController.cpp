/* MuscleWithSpindleController.cpp
*/

//=============================================================================
// INCLUDES
//=============================================================================
#include "MuscleWithSpindleController.h"
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/PiecewiseConstantFunction.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Actuator.h>

//=============================================================================
// STATICS
//=============================================================================

// This command indicates that any identifier (class, variable, method, etc.)
// defined within the OpenSim namespace can be used in this file without the
// "OpenSim::" prefix.
using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
/*
 * Default constructor.
 */
MuscleWithSpindleController::MuscleWithSpindleController() :
	PrescribedController()
{
	setNull();
	constructProperties();
}

/*
 * Convenience constructor.
 */
MuscleWithSpindleController::
MuscleWithSpindleController(const std::string& controlsFileName,
	int interpMethodType) : PrescribedController()
{
	setNull();
	constructProperties();
	set_controls_file(controlsFileName);
	set_interpolation_method(interpMethodType);
}

/*
 * Destructor.
 */
MuscleWithSpindleController::~MuscleWithSpindleController()
{
}

/*
 * Set NULL values for all member variables.
 */
void MuscleWithSpindleController::setNull()
{
	setAuthors("Hugh Osborne from code by Sergio Verduzco from code by Ajay Seth");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MuscleWithSpindleController::constructProperties()
{
	constructProperty_ControlFunctions(FunctionSet());
	constructProperty_SpindleFunctionsStatic(FunctionSet());
	constructProperty_SpindleFunctionsDynamic(FunctionSet());
	constructProperty_controls_file();
	constructProperty_interpolation_method();
}


void MuscleWithSpindleController::extendConnectToModel(Model& model)
{
	Super::extendConnectToModel(model);
	
	// If a controls for spindle static file was specified, load it and create control functions
	// for any actuators that do not already have one.
	if (!getProperty_controls_file_spindle_static().empty()) {

		// Load the controls file and find the time column and column labels.
		const Storage controls(get_controls_file_spindle_static());
		const Array<string>& columnLabels = controls.getColumnLabels();
		int tcol = columnLabels.findIndex("time");
		if (tcol < 0) {
			tcol = columnLabels.findIndex("t");
			OPENSIM_THROW_IF_FRMOBJ(tcol < 0, Exception, "Prescribed controls "
				"file was not specified as a function of time.")
		}
		int nrows = controls.getSize();
		Array<double> time(0.0, nrows);
		Array<double> data(0.0, nrows);
		controls.getTimeColumn(time);

		for (int i = 0; i < columnLabels.getSize(); ++i) {
			// Skip the time column.
			if (i == tcol) continue;

			// If this column does not have an associated control function, we
			// need to create one.
			const string& columnLabel = columnLabels[i];
			if (!_actuLabelsToSpindleStaticControlFunctionIndexMap.count(columnLabel)) {

				// See if the column label matches a connected actuator. If not,
				// find and add the actuator to the controller.
				int actuIndex = getActuatorIndexFromLabel(columnLabel);
				if (actuIndex < 0) {
					bool foundActuator = false;
					for (const auto& actu : model.getComponentList<Actuator>()) {
						if (actu.getName() == columnLabel) {
							addActuator(actu);
							foundActuator = true;
							break;
						}
						if (actu.getAbsolutePathString() == columnLabel) {
							addActuator(actu);
							foundActuator = true;
							break;
						}
					}
					OPENSIM_THROW_IF_FRMOBJ(!foundActuator, Exception,
						"Control provided from file with label {}, but no "
						"matching Actuator was found in the model.",
						columnLabel)

						// If we found a matching actuator, call
						// finalizeConnection() to sync the connectee path names
						// with the Actuator connectee.
						socket.finalizeConnection(model);
				}

				// Create the control function and assign it to the actuator.
				controls.getDataColumn(
					controls.getStateIndex(columnLabel), data);
				std::unique_ptr<Function> controlFunction =
					createFunctionFromData(columnLabel, time, data);
				prescribeSpindleStaticControlForActuator(columnLabel, *controlFunction);
			}
		}
	}

	// Populate the actuator index to control function index map.
	std::unordered_map<int, int> actuToSpindleStaticControlFunctionIndexMap;
	for (const auto& pair : _actuLabelsToSpindleStaticControlFunctionIndexMap) {
		int actuIndex = getActuatorIndexFromLabel(pair.first);
		OPENSIM_THROW_IF_FRMOBJ(actuIndex < 0, Exception,
			"Actuator {} was not found in the model.", pair.first)

			OPENSIM_THROW_IF_FRMOBJ(
				actuToSpindleStaticControlFunctionIndexMap.count(actuIndex), Exception,
				"Expected actuator {} to have one control function "
				"assigned, but multiple control functions were detected. "
				"This may have occurred because a control function was "
				"specified by actuator name and by actuator path.",
				socket.getConnectee(actuIndex).getAbsolutePathString())

			actuToSpindleStaticControlFunctionIndexMap[actuIndex] = pair.second;
	}

	// Reorder the control functions to match the order of the actuators. We 
	// must do this so that the actuator connectee order matches the control
	// function order during serialization.
	FunctionSet controlFuncsCopy = get_SpindleFunctionsStatic();
	for (int i = 0; i < (int)socket.getNumConnectees(); ++i) {
		const int controlFuncIndex = actuToSpindleStaticControlFunctionIndexMap.at(i);
		upd_SpindleFunctionsStatic().set(i, controlFuncsCopy.get(controlFuncIndex));
	}


	// If a controls for spindle dynamic file was specified, load it and create control functions
	// for any actuators that do not already have one.
	if (!getProperty_controls_file_spindle_dynamic().empty()) {

		// Load the controls file and find the time column and column labels.
		const Storage controls(get_controls_file_spindle_dynamic());
		const Array<string>& columnLabels = controls.getColumnLabels();
		int tcol = columnLabels.findIndex("time");
		if (tcol < 0) {
			tcol = columnLabels.findIndex("t");
			OPENSIM_THROW_IF_FRMOBJ(tcol < 0, Exception, "Prescribed controls "
				"file was not specified as a function of time.")
		}
		int nrows = controls.getSize();
		Array<double> time(0.0, nrows);
		Array<double> data(0.0, nrows);
		controls.getTimeColumn(time);

		for (int i = 0; i < columnLabels.getSize(); ++i) {
			// Skip the time column.
			if (i == tcol) continue;

			// If this column does not have an associated control function, we
			// need to create one.
			const string& columnLabel = columnLabels[i];
			if (!_actuLabelsToSpindleDynamicControlFunctionIndexMap.count(columnLabel)) {

				// See if the column label matches a connected actuator. If not,
				// find and add the actuator to the controller.
				int actuIndex = getActuatorIndexFromLabel(columnLabel);
				if (actuIndex < 0) {
					bool foundActuator = false;
					for (const auto& actu : model.getComponentList<Actuator>()) {
						if (actu.getName() == columnLabel) {
							addActuator(actu);
							foundActuator = true;
							break;
						}
						if (actu.getAbsolutePathString() == columnLabel) {
							addActuator(actu);
							foundActuator = true;
							break;
						}
					}
					OPENSIM_THROW_IF_FRMOBJ(!foundActuator, Exception,
						"Control provided from file with label {}, but no "
						"matching Actuator was found in the model.",
						columnLabel)

						// If we found a matching actuator, call
						// finalizeConnection() to sync the connectee path names
						// with the Actuator connectee.
						socket.finalizeConnection(model);
				}

				// Create the control function and assign it to the actuator.
				controls.getDataColumn(
					controls.getStateIndex(columnLabel), data);
				std::unique_ptr<Function> controlFunction =
					createFunctionFromData(columnLabel, time, data);
				prescribeSpindleDynamicControlForActuator(columnLabel, *controlFunction);
			}
		}
	}

	// Populate the actuator index to control function index map.
	std::unordered_map<int, int> actuToSpindleDynamicControlFunctionIndexMap;
	for (const auto& pair : _actuLabelsToSpindleDynamicControlFunctionIndexMap) {
		int actuIndex = getActuatorIndexFromLabel(pair.first);
		OPENSIM_THROW_IF_FRMOBJ(actuIndex < 0, Exception,
			"Actuator {} was not found in the model.", pair.first)

			OPENSIM_THROW_IF_FRMOBJ(
				actuToSpindleDynamicControlFunctionIndexMap.count(actuIndex), Exception,
				"Expected actuator {} to have one control function "
				"assigned, but multiple control functions were detected. "
				"This may have occurred because a control function was "
				"specified by actuator name and by actuator path.",
				socket.getConnectee(actuIndex).getAbsolutePathString())

			actuToSpindleDynamicControlFunctionIndexMap[actuIndex] = pair.second;
	}

	// Reorder the control functions to match the order of the actuators. We 
	// must do this so that the actuator connectee order matches the control
	// function order during serialization.
	FunctionSet controlFuncsCopy = get_SpindleFunctionsDynamic();
	for (int i = 0; i < (int)socket.getNumConnectees(); ++i) {
		const int controlFuncIndex = actuToSpindleDynamicControlFunctionIndexMap.at(i);
		upd_SpindleFunctionsDynamic().set(i, controlFuncsCopy.get(controlFuncIndex));
	}
}


// compute the control value for an actuator
void MuscleWithSpindleController::computeControls(const SimTK::State& s, SimTK::Vector& controls) const
{
	SimTK::Vector actControls(3, 0.0);
	SimTK::Vector time(1, s.getTime());

	const auto& socket = getSocket<Actuator>("actuators");
	for (int i = 0; i < (int)socket.getNumConnectees(); ++i) {
		actControls[0] = get_ControlFunctions()[i].calcValue(time);
		actControls[1] = get_SpindleFunctionsStatic()[i].calcValue(time);
		actControls[2] = get_SpindleFunctionsDynamic()[i].calcValue(time);
		socket.getConnectee(i).addInControls(actControls, controls);
	}

}


//=============================================================================
// GET AND SET
//=============================================================================

void MuscleWithSpindleController::prescribeSpindleStaticControlForActuator(const std::string& actName,
	Function& prescribedFunctionStatic)
{
	FunctionSet& controlFuncs = upd_SpindleFunctionsStatic();
	if (_actuLabelsToSpindleStaticControlFunctionIndexMap.count(actName)) {
		const int index = _actuLabelsToControlFunctionIndexMap.at(actName);
		controlFuncs.set(index, prescribedFunctionStatic);
	}
	else {
		const int size = controlFuncs.getSize();
		controlFuncs.setSize(size + 1);
		controlFuncs.set(size, prescribedFunctionStatic);
		_actuLabelsToSpindleStaticControlFunctionIndexMap[actName] = size;
	}
}

void MuscleWithSpindleController::prescribeSpindleDynamicControlForActuator(const std::string& actName,
	Function& prescribedFunctionDynamic)
{
	FunctionSet& controlFuncs = upd_SpindleFunctionsDynamic();
	if (_actuLabelsToSpindleDynamicControlFunctionIndexMap.count(actName)) {
		const int index = _actuLabelsToControlFunctionIndexMap.at(actName);
		controlFuncs.set(index, prescribedFunctionDynamic);
	}
	else {
		const int size = controlFuncs.getSize();
		controlFuncs.setSize(size + 1);
		controlFuncs.set(size, prescribedFunctionDynamic);
		_actuLabelsToSpindleDynamicControlFunctionIndexMap[actName] = size;
	}
}
