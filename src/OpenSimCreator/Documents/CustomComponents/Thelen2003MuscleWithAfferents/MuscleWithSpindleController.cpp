/* MuscleWithSpindleController.cpp
*/

//=============================================================================
// INCLUDES
//=============================================================================
#include "MuscleWithSpindleController.h"
#include <OpenSim/Common/CommonUtilities.h>
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
    IndividualController()
{
	setNull();
}

MuscleWithSpindleController::~MuscleWithSpindleController() = default;

/*
 * Set NULL values for all member variables.
 */
void MuscleWithSpindleController::setNull()
{
	setAuthors("Hugh Osborne");
}

void MuscleWithSpindleController::extendConnectToModel(Model& model)
{
	Super::extendConnectToModel(model);

	// Check the sockets exist? Perhaps this is handled already...
}

// compute the control value for an actuator
void MuscleWithSpindleController::computeControls(const SimTK::State& s, SimTK::Vector& controls) const
{
	SimTK::Vector actControls(3, 0.0);
	s.toString();
	const auto& socket = getSocket<Actuator>("actuator");
	actControls[0] = getSocket<MotorUnitGroup>("AlphaInput").getConnectee().getMuscleExcitation(s);
	actControls[1] = 100.0; //getSocket<Thelen2003MuscleWithAfferents>("SpindleInputStatic").getConnectee().getIaOutput(s);
	actControls[2] = 100.0; // getSocket<Thelen2003MuscleWithAfferents>("SpindleInputDynamic").getConnectee().getIaOutput(s);
	socket.getConnectee().addInControls(actControls, controls);
}