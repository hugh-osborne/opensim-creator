//=============================================================================
// INCLUDES
//=============================================================================
#include "IndividualController.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Actuator.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
IndividualController::IndividualController() :
        Controller{}, _numControls{0} {
    constructProperties();
}

IndividualController::~IndividualController() noexcept = default;

IndividualController::IndividualController(const IndividualController&) = default;

IndividualController::IndividualController(IndividualController&&) = default;

IndividualController& IndividualController::operator=(const IndividualController&) = default;

IndividualController& IndividualController::operator=(IndividualController&&) = default;

void IndividualController::constructProperties() {
    setAuthors("Hugh Osborne based on work by Ajay Seth, Frank Anderson, Chand John, Samuel Hamner");
}

//=============================================================================
// GET AND SET
//=============================================================================
bool IndividualController::isEnabled() const {
    return get_enabled();
}

void IndividualController::setEnabled(bool aTrueFalse) {
    upd_enabled() = aTrueFalse;
}

void IndividualController::setActuator(const Actuator& actuator) {
    updSocket<Actuator>("actuator").disconnect();
    updSocket<Actuator>("actuator").connect(actuator);
}
