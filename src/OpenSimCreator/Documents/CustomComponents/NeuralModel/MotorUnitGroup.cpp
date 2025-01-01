//=============================================================================
// INCLUDES
//=============================================================================
#include "MotorUnitGroup.h"
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
MotorUnitGroup::MotorUnitGroup() :
        ModelComponent{}{
    constructProperties();
}

MotorUnitGroup::~MotorUnitGroup() = default;

void MotorUnitGroup::constructProperties() {
    setAuthors("Hugh Osborne based on work by Ajay Seth, Frank Anderson, Chand John, Samuel Hamner");
    constructProperty_enabled(true);
}

//=============================================================================
// GET AND SET
//=============================================================================
bool MotorUnitGroup::isEnabled() const {
    return get_enabled();
}

void MotorUnitGroup::setEnabled(bool aTrueFalse) {
    upd_enabled() = aTrueFalse;
}