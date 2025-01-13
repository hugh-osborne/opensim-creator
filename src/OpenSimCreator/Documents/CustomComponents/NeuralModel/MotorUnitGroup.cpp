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
// STATE VARIABLE NAMES
//=============================================================================
const std::string MotorUnitGroup::STATE_MUSCLE_EXCITATION_NAME= "muscle_excitation";

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
MotorUnitGroup::MotorUnitGroup() :
        NeuralPopulation{}{
    constructProperties();
}

MotorUnitGroup::~MotorUnitGroup() = default;

void MotorUnitGroup::constructProperties() {
    setAuthors("Hugh Osborne");

    constructProperty_multiplier(1.0);
}

//=============================================================================
// GET AND SET
//=============================================================================

double MotorUnitGroup::getMuscleExcitation(const SimTK::State& s) const {
    return get_multiplier() * (getTotalSpikes(s) / get_num_neurons());
}