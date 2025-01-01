//=============================================================================
// INCLUDES
//=============================================================================
#include "NeuralPopulation.h"
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
NeuralPopulation::NeuralPopulation() :
        ModelComponent{}{
    constructProperties();
}

NeuralPopulation::~NeuralPopulation() = default;

void NeuralPopulation::constructProperties() {
    setAuthors("Hugh Osborne based on work by Ajay Seth, Frank Anderson, Chand John, Samuel Hamner");
    constructProperty_enabled(true);
}

//=============================================================================
// GET AND SET
//=============================================================================
bool NeuralPopulation::isEnabled() const {
    return get_enabled();
}

void NeuralPopulation::setEnabled(bool aTrueFalse) {
    upd_enabled() = aTrueFalse;
}