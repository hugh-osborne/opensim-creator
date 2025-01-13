//=============================================================================
// INCLUDES
//=============================================================================
#include "SynapseConnection.h"
#include "NeuralPopulation.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <random>

//=============================================================================
// STATICS
//=============================================================================
using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
SynapseConnection::SynapseConnection() :
    ModelComponent{} {

}

SynapseConnection::~SynapseConnection() = default;

void SynapseConnection::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);

    getSocket<NeuralPopulation>("OutputPopulation").getConnectee().registerIncomingConnection(this);
}
