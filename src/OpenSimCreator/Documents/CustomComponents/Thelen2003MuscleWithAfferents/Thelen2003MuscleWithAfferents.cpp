/* -------------------------------------------------------------------------- *
 *                 OpenSim:  Thelen2003MuscleWithAfferents.cpp               *
 * -------------------------------------------------------------------------- *
 */
 
//=============================================================================
// INCLUDES
//=============================================================================
#include "Thelen2003MuscleWithAfferents.h"
#include <iostream>  // remove later

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

///@cond  
const std::string Thelen2003MuscleWithAfferents::STATE_LPF_VELOCITY_NAME = "LPF_velocity";
const std::string Thelen2003MuscleWithAfferents::STATE_LPF_ACCELERATION_NAME = "LPF_acceleration";
// The below names are declared private in Thelen2003MuscleWithAfferents so we have to duplicate them here. Boooo!
const string Thelen2003MuscleWithAfferents::AFF_STATE_ACTIVATION_NAME = "activation";
const string Thelen2003MuscleWithAfferents::AFF_STATE_FIBER_LENGTH_NAME = "fiber_length";
///@endcond  

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
/*
 * Default constructor.
 */
Thelen2003MuscleWithAfferents::Thelen2003MuscleWithAfferents() : Thelen2003Muscle()
{
	constructProperties();
}

/*
 * Constructor.
 */
Thelen2003MuscleWithAfferents::Thelen2003MuscleWithAfferents(const std::string &name, 
					double maxIsometricForce, double optimalFiberLength, 
					double tendonSlackLength, double pennationAngle) : 
	Super(name, maxIsometricForce, optimalFiberLength, tendonSlackLength, 
          pennationAngle)
{
	constructProperties();
}

Thelen2003MuscleWithAfferents::Thelen2003MuscleWithAfferents(const Thelen2003Muscle& muscle)
: Super(muscle.getName(), muscle.getMaxIsometricForce(), muscle.getOptimalFiberLength(), muscle.getTendonSlackLength(), 
          muscle.getPennationAngleAtOptimalFiberLength())
{
	constructProperties();
}

/*
 * Construct and initialize properties.
 * All properties are added to the property set. Once added, they can be
 * read in and written to files.
 */
void Thelen2003MuscleWithAfferents::constructProperties()
{
	setAuthors("Sergio Verduzco from code by Ajay Seth");
	constructProperty_lpf_tau(1.0); // LPF time constant
	constructProperty_lpf_default_activation(0.05);

	for (unsigned int i = 0; i < SMOOTHING_WINDOW; i++) {
		acc_approx_vels[i] = 0.0;
		acc_approx_ts[i] = 0.0;
		vel_approx_lens[i] = 0.0;
		vel_approx_ts[i] = 0.0;
	}
}

// Define new states and their derivatives in the underlying system
void Thelen2003MuscleWithAfferents::extendAddToSystem(SimTK::MultibodySystem& system) const
{
	// Allow Thelen2003MuscleWithAfferents to add its states, cache, etc.
	// to the system
	Super::extendAddToSystem(system);
	
	// low-pass filtered state variables used to calculate derivatives 
	addStateVariable(STATE_LPF_VELOCITY_NAME); // fiber velocity
	addStateVariable(STATE_LPF_ACCELERATION_NAME); // fiber acceleration

	// Spindle

	addStateVariable(Mileusnic06Spindle::STATE_dynamic_activation_NAME);
	addStateVariable(Mileusnic06Spindle::STATE_static_activation_NAME);
	addStateVariable(Mileusnic06Spindle::STATE_tension_bag1_NAME);
	addStateVariable(Mileusnic06Spindle::STATE_tension_bag2_NAME);
	addStateVariable(Mileusnic06Spindle::STATE_tension_chain_NAME);
	addStateVariable(Mileusnic06Spindle::STATE_tension_bag1_deriv_NAME); // the first derivatives are also
	addStateVariable(Mileusnic06Spindle::STATE_tension_bag2_deriv_NAME); // state variables
	addStateVariable(Mileusnic06Spindle::STATE_tension_chain_deriv_NAME);

	addCacheVariable(Mileusnic06Spindle::CACHE_primaryIa_NAME, 0.0, SimTK::Stage::Dynamics);
	addCacheVariable(Mileusnic06Spindle::CACHE_secondaryII_NAME, 0.0, SimTK::Stage::Dynamics);

	// GTO

	addStateVariable(Lin02GolgiTendonOrgan::STATE_LPF_OUTPUT_NAME); // LPF output of Eq. 1
	addStateVariable(Lin02GolgiTendonOrgan::STATE_LPF_DERIV_NAME);
	addStateVariable(Lin02GolgiTendonOrgan::STATE_FILTER_INTER_NAME);
	addStateVariable(Lin02GolgiTendonOrgan::STATE_FILTER_OUTPUT_NAME); // output of Eq. 2

	// a cache variable for the aggregate output of the GTOs
	addCacheVariable(Lin02GolgiTendonOrgan::CACHE_GTO_OUT_NAME, 0.0, SimTK::Stage::Dynamics);

}

void Thelen2003MuscleWithAfferents::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);
	
	// I'll init the state, but not from properties
	setLPFvelocity(s, 0.0);
	setLPFacceleration(s, 0.0);

	// Spindle

	spindle.extendInitStateFromProperties(s);

	// GTO

	GTO.extendInitStateFromProperties(s);
}

void Thelen2003MuscleWithAfferents::extendSetPropertiesFromState(const SimTK::State& s)
{
	Super::extendSetPropertiesFromState(s);

	setLPFDefaultActivation(getDynamicActivation(s)); // ignoring static activation
}

void Thelen2003MuscleWithAfferents::extendConnectToModel(Model& aModel)
{
	Super::extendConnectToModel(aModel);

	spindle.setParentMuscle(this);
	GTO.setParentMuscle(this);
}

//--------------------------------------------------------------------------
// GET & SET Properties
//--------------------------------------------------------------------------
void Thelen2003MuscleWithAfferents::setLPFtau(double aLPFtau) {
	set_lpf_tau(aLPFtau);
}

void Thelen2003MuscleWithAfferents::setLPFDefaultActivation(double aLPFDefaultActivation) {
	set_lpf_default_activation(aLPFDefaultActivation);
}

// Spindle

// dynamic_activation
double Thelen2003MuscleWithAfferents::getDynamicActivation(const SimTK::State & s) const {
	return getStateVariableValue(s, Mileusnic06Spindle::STATE_dynamic_activation_NAME);
}
void Thelen2003MuscleWithAfferents::setDynamicActivation(SimTK::State& s, double Activation) const {
	setStateVariableValue(s, Mileusnic06Spindle::STATE_dynamic_activation_NAME, Activation);
}
// static_activation
double Thelen2003MuscleWithAfferents::getStaticActivation(const SimTK::State& s) const {
	return getStateVariableValue(s, Mileusnic06Spindle::STATE_static_activation_NAME);
}
void Thelen2003MuscleWithAfferents::setStaticActivation(SimTK::State& s, double Activation) const {
	setStateVariableValue(s, Mileusnic06Spindle::STATE_static_activation_NAME, Activation);
}
// bag1 tension
double Thelen2003MuscleWithAfferents::getTensionBag1(const SimTK::State& s) const {
	return getStateVariableValue(s, Mileusnic06Spindle::STATE_tension_bag1_NAME);
}
void Thelen2003MuscleWithAfferents::setTensionBag1(SimTK::State& s, double Tension) const {
	setStateVariableValue(s, Mileusnic06Spindle::STATE_tension_bag1_NAME, Tension);
}
double Thelen2003MuscleWithAfferents::getTensionBag1Deriv(const SimTK::State& s) const {
	return getStateVariableValue(s, Mileusnic06Spindle::STATE_tension_bag1_deriv_NAME);
}
void Thelen2003MuscleWithAfferents::setTensionBag1Deriv(SimTK::State& s,
	double TensionDeriv) const {
	setStateVariableValue(s, Mileusnic06Spindle::STATE_tension_bag1_deriv_NAME, TensionDeriv);
}
// bag2 tension
double Thelen2003MuscleWithAfferents::getTensionBag2(const SimTK::State& s) const {
	return getStateVariableValue(s, Mileusnic06Spindle::STATE_tension_bag2_NAME);
}
void Thelen2003MuscleWithAfferents::setTensionBag2(SimTK::State& s, double Tension) const {
	setStateVariableValue(s, Mileusnic06Spindle::STATE_tension_bag2_NAME, Tension);
}
double Thelen2003MuscleWithAfferents::getTensionBag2Deriv(const SimTK::State& s) const {
	return getStateVariableValue(s, Mileusnic06Spindle::STATE_tension_bag2_deriv_NAME);
}
void Thelen2003MuscleWithAfferents::setTensionBag2Deriv(SimTK::State& s,
	double TensionDeriv) const {
	setStateVariableValue(s, Mileusnic06Spindle::STATE_tension_bag2_deriv_NAME, TensionDeriv);
}
// chain tension
double Thelen2003MuscleWithAfferents::getTensionChain(const SimTK::State& s) const {
	return getStateVariableValue(s, Mileusnic06Spindle::STATE_tension_chain_NAME);
}
void Thelen2003MuscleWithAfferents::setTensionChain(SimTK::State& s, double Tension) const {
	setStateVariableValue(s, Mileusnic06Spindle::STATE_tension_chain_NAME, Tension);
}
double Thelen2003MuscleWithAfferents::getTensionChainDeriv(const SimTK::State& s) const {
	return getStateVariableValue(s, Mileusnic06Spindle::STATE_tension_chain_deriv_NAME);
}
void Thelen2003MuscleWithAfferents::setTensionChainDeriv(SimTK::State& s,
	double TensionDeriv) const {
	setStateVariableValue(s, Mileusnic06Spindle::STATE_tension_chain_deriv_NAME, TensionDeriv);
}
// output variables
double Thelen2003MuscleWithAfferents::getIaOutput(const SimTK::State& s) const {
	return getCacheVariableValue<double>(s, Mileusnic06Spindle::CACHE_primaryIa_NAME);
}
double& Thelen2003MuscleWithAfferents::updIaOutput(const SimTK::State& s) const
{
	return updCacheVariableValue<double>(s, Mileusnic06Spindle::CACHE_primaryIa_NAME);
}
double Thelen2003MuscleWithAfferents::getIIOutput(const SimTK::State& s) const {
	return getCacheVariableValue<double>(s, Mileusnic06Spindle::CACHE_secondaryII_NAME);
}
double& Thelen2003MuscleWithAfferents::updIIOutput(const SimTK::State& s) const
{
	return updCacheVariableValue<double>(s, Mileusnic06Spindle::CACHE_secondaryII_NAME);
}

// GTO

double Thelen2003MuscleWithAfferents::getX(const SimTK::State& s) const {
	return getStateVariableValue(s, Lin02GolgiTendonOrgan::STATE_LPF_OUTPUT_NAME);
}
void Thelen2003MuscleWithAfferents::setX(SimTK::State& s, double X) const {
	setStateVariableValue(s, Lin02GolgiTendonOrgan::STATE_LPF_OUTPUT_NAME, X);
}
// derivative of the LPF output of the log nonlinearity
double Thelen2003MuscleWithAfferents::getXp(const SimTK::State& s) const {
	return getStateVariableValue(s, Lin02GolgiTendonOrgan::STATE_LPF_DERIV_NAME);
}
void Thelen2003MuscleWithAfferents::setXp(SimTK::State& s, double Xp) const {
	setStateVariableValue(s, Lin02GolgiTendonOrgan::STATE_LPF_DERIV_NAME, Xp);
}
// Intermediate variable for the transfer function filter
double Thelen2003MuscleWithAfferents::getY(const SimTK::State& s) const {
	return getStateVariableValue(s, Lin02GolgiTendonOrgan::STATE_FILTER_INTER_NAME);
}
void Thelen2003MuscleWithAfferents::setY(SimTK::State& s, double Y) const {
	setStateVariableValue(s, Lin02GolgiTendonOrgan::STATE_FILTER_INTER_NAME, Y);
}
// output variable of the transfer function
double Thelen2003MuscleWithAfferents::getZ(const SimTK::State& s) const {
	return getStateVariableValue(s, Lin02GolgiTendonOrgan::STATE_FILTER_OUTPUT_NAME);
}
void Thelen2003MuscleWithAfferents::setZ(SimTK::State& s, double Z) const {
	setStateVariableValue(s, Lin02GolgiTendonOrgan::STATE_FILTER_OUTPUT_NAME, Z);
}
// output of the Golgi tendon organ
double Thelen2003MuscleWithAfferents::getGTOout(const SimTK::State& s) const {
	return getCacheVariableValue<double>(s, Lin02GolgiTendonOrgan::CACHE_GTO_OUT_NAME);
}

double& Thelen2003MuscleWithAfferents::updGTOout(const SimTK::State& s) const
{
	return updCacheVariableValue<double>(s, Lin02GolgiTendonOrgan::CACHE_GTO_OUT_NAME);
}


//--------------------------------------------------------------------------
// GET & SET States and their derivatives
//--------------------------------------------------------------------------

double Thelen2003MuscleWithAfferents::getLPFvelocity(const SimTK::State& s) const {
	return getStateVariableValue(s, STATE_LPF_VELOCITY_NAME);
}	
void Thelen2003MuscleWithAfferents::setLPFvelocity(SimTK::State& s, double Velocity) const {
	setStateVariableValue(s, STATE_LPF_VELOCITY_NAME, Velocity);
}	
double Thelen2003MuscleWithAfferents::getLPFacceleration(const SimTK::State& s) const {
	return getStateVariableValue(s, STATE_LPF_ACCELERATION_NAME);
}
void Thelen2003MuscleWithAfferents::setLPFacceleration(SimTK::State& s, double Acceleration) const {
	setStateVariableValue(s, STATE_LPF_ACCELERATION_NAME, Acceleration);
}

//=============================================================================
// COMPUTATION
//=============================================================================
void Thelen2003MuscleWithAfferents::
computeInitialFiberEquilibrium(SimTK::State& s) const
{
	// First let the muscle find an equilibrium state
	Super::computeInitialFiberEquilibrium(s);
	
	setLPFvelocity(s, 0.0);
	// a simplifying assumption is a steady state
	setLPFacceleration(s, 0.0); 

	for (unsigned int i = 0; i < SMOOTHING_WINDOW; i++) {
		acc_approx_vels[i] = 0.0;
		acc_approx_ts[i] = 0.0;
		vel_approx_lens[i] = getFiberLength(s);
		vel_approx_ts[i] = 0.0;
	}

	// Spindle

	spindle.computeInitialSpindleEquilibrium(s);

	// GTO

	GTO.initFromMuscle(s);
}

void Thelen2003MuscleWithAfferents::computeStateVariableDerivatives(const SimTK::State& s) const
{	
	Super::computeStateVariableDerivatives(s);

	// vector of the derivatives to be returned
	SimTK::Vector derivs(4, 0.0);


// This is the parent's computeStateVariableDerivatives
/*--------------------------------------------------------------------
	int idx = 0;

    if (!isDisabled(s)) {
        // Activation is the first state (if it is a state at all)
        if(!get_ignore_activation_dynamics() &&
           idx+1 <= getNumStateVariables()) {
               derivs[idx] = getActivationDerivative(s);
               idx++;
        }

        // Fiber length is the next state (if it is a state at all)
        if(!get_ignore_tendon_compliance() && idx+1 <= getNumStateVariables()) {
            derivs[idx] = getFiberVelocity(s);
        }
    }
--------------------------------------------------------------------*/
// This is a "carefree" version of that:
	derivs[0] = getActivationRate(s);
	derivs[1] = getFiberVelocity(s);

	// next state is the LPF velocity
	derivs[2] = (approxFiberVelocity(s) - getLPFvelocity(s)) / getLPFtau();

	// the LPF acceleration
	derivs[3] = (approxFiberAcceleration(s) - getLPFacceleration(s)) / getLPFtau();

	setStateVariableDerivativeValue(s, AFF_STATE_ACTIVATION_NAME, derivs[0]);
	setStateVariableDerivativeValue(s, AFF_STATE_FIBER_LENGTH_NAME, derivs[1]);
	setStateVariableDerivativeValue(s, STATE_LPF_VELOCITY_NAME, derivs[2]);
	setStateVariableDerivativeValue(s, STATE_LPF_ACCELERATION_NAME, derivs[3]);

	// The state variables corresponding to the entries in derivs are: 
	// derivs[0] --> Dynamic bag fiber activation
	// derivs[1] --> Static bag fiber activation
	// derivs[2] --> Dynamic bag fiber tension
	// derivs[3] --> Static bag fiber tension
	// derivs[4] --> Chain fiber tension
	// derivs[5] --> Dynamic bag fiber tension's derivative
	// derivs[6] --> Static bag fiber tension's derivative
	// derivs[7] --> Chain fiber tension's derivative

	auto spindle_derivs = spindle.computeStateVariableDerivatives(s);

	setStateVariableDerivativeValue(s, Mileusnic06Spindle::STATE_dynamic_activation_NAME, spindle_derivs[0]);
	setStateVariableDerivativeValue(s, Mileusnic06Spindle::STATE_static_activation_NAME, spindle_derivs[1]);
	setStateVariableDerivativeValue(s, Mileusnic06Spindle::STATE_tension_bag1_NAME, spindle_derivs[2]);
	setStateVariableDerivativeValue(s, Mileusnic06Spindle::STATE_tension_bag2_NAME, spindle_derivs[3]);
	setStateVariableDerivativeValue(s, Mileusnic06Spindle::STATE_tension_chain_NAME, spindle_derivs[4]);
	setStateVariableDerivativeValue(s, Mileusnic06Spindle::STATE_tension_bag1_deriv_NAME, spindle_derivs[5]);
	setStateVariableDerivativeValue(s, Mileusnic06Spindle::STATE_tension_bag2_deriv_NAME, spindle_derivs[6]);
	setStateVariableDerivativeValue(s, Mileusnic06Spindle::STATE_tension_chain_deriv_NAME, spindle_derivs[7]);

	auto gto_derivs = GTO.computeStateVariableDerivatives(s);

	// The state variables corresponding to the entries in derivs are:
	// derivs[0] = low-pass filtered output of Eq. 1
	// derivs[1] = derivative of the LPF output of Eq. 1
	// derivs[2] = intermediate variable of the filter
	// derivs[3] = output variable of the filter 

	setStateVariableDerivativeValue(s, Lin02GolgiTendonOrgan::STATE_LPF_OUTPUT_NAME, gto_derivs[0]);
	setStateVariableDerivativeValue(s, Lin02GolgiTendonOrgan::STATE_LPF_DERIV_NAME, gto_derivs[1]);
	setStateVariableDerivativeValue(s, Lin02GolgiTendonOrgan::STATE_FILTER_INTER_NAME, gto_derivs[2]);
	setStateVariableDerivativeValue(s, Lin02GolgiTendonOrgan::STATE_FILTER_OUTPUT_NAME, gto_derivs[3]);
}

//--------------------------------------------------------------------------
// Approximate the muscle fiber acceleration
//--------------------------------------------------------------------------
// HO : Originally approxFiberAcceleration used a more sophisticated way to estimate the
// acceleration but it doesn't play nice with a static time step which we need
// for syncing with the neural simulation so we just use the less accurate, naive, but
// faster solution just using the previous value.
double Thelen2003MuscleWithAfferents::
approxFiberVelocity(const SimTK::State& s) const
{
	double curr_time = s.getTime();
	double curr_vel = getFiberLength(s);
	const unsigned int smooth_window = SMOOTHING_WINDOW;
	double v, vs[smooth_window];
	v = 0.0;
	for (unsigned int i = 0; i < smooth_window; i++)
		vs[i] = 0.0;

	if (vel_approx_ts[smooth_window - 2] - vel_approx_ts[smooth_window - 1] > 0) {
		if (curr_time - vel_approx_ts(0) > 0.0) {
			vs[0] = (curr_vel - vel_approx_lens(0)) / (curr_time - vel_approx_ts(0));
			v += vs[0];
			for (unsigned int i = 1; i < smooth_window; i++) {
				vs[i] = (vel_approx_lens(i - 1) - vel_approx_lens(i)) / (vel_approx_ts(i - 1) - vel_approx_ts(i));
				v += vs[i];
			}

			v /= (double)smooth_window;

			for (unsigned int i = smooth_window - 1; i > 0; i--) {
				vel_approx_ts[i] = vel_approx_ts[i - 1];
				vel_approx_lens[i] = vel_approx_lens[i - 1];
			}
			vel_approx_ts[0] = curr_time;
			vel_approx_lens[0] = curr_vel;
		}
		else {
			v = 0.0;
		}
	}
	else {
		if (curr_time - vel_approx_ts(0) > 0.0) {
			vs[0] = (curr_vel - vel_approx_lens(0)) / (curr_time - vel_approx_ts(0));

			for (unsigned int i = smooth_window - 1; i > 0; i--) {
				vel_approx_ts[i] = vel_approx_ts[i - 1];
				vel_approx_lens[i] = vel_approx_lens[i - 1];
			}
			vel_approx_ts[0] = curr_time;
			vel_approx_lens[0] = curr_vel;

			v = vs[0];
		}
		else {
			v = 0.0;
		}
	}

	return v;
}

//--------------------------------------------------------------------------
// Approximate the muscle fiber acceleration
//--------------------------------------------------------------------------
// HO : Originally approxFiberAcceleration used a more sophisticated way to estimate the
// acceleration but it doesn't play nice with a static time step which we need
// for syncing with the neural simulation so we just use the less accurate, naive, but
// faster solution just using the previous value.
double Thelen2003MuscleWithAfferents::
approxFiberAcceleration(const SimTK::State& s) const
{
	double curr_time = s.getTime();
	double curr_vel = getLPFvelocity(s);
	const unsigned int smooth_window = SMOOTHING_WINDOW;
	double v, vs[smooth_window];
	v = 0.0;
	for (unsigned int i = 0; i < smooth_window; i++)
		vs[i] = 0.0;

	if (acc_approx_ts[smooth_window - 2] - acc_approx_ts[smooth_window - 1] > 0) {
		if (curr_time - acc_approx_ts(0) > 0.0) {
			vs[0] = (curr_vel - acc_approx_vels(0)) / (curr_time - acc_approx_ts(0));
			v += vs[0];
			for (unsigned int i = 1; i < smooth_window; i++) {
				vs[i] = (acc_approx_vels(i - 1) - acc_approx_vels(i)) / (acc_approx_ts(i - 1) - acc_approx_ts(i));
				v += vs[i];
			}

			v /= (double)smooth_window;

			for (unsigned int i = smooth_window - 1; i > 0; i--) {
				acc_approx_ts[i] = acc_approx_ts[i - 1];
				acc_approx_vels[i] = acc_approx_vels[i - 1];
			}
			acc_approx_ts[0] = curr_time;
			acc_approx_vels[0] = curr_vel;
		}
		else {
			v = 0.0;
		}
	}
	else {
		if (curr_time - acc_approx_ts(0) > 0.0) {
			vs[0] = (curr_vel - acc_approx_vels(0)) / (curr_time - acc_approx_ts(0));

			for (unsigned int i = smooth_window - 1; i > 0; i--) {
				acc_approx_ts[i] = acc_approx_ts[i - 1];
				acc_approx_vels[i] = acc_approx_vels[i - 1];
			}
			acc_approx_ts[0] = curr_time;
			acc_approx_vels[0] = curr_vel;

			v = vs[0];
		}
		else {
			v = 0.0;
		}
	}

	return v;
}