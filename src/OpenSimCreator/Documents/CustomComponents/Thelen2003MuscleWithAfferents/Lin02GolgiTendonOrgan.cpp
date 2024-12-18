/* 
* The Lin02GolgiTendonOrgan class implements a model for the aggregate
* population response of the Golgi tendon organs in a muscle.
* The model is taken from: Lin & Crago, "Neural and Mechanical Contributions
* to the Stretch Reflex: A Model Synthesis" Ann Biomed Eng 30:54-67.
*
* Objects of this class are meant to be contained in particular muscle
* objects, such as those of the Thelen2003MuscleWithAfferents class.
*
* @author Sergio Verduzco Flores
*/

#include <iostream>  // remove later
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include "Thelen2003MuscleWithAfferents.h"
#include <cmath>  // for the log() function

using namespace OpenSim;

// The state variables corresponding to the entries in derivs are:
// derivs[0] = low-pass filtered output of Eq. 1
// derivs[1] = derivative of the LPF output of Eq. 1
// derivs[2] = intermediate variable of the filter
// derivs[3] = output variable of the filter 

///@cond  
const std::string Lin02GolgiTendonOrgan::STATE_LPF_OUTPUT_NAME = "nonlinear";
const std::string Lin02GolgiTendonOrgan::STATE_LPF_DERIV_NAME = "nonlinear_deriv";
const std::string Lin02GolgiTendonOrgan::STATE_FILTER_INTER_NAME = "filter_out";
const std::string Lin02GolgiTendonOrgan::STATE_FILTER_OUTPUT_NAME = "filter_out_deriv";
const std::string Lin02GolgiTendonOrgan::CACHE_GTO_OUT_NAME = "gto_out";
///@endcond  

Lin02GolgiTendonOrgan::Lin02GolgiTendonOrgan()
{
	// Set parameter values
	Gg = 0.6;  	// pulses/s
	Gf = 10;		// Newtons
	thr = 0;	// pulses/s
	// Initialize work variables
	nl = 0; Dnl = 0; 
	ts[4] = -0.004; ts[3] = -0.003; ts[2] = -0.002; ts[1] = -0.001; ts[0] = 0.0;

	for (unsigned int i = 0; i < GTO_SMOOTHING_WINDOW; i++) {
		nl_approx_xs[i] = 0.0;
		nl_approx_ts[i] = 0.0;
		dnl_approx_nls[i] = 0.0;
	}
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================

void Lin02GolgiTendonOrgan::extendInitStateFromProperties(SimTK::State& s) const
{
	musclePtr->setX(s,0.0);
	musclePtr->setXp(s,0.0);
	musclePtr->setY(s, 0.0);
	musclePtr->setZ(s, 0.0);
	// Initialize the work variables 
	nl = 0; Dnl = 0;
	ts[4] = -0.004; ts[3] = -0.003; ts[2] = -0.002; ts[1] = -0.001; ts[0] = 0.0;
}

//=============================================================================
// Derivative computation
//=============================================================================
SimTK::Vector Lin02GolgiTendonOrgan::
computeStateVariableDerivatives(const SimTK::State& s) const
{
	SimTK::Vector derivs(4, SimTK::NaN);

	// The state variables corresponding to the entries in derivs are:
	// derivs[0] = low-pass filtered output of Eq. 1
	// derivs[1] = derivative of the LPF output of Eq. 1
	// derivs[2] = intermediate variable of the filter
	// derivs[3] = output variable of the filter 

	// value in Eq. 1
	double non_lin = Gg * std::log((musclePtr->getFiberForce(s) / Gf) + 1);
	derivs[0] = (non_lin - musclePtr->getX(s)) / musclePtr->getLPFtau();

	// Getting derivatives of the muscle force
	SimTK::Vec<2> diff = calculateDerivatives(s);
	// diff(0) = derivative of LPF nonlinearity
	// diff(1) = derivative of LPF diff(0) 
	derivs[1] = (diff(0) - musclePtr->getXp(s)) / musclePtr->getLPFtau();

	double Xpp = diff(1);

	// the variable Z is the derivative of the output Y
	derivs[2] = musclePtr->getZ(s);

	// The transfer function, as in the 5/16/16 notes
	// This is the derivative of the Z variable
	derivs[3] = -2.2 * musclePtr->getZ(s) - 0.4 * musclePtr->getY(s)
		+ 68.0 * Xpp + 103.2 * musclePtr->getXp(s) + 16.0 * musclePtr->getX(s);
	
	// putting the output of the GTO in a cache variable
	// and making sure it is not negative
	double& out = musclePtr->updGTOout(s);

	out = (musclePtr->getY(s) > thr) ? musclePtr->getY(s) : 0.0;
	musclePtr->markCacheVariableValid(s, CACHE_GTO_OUT_NAME);

	return derivs;
}

//=============================================================================
// Initializing values
//=============================================================================
void Lin02GolgiTendonOrgan::initFromMuscle(SimTK::State& s) const
{
	musclePtr->setXp(s, 0.0);
	double nonLin = Gg * std::log((musclePtr->getFiberForce(s) / Gf) + 1);
	musclePtr->setX(s, nonLin);
	musclePtr->setY(s, 40.0 * nonLin);  // this is the fixed point
	musclePtr->setZ(s, 0.0);
	// the work variables too
	nl = 0; Dnl = 0;
	ts[4] = -0.004; ts[3] = -0.003; ts[2] = -0.002; ts[1] = -0.001; ts[0] = 0.0;

	for (unsigned int i = 0; i < GTO_SMOOTHING_WINDOW; i++) {
		nl_approx_xs[i] = nonLin;
		nl_approx_ts[i] = 0.0;
		dnl_approx_nls[i] = 0.0;
	}
}


//--------------------------------------------------------------------------
// Approximate the derivatives required by Eq. 2
//--------------------------------------------------------------------------
// HO : Originally calculateDerivs used a more sophisticated way to estimate the
// derivatives but it doesn't play nice with a static time step which we need
// for syncing with the neural simulation so we just use the less accurate, naive, but
// faster solution.
//SimTK::Vec<2> Lin02GolgiTendonOrgan::
//calculateDerivatives(const SimTK::State& s) const
//{
//	// three point formula
//	double curr_nl = getX(s);
//	double curr_Dnl = getXp(s);
//	double curr_time = s.getTime();	// time in the simulation
//	// diff(0) = LPF derivative of nonlinear output
//	// diff(1) = derivative of diff(0)'s variable 
//	SimTK::Vec<2> diff;
//
//	if (curr_time - ts(0) > 0.05) {
//		if (ts(3) >= 0.0) {
//			diff(0) = ((-2 * nl(2)) + (9 * nl(1)) - (18 * nl(0)) + (11 * curr_nl)) / (6 * (curr_time - ts(0)));
//			diff(1) = ((-2 * Dnl(2)) + (9 * Dnl(1)) - (18 * Dnl(0)) + (11 * curr_Dnl)) / (6 * (curr_time - ts(0)));
//		}
//		else {
//			diff(0) = 0.0;
//			diff(1) = 0.0;
//		}
//
//		ts(4) = ts(3); ts(3) = ts(2); ts(2) = ts(1); ts(1) = ts(0);
//		ts(0) = curr_time;
//
//		nl(4) = nl(3); nl(3) = nl(2); nl(2) = nl(1); nl(1) = nl(0);
//		nl(0) = curr_nl;
//
//		Dnl(4) = Dnl(3); Dnl(3) = Dnl(2); Dnl(2) = Dnl(1); Dnl(1) = Dnl(0);
//		Dnl(0) = curr_Dnl;
//	}
//	else {
//		diff(0) = 0.0;
//		diff(1) = 0.0;
//
//		ts(4) = ts(3); ts(3) = ts(2); ts(2) = ts(1); ts(1) = ts(0);
//		ts(0) = curr_time;
//
//		nl(4) = nl(3); nl(3) = nl(2); nl(2) = nl(1); nl(1) = nl(0);
//		nl(0) = curr_nl;
//
//		Dnl(4) = Dnl(3); Dnl(3) = Dnl(2); Dnl(2) = Dnl(1); Dnl(1) = Dnl(0);
//		Dnl(0) = curr_Dnl;
//	}
//
//	return diff;
//}

//--------------------------------------------------------------------------
// Approximate the derivatives required by Eq. 2
//--------------------------------------------------------------------------
// HO : Originally calculateDerivs used a more sophisticated way to estimate the
// derivatives but it doesn't play nice with a static time step which we need
// for syncing with the neural simulation so we just use the less accurate, naive, but
// faster solution.
SimTK::Vec<2> Lin02GolgiTendonOrgan::
calculateDerivatives(const SimTK::State& s) const
{
	// three point formula
	double curr_nl = musclePtr->getX(s);
	double curr_Dnl = musclePtr->getXp(s);
	double curr_time = s.getTime();	// time in the simulation
	// diff(0) = LPF derivative of nonlinear output
	// diff(1) = derivative of diff(0)'s variable 
	SimTK::Vec<2> diff;

	const unsigned int smooth_window = GTO_SMOOTHING_WINDOW;
	double diff0s[GTO_SMOOTHING_WINDOW], diff1s[GTO_SMOOTHING_WINDOW];

	for (unsigned int i = 0; i < smooth_window; i++) {
		diff0s[i] = 0.0;
		diff1s[i] = 0.0;
	}
		
	if (nl_approx_ts[smooth_window - 2] - nl_approx_ts[smooth_window - 1] > 0) {
		if (curr_time - nl_approx_ts(0) > 0.0) {
			diff0s[0] = (curr_nl - nl_approx_xs(0)) / (curr_time - nl_approx_ts(0));
			diff1s[0] = (curr_Dnl - dnl_approx_nls(0)) / (curr_time - nl_approx_ts(0));
			diff[0] += diff0s[0];
			diff[1] += diff1s[0];
			for (unsigned int i = 1; i < smooth_window; i++) {
				diff0s[i] = (nl_approx_xs(i - 1) - nl_approx_xs(i)) / (nl_approx_ts(i - 1) - nl_approx_ts(i));
				diff1s[i] = (dnl_approx_nls(i - 1) - dnl_approx_nls(i)) / (nl_approx_ts(i - 1) - nl_approx_ts(i));
				diff[0] += diff0s[i];
				diff[1] += diff1s[i];
			}

			diff[0] /= (double)smooth_window;
			diff[1] /= (double)smooth_window;

			for (unsigned int i = smooth_window - 1; i > 0; i--) {
				nl_approx_ts[i] = nl_approx_ts[i - 1];
				nl_approx_xs[i] = nl_approx_xs[i - 1];
				dnl_approx_nls[i] = dnl_approx_nls[i - 1];
			}
			nl_approx_ts[0] = curr_time;
			nl_approx_xs[0] = curr_nl;
			dnl_approx_nls[0] = curr_Dnl;
		}
		else {
			diff(0) = 0.0;
			diff(1) = 0.0;
		}
	}
	else {
		if (curr_time - nl_approx_ts(0) > 0.0) {
			diff0s[0] = (curr_nl - nl_approx_xs(0)) / (curr_time - nl_approx_ts(0));
			diff1s[0] = (curr_Dnl - dnl_approx_nls(0)) / (curr_time - nl_approx_ts(0));

			for (unsigned int i = smooth_window - 1; i > 0; i--) {
				nl_approx_ts[i] = nl_approx_ts[i - 1];
				nl_approx_xs[i] = nl_approx_xs[i - 1];
				dnl_approx_nls[i] = dnl_approx_nls[i - 1];
			}
			nl_approx_ts[0] = curr_time;
			nl_approx_xs[0] = curr_nl;
			dnl_approx_nls[0] = curr_Dnl;

			diff(0) = diff0s[0];
			diff(1) = diff1s[0];
		}
		else {
			diff(0) = 0.0;
			diff(1) = 0.0;
		}
	}

	return diff;
}