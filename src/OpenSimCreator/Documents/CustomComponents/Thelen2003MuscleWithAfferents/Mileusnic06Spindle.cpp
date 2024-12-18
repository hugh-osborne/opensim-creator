
//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>  // for warning mesages
#include <cmath>  // for the pow() function
#include <stdlib.h> // for the abs function
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include "Thelen2003MuscleWithAfferents.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;

///@cond  
const std::string Mileusnic06Spindle::STATE_dynamic_activation_NAME = "dynamic_activation";
const std::string Mileusnic06Spindle::STATE_static_activation_NAME = "static_activation";
const std::string Mileusnic06Spindle::STATE_tension_bag1_NAME = "tension_bag1";
const std::string Mileusnic06Spindle::STATE_tension_bag2_NAME = "tension_bag2";
const std::string Mileusnic06Spindle::STATE_tension_chain_NAME = "tension_chain";
const std::string Mileusnic06Spindle::STATE_tension_bag1_deriv_NAME = "tension_bag1_deriv";
const std::string Mileusnic06Spindle::STATE_tension_bag2_deriv_NAME = "tension_bag2_deriv";
const std::string Mileusnic06Spindle::STATE_tension_chain_deriv_NAME = "tension_chain_deriv";
const std::string Mileusnic06Spindle::CACHE_primaryIa_NAME = "primaryIa";
const std::string Mileusnic06Spindle::CACHE_secondaryII_NAME = "secondaryII";
///@endcond  

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================

Mileusnic06Spindle::Mileusnic06Spindle() 
{
	// one parameter value that I didn't set anywhere else
	S = 0.156;

	for (unsigned int i = 0; i < SPINDLE_SMOOTHING_WINDOW; i++) {
		bag1_approx_lens[i] = 0.0;
		bag1_approx_ts[i] = 0.0;
		bag2_approx_lens[i] = 0.0;
		bag2_approx_ts[i] = 0.0;
		chain_approx_lens[i] = 0.0;
		chain_approx_ts[i] = 0.0;
	}
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================

void Mileusnic06Spindle::extendInitStateFromProperties(SimTK::State& s) const
{	
	musclePtr->setDynamicActivation(s, musclePtr->getDefaultActivation());
	musclePtr->setStaticActivation(s, musclePtr->getDefaultActivation());
	
	// also initiate the state variables that don't come from properties
	musclePtr->setTensionBag1(s,0.05);
	musclePtr->setTensionBag1Deriv(s,0.0);
	musclePtr->setTensionBag2(s,0.05);
	musclePtr->setTensionBag2Deriv(s,0.0);
	musclePtr->setTensionChain(s,0.05);
	musclePtr->setTensionChainDeriv(s,0.0);
}


//=============================================================================
// COMPUTATION OF DERIVATIVES
//=============================================================================
 
SimTK::Vector Mileusnic06Spindle::computeStateVariableDerivatives(const SimTK::State& s) const
{
	// vector of the derivatives to be returned
	SimTK::Vector derivs(8, SimTK::NaN);

	// The state variables corresponding to the entries in derivs are: 
	// derivs[0] --> Dynamic bag fiber activation
	// derivs[1] --> Static bag fiber activation
	// derivs[2] --> Dynamic bag fiber tension
	// derivs[3] --> Static bag fiber tension
	// derivs[4] --> Chain fiber tension
	// derivs[5] --> Dynamic bag fiber tension's derivative
	// derivs[6] --> Static bag fiber tension's derivative
	// derivs[7] --> Chain fiber tension's derivative
	
	// The fusimotor inputs come from an AfferentController object
	// that provides 3 controls to the muscle. 
	SimTK::VectorView ctrlVec = musclePtr->getControls(s);
	// ctrlVec[0] --> muscle excitation
	// ctrlVec[1] --> dynamic   fusimotor inputs
	// ctrlVec[2] --> static fusimotor inputs

	// Sometimes, controls can come in as NaNs - very concerning.
	// Let's assume it's supposed to be 0 for now.
	double gssq = 0.0;
	if (!isnan(ctrlVec[2]))
		gssq = (ctrlVec[2]) * (ctrlVec[2]);

	double gdsq = 0.0;
	if (!isnan(ctrlVec[1]))
		gdsq = (ctrlVec[1]) * (ctrlVec[1]);

	// Calculating activation derivatives
	double fdsq = (bag1.freq)*(bag1.freq); // I could store these two 
	double fssq = (bag2.freq)*(bag2.freq); // to speed things up
	double fcsq = (chain.freq)*(chain.freq);

	// Dynamic Activation (Equation 1)
	derivs[0] = ( (gdsq/(gdsq+fdsq)) - musclePtr->getDynamicActivation(s) )/bag1.tau;
	// Static Activation (Equation 1)
	derivs[1] = ( (gssq/(gssq+fssq)) - musclePtr->getStaticActivation(s) )/bag2.tau;
	// calculating chain fiber activation, which is instantaneous (Equation 1)
	double ch_act;
	ch_act = gssq / ( gssq + fcsq );
	
	// Calculating damping terms (beta)
	double beta_bag1, beta_bag2, beta_chain;
	// dynamic damping term. Equation 4 with beta_2 = 0.
	beta_bag1 = bag1.beta_0 + bag1.beta_1* musclePtr->getDynamicActivation(s);
	// static bag damping term. Equation 4 with beta_1 = 0.
	beta_bag2 = bag2.beta_0 + bag2.beta_2* musclePtr->getStaticActivation(s);
	// chain damping term. Equation 4 with beta_1 = 0.
	beta_chain = chain.beta_0 + chain.beta_2*ch_act;
	
	// calculating the force generator terms (Gamma)
	double Gamma_bag1, Gamma_bag2, Gamma_chain;
	// bag1 force generator term. Equation 5 with Gamma_2 = 0
	Gamma_bag1 = bag1.Gamma_1* musclePtr->getDynamicActivation(s);
	// bag2 force generator term. Equation 5 with Gamma_1 = 0
	Gamma_bag2 = bag2.Gamma_2* musclePtr->getStaticActivation(s);
	// chain force generator term. Equation 5 with Gamma_1 = 0
	Gamma_chain = chain.Gamma_2*ch_act;

	// The first derivative of the tension state variables
	// comes from another set of state variables
	derivs[2] = std::isnan(musclePtr->getTensionBag1Deriv(s)) ? 0.0 : musclePtr->getTensionBag1Deriv(s);
	derivs[3] = std::isnan(musclePtr->getTensionBag2Deriv(s)) ? 0.0 : musclePtr->getTensionBag2Deriv(s);
	derivs[4] = std::isnan(musclePtr->getTensionChainDeriv(s)) ? 0.0 : musclePtr->getTensionChainDeriv(s);
	
	// Now let's compute the second derivative of the tension
	// (Eq. 6), and also get the afferent potentials (Eqs. 7, 8)
	
	// equation 6 needs some muscle information
	double L0 = musclePtr->getOptimalFiberLength();
	double L = musclePtr->getFiberLength(s)/L0;
	double Lp = (musclePtr->getLPFvelocity(s))/L0;
	double Lpp = (musclePtr->getLPFacceleration(s))/L0;
	double  C, term1, term2, T, Tp;	// auxiliary variables 
	
	// Tension 2nd derivative for bag1 
	C = (Lp>0.0) ? bag1.C_L : bag1.C_S;
	T = musclePtr->getTensionBag1(s);
	Tp = derivs[2];
	 
	term1 = C * beta_bag1 * sgn(Lp - (Tp/bag1.K_SR))
		  * std::pow(std::abs(Lp - (Tp/bag1.K_SR)), bag1.a)
		  * (L - bag1.L_0SR - (T/bag1.K_SR) - bag1.R);
	term2 = bag1.K_PR*(L - bag1.L_0SR - (T/bag1.K_SR) - bag1.L_0PR);
	
	double val = (bag1.K_SR / bag1.M) * (term1 + term2
		+ bag1.M * Lpp + Gamma_bag1 - T);

	derivs[5] = val;//smoothBag1SecondDeriv(s, val) * 0.001;

	// afferent potential for bag1 (equation 7)
	double APbag1; 
	//APbag1 = bag1.G * max( (T/bag1.K_SR) - (bag1.L_NSR - bag1.L_0SR) , 0.0);
	APbag1 = bag1.G * ( (T/bag1.K_SR) - (bag1.L_NSR - bag1.L_0SR) );			
			
	// Tension 2nd derivative for bag2 
	C = (Lp>0.0) ? bag2.C_L : bag2.C_S;
	T = musclePtr->getTensionBag2(s);
	Tp = derivs[3];
	 
	term1 = C * beta_bag2 * sgn(Lp - (Tp/bag2.K_SR))
		  * std::pow(std::abs(Lp - (Tp/bag2.K_SR)), bag2.a)
		  * (L - bag2.L_0SR - (T/bag2.K_SR) - bag2.R);
	term2 = bag2.K_PR*(L - bag2.L_0SR - (T/bag2.K_SR) - bag2.L_0PR);

	val = (bag2.K_SR / bag2.M) * (term1 + term2
		+ bag2.M * Lpp + Gamma_bag2 - T);
	
	derivs[6] = val;// smoothBag2SecondDeriv(s, val) * 0.001;
			
	// afferent potential for bag2 (equation 8 except G product)
	double APbag2;
	
	/*APbag2 = bag2.X * (bag2.L_sec/bag2.L_0SR)
	                * max((T/bag2.K_SR) - (bag2.L_NSR - bag2.L_0SR), 0.0)
		   + (1.0-bag2.X) * (bag2.L_sec/bag2.L_0PR)
					* max(L - (T/bag2.K_SR) - bag2.L_0SR - bag2.L_NPR, 0.0); */
	

	APbag2 = bag2.X * (bag2.L_sec/bag2.L_0SR)
	                * ((T/bag2.K_SR) - (bag2.L_NSR - bag2.L_0SR))
		   + (1.0-bag2.X) * (bag2.L_sec/bag2.L_0PR)
					* (L - (T/bag2.K_SR) - bag2.L_0SR - bag2.L_NPR); 
					
	// Tension 2nd derivative for the chain fiber
	C = (Lp>0.0) ? chain.C_L : chain.C_S;
	T = musclePtr->getTensionChain(s);
	Tp = derivs[4];
	 
	term1 = C * beta_chain * sgn(Lp - (Tp/chain.K_SR))
		  * std::pow(std::abs(Lp - (Tp/chain.K_SR)), chain.a)
		  * (L - chain.L_0SR - (T/chain.K_SR) - chain.R);	
	term2 = chain.K_PR*(L - chain.L_0SR - (T/chain.K_SR) - chain.L_0PR);
	
	val = (chain.K_SR / chain.M) * (term1 + term2
		+ chain.M * Lpp + Gamma_chain - T);

	derivs[7] = val;//smoothChainSecondDeriv(s, val) * 0.001;
			  
	// afferent potential for chain (equation 8 except G product)
	double APchain;
	
	/*APchain = chain.X * (chain.L_sec/chain.L_0SR)
	                  * max((T/chain.K_SR) - (chain.L_NSR - chain.L_0SR), 0.0)
		    + (1.0-chain.X) * (chain.L_sec/chain.L_0PR)
					  * max(L - (T/chain.K_SR) - chain.L_0SR - chain.L_NPR, 0.0);*/
	
	APchain = chain.X * (chain.L_sec/chain.L_0SR)
	                  * ((T/chain.K_SR) - (chain.L_NSR - chain.L_0SR))
		    + (1.0-chain.X) * (chain.L_sec/chain.L_0PR)
					  * (L - (T/chain.K_SR) - chain.L_0SR - chain.L_NPR);
			  
	// calculating the afferent firing
	double primary, secondary, pri_stat;
	pri_stat = bag2.G_pri*APbag2 + chain.G_pri*APchain;
	//primary = max(max(APbag1, pri_stat) + S * min(APbag1, pri_stat), 0.0);
	primary = max(APbag1 + (S * pri_stat), 0.0);
	secondary = max(bag2.G_sec*APbag2 + chain.G_sec*APchain, 0.0);
	
	// cache the output so it can be accessed
	double& iaout = musclePtr->updIaOutput(s);
    iaout = primary;
	musclePtr->markCacheVariableValid(s,CACHE_primaryIa_NAME);

	double& iiout = musclePtr->updIIOutput(s);
    iiout = secondary;
	musclePtr->markCacheVariableValid(s,CACHE_secondaryII_NAME);

	return derivs;
}

double Mileusnic06Spindle::
smoothBag1SecondDeriv(const SimTK::State& s, double curr_val) const
{
	double curr_time = s.getTime();
	double curr_vel = std::isnan(curr_val) ? 0.0 : curr_val;
	const unsigned int smooth_window = SPINDLE_SMOOTHING_WINDOW;
	double v, vs[smooth_window];
	v = 0.0;
	for (unsigned int i = 0; i < smooth_window; i++)
		vs[i] = 0.0;

	if (bag1_approx_ts[smooth_window - 2] - bag1_approx_ts[smooth_window - 1] > 0) {
		if (curr_time - bag1_approx_ts(0) > 0.0) {
			vs[0] = (curr_vel - bag1_approx_lens(0)) / (curr_time - bag1_approx_ts(0));
			v += vs[0];
			for (unsigned int i = 1; i < smooth_window; i++) {
				vs[i] = (bag1_approx_lens(i - 1) - bag1_approx_lens(i)) / (bag1_approx_ts(i - 1) - bag1_approx_ts(i));
				v += vs[i];
			}

			v /= (double)smooth_window;

			for (unsigned int i = smooth_window - 1; i > 0; i--) {
				bag1_approx_ts[i] = bag1_approx_ts[i - 1];
				bag1_approx_lens[i] = bag1_approx_lens[i - 1];
			}
			bag1_approx_ts[0] = curr_time;
			bag1_approx_lens[0] = curr_vel;
		}
		else {
			v = 0.0;
		}
	}
	else {
		if (curr_time - bag1_approx_ts(0) > 0.0) {
			vs[0] = (curr_vel - bag1_approx_lens(0)) / (curr_time - bag1_approx_ts(0));

			for (unsigned int i = smooth_window - 1; i > 0; i--) {
				bag1_approx_ts[i] = bag1_approx_ts[i - 1];
				bag1_approx_lens[i] = bag1_approx_lens[i - 1];
			}
			bag1_approx_ts[0] = curr_time;
			bag1_approx_lens[0] = curr_vel;

			v = vs[0];
		}
		else {
			v = 0.0;
		}
	}

	return v;
}

double Mileusnic06Spindle::
smoothBag2SecondDeriv(const SimTK::State& s, double curr_val) const
{
	double curr_time = s.getTime();
	double curr_vel = std::isnan(curr_val) ? 0.0 : curr_val;
	const unsigned int smooth_window = SPINDLE_SMOOTHING_WINDOW;
	double v, vs[smooth_window];
	v = 0.0;
	for (unsigned int i = 0; i < smooth_window; i++)
		vs[i] = 0.0;

	if (bag2_approx_ts[smooth_window - 2] - bag2_approx_ts[smooth_window - 1] > 0) {
		if (curr_time - bag2_approx_ts(0) > 0.0) {
			vs[0] = (curr_vel - bag2_approx_lens(0)) / (curr_time - bag2_approx_ts(0));
			v += vs[0];
			for (unsigned int i = 1; i < smooth_window; i++) {
				vs[i] = (bag2_approx_lens(i - 1) - bag2_approx_lens(i)) / (bag2_approx_ts(i - 1) - bag2_approx_ts(i));
				v += vs[i];
			}

			v /= (double)smooth_window;

			for (unsigned int i = smooth_window - 1; i > 0; i--) {
				bag2_approx_ts[i] = bag2_approx_ts[i - 1];
				bag2_approx_lens[i] = bag2_approx_lens[i - 1];
			}
			bag2_approx_ts[0] = curr_time;
			bag2_approx_lens[0] = curr_vel;
		}
		else {
			v = 0.0;
		}
	}
	else {
		if (curr_time - bag2_approx_ts(0) > 0.0) {
			vs[0] = (curr_vel - bag2_approx_lens(0)) / (curr_time - bag2_approx_ts(0));

			for (unsigned int i = smooth_window - 1; i > 0; i--) {
				bag2_approx_ts[i] = bag2_approx_ts[i - 1];
				bag2_approx_lens[i] = bag2_approx_lens[i - 1];
			}
			bag2_approx_ts[0] = curr_time;
			bag2_approx_lens[0] = curr_vel;

			v = vs[0];
		}
		else {
			v = 0.0;
		}
	}

	return v;
}

double Mileusnic06Spindle::
smoothChainSecondDeriv(const SimTK::State& s, double curr_val) const
{
	double curr_time = s.getTime();
	double curr_vel = std::isnan(curr_val) ? 0.0 : curr_val;
	const unsigned int smooth_window = SPINDLE_SMOOTHING_WINDOW;
	double v, vs[smooth_window];
	v = 0.0;
	for (unsigned int i = 0; i < smooth_window; i++)
		vs[i] = 0.0;

	if (chain_approx_ts[smooth_window - 2] - chain_approx_ts[smooth_window - 1] > 0) {
		if (curr_time - chain_approx_ts(0) > 0.0) {
			vs[0] = (curr_vel - chain_approx_lens(0)) / (curr_time - chain_approx_ts(0));
			v += vs[0];
			for (unsigned int i = 1; i < smooth_window; i++) {
				vs[i] = (chain_approx_lens(i - 1) - chain_approx_lens(i)) / (chain_approx_ts(i - 1) - chain_approx_ts(i));
				v += vs[i];
			}

			v /= (double)smooth_window;

			for (unsigned int i = smooth_window - 1; i > 0; i--) {
				chain_approx_ts[i] = chain_approx_ts[i - 1];
				chain_approx_lens[i] = chain_approx_lens[i - 1];
			}
			chain_approx_ts[0] = curr_time;
			chain_approx_lens[0] = curr_vel;
		}
		else {
			v = 0.0;
		}
	}
	else {
		if (curr_time - chain_approx_ts(0) > 0.0) {
			vs[0] = (curr_vel - chain_approx_lens(0)) / (curr_time - chain_approx_ts(0));

			for (unsigned int i = smooth_window - 1; i > 0; i--) {
				chain_approx_ts[i] = chain_approx_ts[i - 1];
				chain_approx_lens[i] = chain_approx_lens[i - 1];
			}
			chain_approx_ts[0] = curr_time;
			chain_approx_lens[0] = curr_vel;

			v = vs[0];
		}
		else {
			v = curr_vel;
		}
	}

	return v;
}


//--------------------------------------------------------------------------
// Method to set the initial conditions
//--------------------------------------------------------------------------
void Mileusnic06Spindle::
computeInitialSpindleEquilibrium(SimTK::State& s) const
{	
	// Calculating damping terms (beta)
	double beta_bag1, beta_bag2, beta_chain;
	// dynamic damping term. Equation 4 with beta_2 = 0.
	beta_bag1 = bag1.beta_0 + bag1.beta_1* musclePtr->getDynamicActivation(s);
	// static bag damping term. Equation 4 with beta_1 = 0.
	beta_bag2 = bag2.beta_0 + bag2.beta_2* musclePtr->getStaticActivation(s);
	// chain damping term. Equation 4 with beta_1 = 0.
	beta_chain = chain.beta_0 + chain.beta_2* musclePtr->getStaticActivation(s);
	
	// calculating the force generator terms (Gamma)
	double Gamma_bag1, Gamma_bag2, Gamma_chain;
	// bag1 force generator term. Equation 5 with Gamma_2 = 0
	Gamma_bag1 = bag1.Gamma_1* musclePtr->getDynamicActivation(s);
	// bag2 force generator term. Equation 5 with Gamma_1 = 0
	Gamma_bag2 = bag2.Gamma_2* musclePtr->getStaticActivation(s);
	// chain force generator term. Equation 5 with Gamma_1 = 0
	Gamma_chain = chain.Gamma_2* musclePtr->getStaticActivation(s);
	
	// normalized fiber length and velocity
	double L0 = musclePtr->getOptimalFiberLength();
	double L = musclePtr->getNormalizedFiberLength(s);
	double Lp = (musclePtr->getLPFvelocity(s))/L0;
	// clipping away large velocities 
	Lp = (Lp>15.0)? 15.0 : (Lp<-15.0)? -15.0 : Lp;
	
	//*******************************************************
	// Iteratively calculate LPR, dLPR, and tension
	double LPRb1, LPRb2, LPRc;
	double Tb1, Tb2, Tc, dTb1, dTb2, dTc;
	// Initial estimate of dLPR
	double dLPRb1, dLPRb2, dLPRc;
	dLPRb1 = bag1.K_SR * Lp / (bag1.K_SR + bag1.K_PR);
	dLPRb2 = bag2.K_SR * Lp / (bag2.K_SR + bag2.K_PR);
	dLPRc = chain.K_SR * Lp / (chain.K_SR + chain.K_PR);
	
	// calculate some terms that remain constant through
	// the iterations
	double Cb1, Cb2, Cc;
	Cb1 = (Lp>0.0) ? bag1.C_L : bag1.C_S;
	Cb2 = (Lp>0.0) ? bag2.C_L : bag2.C_S;
	Cc	= (Lp>0.0) ? chain.C_L : chain.C_S;
	
	double num_b1, num_b2, num_c;
	double den_b1, den_b2, den_c;
	num_b1 = bag1.K_SR*(L-bag1.L_0SR) + bag1.K_PR*bag1.L_0PR + Gamma_bag1;
	num_b2 = bag2.K_SR*(L-bag2.L_0SR) + bag2.K_PR*bag2.L_0PR + Gamma_bag2;
	num_c = chain.K_SR*(L-chain.L_0SR) + chain.K_PR*chain.L_0PR + Gamma_chain;
	den_b1 = bag1.K_SR + bag1.K_PR;
	den_b2 = bag2.K_SR + bag2.K_PR;
	den_c = chain.K_SR + chain.K_PR;
	
	double sig = (double)sgn(Lp);
	double rab1 = 1.0/bag1.a;
	double rab2 = 1.0/bag2.a;
	double rac = 1.0/chain.a;
	
	double work_pow; // working variable
	
	// iterations
	for( int i=0; i <= 4; i++)
	{
		// bag 1
		work_pow = beta_bag1*Cb1*sig*std::pow(std::abs(dLPRb1),bag1.a);
		LPRb1 = (num_b1 + work_pow) / (den_b1 + work_pow);
		Tb1 = bag1.K_SR*(L - LPRb1 - bag1.L_0SR);
		dLPRb1 = (Tb1 - bag1.K_PR*(LPRb1 - bag1.L_0PR) + Gamma_bag1)
		     / (beta_bag1*Cb1*(LPRb1 - bag1.R));
		dLPRb1 = sig*std::pow(std::abs(dLPRb1),rab1);
		
		// bag 2
		work_pow = beta_bag2*Cb2*sig*std::pow(std::abs(dLPRb2),bag2.a);
		LPRb2 = (num_b2 + work_pow) / (den_b2 + work_pow);
		Tb2 = bag2.K_SR*(L - LPRb2 - bag2.L_0SR);
		dLPRb2 = (Tb2 - bag2.K_PR*(LPRb2 - bag2.L_0PR) + Gamma_bag2)
		     / (beta_bag2*Cb2*(LPRb2 - bag2.R));
		dLPRb2 = sig*std::pow(std::abs(dLPRb2),rab2);
	
		// chain	
		work_pow = beta_chain*Cc*sig*std::pow(std::abs(dLPRc),chain.a);
		LPRc = (num_c + work_pow) / (den_c + work_pow);
		Tc = chain.K_SR*(L - LPRc - chain.L_0SR);
		dLPRc = (Tc - chain.K_PR*(LPRc - chain.L_0PR) + Gamma_chain)
		     / (beta_chain*Cc*(LPRc - chain.R));
		dLPRc = sig*std::pow(std::abs(dLPRc),rac);	
	}
	// TODO: check convergence
	
	// tension derivatives 
	dTb1 = bag1.K_SR * (Lp - dLPRb1);
	dTb2 = bag2.K_SR * (Lp - dLPRb2);
	dTc = chain.K_SR * (Lp - dLPRc);

	for (unsigned int i = 0; i < SPINDLE_SMOOTHING_WINDOW; i++) {
		bag1_approx_lens[i] = dTb1;
		bag1_approx_ts[i] = 0.0;
		bag2_approx_lens[i] = dTb2;
		bag2_approx_ts[i] = 0.0;
		chain_approx_lens[i] = dTc;
		chain_approx_ts[i] = 0.0;
	}
	
	// set values
	musclePtr->setTensionBag1(s, Tb1);
	musclePtr->setTensionBag1Deriv(s, dTb1);
	musclePtr->setTensionBag2(s, Tb2);
	musclePtr->setTensionBag2Deriv(s, dTb2);
	musclePtr->setTensionChain(s, Tc);
	musclePtr->setTensionChainDeriv(s, dTc);
}
