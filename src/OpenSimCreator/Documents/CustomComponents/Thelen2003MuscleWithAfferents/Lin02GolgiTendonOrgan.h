#ifndef OPENSIM_LIN02GOLGITENDONORGAN_H_
#define OPENSIM_LIN02GOLGITENDONORGAN_H_

#include "OpenSim/Simulation/Model/ModelComponent.h"
#include "OpenSim/Simulation/Model/Model.h"
#include <OpenSim/Simulation/Model/Muscle.h>

namespace OpenSim {
	
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
class Lin02GolgiTendonOrgan{
friend class Thelen2003MuscleWithAfferents;

public:

	Lin02GolgiTendonOrgan();  // the constructor
	
protected:

	static const std::string STATE_LPF_OUTPUT_NAME;
	static const std::string STATE_LPF_DERIV_NAME;
	static const std::string STATE_FILTER_INTER_NAME;
	static const std::string STATE_FILTER_OUTPUT_NAME;

	static const std::string CACHE_GTO_OUT_NAME;

	//--------------------------------------------------------------------------
	// MODEL COMPONENT INTERFACE
	//--------------------------------------------------------------------------
    /// Currently you need to call all these from the owner muscle.
	
	/** initialize state variables from properties */
	void extendInitStateFromProperties(SimTK::State& s) const;

	void setParentMuscle(const Thelen2003MuscleWithAfferents* parent) { musclePtr = parent; }
	
	//-------------------------------------------------------------------------
	// PARAMETERS
	//-------------------------------------------------------------------------	
	// Values used in Eq. 1 (initialized in the constuctor) 
	double Gg, Gf; // pulses/s and Newtons, respectively
	// Threshold for Eq. 3
	double thr;  // pulses / s
	
	/** Set an inital value for the output of the nonlinearity,
	  * and set the other variables to zero */
	void initFromMuscle(SimTK::State& s) const;
	
	//-------------------------------------------------------------------------
	// COMPUTATIONS
	//-------------------------------------------------------------------------
	SimTK::Vector computeStateVariableDerivatives(const SimTK::State& s) const;
	
	/** This method calculates the first and second derivatives of 
	 *  the log nonlinearity. 
	 *  It uses a version of the method in: Fornberg 1998 "Calculation of 
	 *  Weights in Finite Difference Formulas" SIAM Rev.40(3):685-691,
	 *  as well as simpler rules.
	 *  The stored data points to apply the method are in the vectors
	 *  ts, vel, F, and dF .  */
	SimTK::Vec<2> calculateDerivatives(const SimTK::State& s) const;
	
	
	
private:
	
	/** This pointer leads to the the owner muscle. 
	 *  Initialized in Lin02GolgiTendonOrgan::connectToModel */
	const Thelen2003MuscleWithAfferents *musclePtr;
	
	/** These auxiliary vectors are used to approximate the 
	 *  derivatives of the log nonlinearity. 
	 *  'nl' and 'Dnl' are the stored values of the output of the
	 *  nonlinearity (Eq. 1) and its derivative, respectively. 
	 *  'ts' contains the time when they were stored.
	 *  Since computeStateVariableDerivatives is const, mutable is needed. */
	mutable SimTK::Vec<5> ts, nl, Dnl;

#define GTO_SMOOTHING_WINDOW 10
	mutable SimTK::Vec<GTO_SMOOTHING_WINDOW> nl_approx_xs, dnl_approx_nls, nl_approx_ts;

}; // end of class Lin02GolgiTendonOrgan
	
} // end of namespace OpenSim

#endif