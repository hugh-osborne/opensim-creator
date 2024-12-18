
// INCLUDE
#include <OpenSim/OpenSim.h>
#include "Lin02GolgiTendonOrgan.h"
#include "Mileusnic06Spindle.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * This class extends a Thelen2003Muscle by including including
 * inputs and outputs to represent spindle afferents and Golgi tendon organs. 
 * The spindle afferent model comes from: Mileusnic et al 2006 "Mathematical 
 * models of proprioceptors. I. Control and transduction in the muscle spindle"
 * J Neurophysiol 96:1772-1788.
 * The GTO model is the one used in: Lin, Crago 2002 "Neural and mechanical 
 * contributions to the stretch reflex: A model synthesis" 
 * Ann Biomed Eng 30:54-67.
 *
 * The implementation of this class was achieved by modifying the file
 * FatigableMuscle.h, which is part of the OpenSim API examples.
 *
 * @author Sergio Verduzco Flores (based on Thelen2003Muscle)
 *
 * The Muscle base class specifies the interface that must be implemented 
 * by derived muscle classes. The Thelen2003Muscle derives from
 * Thelen2003Muscle, which is a concrete implementation of the 
 * Muscle interface. The dynamics for spindle afferents are handled by
 * a member object of the Mileusnic06Spindle class. The dynamics of the
 * GTO are handled by an object of the Lin02GolgiTendonOrgan class.
 * 
 * The Mileusnic06Spindle object requires muscle fiber acceleration, which is 
 * not available from the Thelen2003Muscle class, so this has to
 * be approximated numerically. To aid this, this muscle class provides a
 * low-pass filtered version of the fiber velocity, and the acceleration is 
 * provided as a state variable.
 *
 * @see Thelen2003Muscle
 * @see Muscle
 * @see Mileusnic06Spindle
 * @see Lin02GolgiTendonOrgan  
 */
class Thelen2003MuscleWithAfferents : public Thelen2003Muscle {
OpenSim_DECLARE_CONCRETE_OBJECT(Thelen2003MuscleWithAfferents,
	Thelen2003Muscle);
friend class Lin02GolgiTendonOrgan;
friend class Mileusnic06Spindle;

public:
//=============================================================================
// PROPERTIES
//=============================================================================
	OpenSim_DECLARE_PROPERTY(lpf_tau, double, 
        "time constant for all the low-pass filters");

	OpenSim_DECLARE_PROPERTY(lpf_default_activation, double,
		"default value for both static and dynamic activation");

	OpenSim_DECLARE_OUTPUT(lpf_velocity, double,
		getLPFvelocity, SimTK::Stage::Dynamics);

	OpenSim_DECLARE_OUTPUT(lpf_acceleration, double,
		getLPFacceleration, SimTK::Stage::Dynamics);

	OpenSim_DECLARE_OUTPUT(primary_Ia, double,
		getIaOutput, SimTK::Stage::Dynamics);

	OpenSim_DECLARE_OUTPUT(secondary_II, double,
		getIIOutput, SimTK::Stage::Dynamics);

	OpenSim_DECLARE_OUTPUT(gto_out, double,
		getGTOout, SimTK::Stage::Dynamics);

//=============================================================================
// THE AFFERENTS >><< >><< >><< >><< >><< >><< >><< >><< >><< >><< >><< >><<
//=============================================================================
	
	Mileusnic06Spindle spindle;
	Lin02GolgiTendonOrgan GTO;

private:

	/** These auxiliary vectors are used to approximate the
	 *  derivatives of muscle variables.
	 *  'vel' and 'ts' are the velocity and time values used in the method
	 *  to calculate the acceleration.
	 *  Since computeStateVariableDerivatives is const, mutable is needed. */
#define SMOOTHING_WINDOW 10
	mutable SimTK::Vec<SMOOTHING_WINDOW> acc_approx_vels, acc_approx_ts;
	mutable SimTK::Vec<SMOOTHING_WINDOW> vel_approx_lens, vel_approx_ts;

public:
//=============================================================================
// METHODS
//=============================================================================
	/** The class constructors */
	Thelen2003MuscleWithAfferents();
	Thelen2003MuscleWithAfferents(const std::string &name, double maxIsometricForce,
					double optimalFiberLength, double tendonSlackLength,
					double pennationAngle);
	Thelen2003MuscleWithAfferents(const Thelen2003Muscle&);

    // employs the default destructor, copy constructor and copy assignment 
	// that are automatically supplied by the compiler if none are defined
	
	//-------------------------------------------------------------------------
	// GET & SET state variables and their derivatives
	//-------------------------------------------------------------------------
	double getLPFvelocity(const SimTK::State& s) const;
	void setLPFvelocity(SimTK::State& s, double Velocity) const;
	
	double getLPFacceleration(const SimTK::State& s) const;
	void setLPFacceleration(SimTK::State& s, double Acceleration) const;

	double getDynamicActivation(const SimTK::State& s) const;
	void setDynamicActivation(SimTK::State& s, double Activation) const;
	double getStaticActivation(const SimTK::State& s) const;
	void setStaticActivation(SimTK::State& s, double Activation) const;

	double getTensionBag1(const SimTK::State& s) const;
	void setTensionBag1(SimTK::State& s, double Tension) const;
	double getTensionBag1Deriv(const SimTK::State& s) const;
	void setTensionBag1Deriv(SimTK::State& s, double TensionDeriv) const;

	double getTensionBag2(const SimTK::State& s) const;
	void setTensionBag2(SimTK::State& s, double Tension) const;
	double getTensionBag2Deriv(const SimTK::State& s) const;
	void setTensionBag2Deriv(SimTK::State& s, double TensionDeriv) const;

	double getTensionChain(const SimTK::State& s) const;
	void setTensionChain(SimTK::State& s, double Tension) const;
	double getTensionChainDeriv(const SimTK::State& s) const;
	void setTensionChainDeriv(SimTK::State& s, double TensionDeriv) const;

	double getIaOutput(const SimTK::State& s) const;
	double& updIaOutput(const SimTK::State& s) const;
	double getIIOutput(const SimTK::State& s) const;
	double& updIIOutput(const SimTK::State& s) const;

	//-------------------------------------------------------------------------
	// GET & SET state variables and cache variables
	//-------------------------------------------------------------------------
	// low-pass filtered output of the nonlinearity
	double getX(const SimTK::State& s) const;
	void setX(SimTK::State& s, double X) const;
	// derivative of the low-pass filtered output of the nonlinearity
	double getXp(const SimTK::State& s) const;
	void setXp(SimTK::State& s, double Xp) const;
	// Intermediate variable for the transfer function filter
	double getY(const SimTK::State& s) const;
	void setY(SimTK::State& s, double Y) const;
	// output variable of the transfer function
	double getZ(const SimTK::State& s) const;
	void setZ(SimTK::State& s, double Z) const;
	// output of the Golgi tendon organ
	double getGTOout(const SimTK::State& s) const;
	double& updGTOout(const SimTK::State& s) const;
	
	//-------------------------------------------------------------------------
	// GET & SET Properties
	//-------------------------------------------------------------------------
	double getLPFtau() const { return get_lpf_tau(); }
	void setLPFtau(double aLPFtau);

	double getLPFDefaultActivation() const { return get_lpf_default_activation(); }
	void setLPFDefaultActivation(double aLPFDefaultActivation);
	
	//-------------------------------------------------------------------------
	// Other methods
	//-------------------------------------------------------------------------
	/** This function should override Actuator::numControls() */
	int numControls() const {return 3;};


	/** This function allows to peek at the Mileusnic06Spindle object.
	 *  Mostly for the sake of Analysis objects. */
	const Mileusnic06Spindle* getSpindle() { return &spindle; };
	
	/** This one allows to peek at the Lin02GolgiTendonOrgan object. */
	const Lin02GolgiTendonOrgan* getGTO() { return &GTO; };
	
protected:

	static const std::string STATE_LPF_VELOCITY_NAME;
	static const std::string STATE_LPF_ACCELERATION_NAME;
	static const std::string AFF_STATE_ACTIVATION_NAME;
	static const std::string AFF_STATE_FIBER_LENGTH_NAME;

	// Model Component Interface
	/** add new dynamical states to the multibody system corresponding
	    to this muscle */
	void extendAddToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;
	/** initialize muscle state variables from properties. For example, any 
	    properties that contain default state values */
	void extendInitStateFromProperties(SimTK::State& s)const OVERRIDE_11;
	/** set current values of state into the properties */
	void extendSetPropertiesFromState(const SimTK::State& s) OVERRIDE_11;
	/** Extension of the parent class method.
	    Used here to declare the spindle as a subcomponent. */
	void extendConnectToModel(Model& aModel) OVERRIDE_11;

	//-------------------------------------------------------------------------
	// COMPUTATIONS
	//-------------------------------------------------------------------------
	/** This function finds an initial state for the tension, and
	 *  then it calls the homonimous method of the parent class. */
	void computeInitialFiberEquilibrium(SimTK::State& s) const OVERRIDE_11;
	
	/** Compute the derivatives for state variables added by this muscle */
	void computeStateVariableDerivatives(const SimTK::State& s) const OVERRIDE_11;
		
	/** The following function calculates the derivative of the fiber velocity
	 *  using the method in: Fornberg 1998 "Calculation of Weights in Finite
	 *  Difference Formulas" SIAM Rev.40(3):685-691.
	 *  The method is adapted to use with four points and to always evaluate at the
	 *  last point.  */
	double approxFiberAcceleration(const SimTK::State& s) const;
	double approxFiberVelocity(const SimTK::State& s) const;

	double getActivationDerivative(const SimTK::State& s) const;
	

private:

	/** construct the new properties and set their default values */
	void constructProperties();

	
public:
	/** This storage object records the afferent outputs */
	Storage *afferents;
	
//=============================================================================
};	// END of class Thelen2003MuscleWithAfferents
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

