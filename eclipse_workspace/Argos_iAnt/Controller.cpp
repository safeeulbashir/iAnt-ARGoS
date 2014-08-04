#include "Controller.h"

/* Construct the Controller object and set all default values to NULL.
 * Variables will be initialized by the Init() function. */
Controller::Controller() :
	steeringActuator(NULL),
	proximitySensor(NULL),
	groundSensor(NULL),
	lightSensor(NULL),
	travelProbability(0.0),
	searchProbability(0.0),
	uninformedSearchCorrelation(CRadians::ZERO),
	informedSearchDecay(0.0),
	siteFidelityRate(0.0),
	pheromoneRate(0.0),
	pheromoneDecayRate(0.0),
	RNG(NULL),
	state(SET_SEARCH_LOCATION)
{}

/* Deconstruct the Controller object (deallocate memory, etc).
 * Currently not necessary and included for future expansion. */
Controller::~Controller() {
	// does nothing... for now
}

/* Inherited function of the CCI_Controller class.
 * Set default values for all components and variables from the XML file. */
void Controller::Init(TConfigurationNode& node) {
	/* TODO move the sensors and actuators into the navigationData class!!! */
	/* Initialize ARGoS sensors and actuators from these categories in the XML file. */
	steeringActuator = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
	proximitySensor  = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
	groundSensor     = GetSensor<CCI_FootBotMotorGroundSensor>("footbot_motor_ground");
	lightSensor      = GetSensor<CCI_FootBotLightSensor>("footbot_light");

	navData.SetSensorsAndActuators(steeringActuator, proximitySensor,
			                       groundSensor, lightSensor);

	/* Initialize default iAnt object values from the <params> tag located inside of
	 * the <controllers> tag inside of the Argos3 XML file. */
	navData.Init(GetNode(node, "navigation"));
	foodData.Init(GetNode(node, "food"));
}

/* Inherited function of the CCI_Controller class.
 * Perform control logic for a single step (frame) of simulation time. */
void Controller::ControlStep() {

	/* Check for collisions and move out of the way before running the state machine. */
	if(!navData.CollisionDetection()) {

		/* Perform actions based on the modified state machine. */
		switch(state) {

			/* The robot will select a location to search for food. */
			case SET_SEARCH_LOCATION:
				SetSearchLocation();
				break;
			/* The robot will travel to the location it has selected to search from. */
			case TRAVEL_TO_SEARCH_SITE:
				TravelToSearchSite();
				break;
			/* The robot will perform an informed walk while searching for food. */
			case PERFORM_INFORMED_WALK:
				PerformInformedWalk();
				break;
			/* The robot will perform an uninformed walk while searching for food. */
			case PERFORM_UNINFORMED_WALK:
				PerformUninformedWalk();
				break;
			/* The robot has found food and is checking the local resource density. */
			case SENSE_LOCAL_RESOURCE_DENSITY:
				SenseLocalResourceDensity();
				break;
			/* The robot is traveling to the nest after finding food or giving up a search. */
			case TRAVEL_TO_NEST:
				TravelToNest();
				break;
			/* We should never be here! Something TERRIBLE has occurred! */
			default:
				LOGERR << "ERROR: void iAnt_controller::ControlStep()" << std::endl
					   << "STATE: [" << state << "] not found!"<< std::endl;

		} /* end switch */

	} /* end if */

}

/* Inherited function of the CCI_Controller class.
 * Reset all variables to their default pre-run values.
 * Sensors/Actuators are reset by Argos outside of this class. */
void Controller::Reset() {
	navData.Reset();
	foodData.Reset();
}

/* Inherited function of the CCI_Controller class.
 * Memory deallocation and clean up. Currently not needed and here for future expansion. */
void Controller::Destroy() {
	// do nothing... for now
}

/***
 *
 * Major TODO:
 *
 * Make the CPFA functions here accept the "navigation" and
 * "food" objects as a parameters and have it work on those
 * objects (pass by reference, etc)!
 *
 */
void Controller::SetSearchLocation() {
}

void Controller::TravelToSearchSite() {
}

void Controller::PerformInformedWalk() {
}

void Controller::PerformUninformedWalk() {
}

void Controller::SenseLocalResourceDensity() {
}

void Controller::TravelToNest() {
}