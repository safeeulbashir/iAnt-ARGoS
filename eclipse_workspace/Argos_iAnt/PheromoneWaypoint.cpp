#include "PheromoneWaypoint.h"

/* Perform exponential decay on "weight" at a rate of "decayRate" over
 * a period of simulation frames equivalent to the "time" variable. */
Real PheromoneWaypoint::exponentialDecay(Real time) {
	return (this->weight * exp(-this->decayRate * time));
}

/* Construct a NULL pheromone waypoint. */
PheromoneWaypoint::PheromoneWaypoint() {
	this->nullStatus = true;
	this->location = CVector2();
	this->lastUpdate = 0;
	this->decayRate = 0.0;
	this->weight = 0.0;
	this->weightThreshold = 0.0;
}

/* Construct a new pheromone waypoint. The location and time created are required. Time is measured
 * in simulation frames where a certain number of frames equals 1.0 seconds (this is set in the .XML
 * configuration file under "ticks_per_second"). The decayRate and weight have default values of 0.01
 * and 1.0 respectively. decayRate should be defined in the .XML and passed to this constructor.
 * weightThreshold shouldn't need to be set differently than its default, but the option is included. */
PheromoneWaypoint::PheromoneWaypoint(CVector2 location, size_t time, Real decayRate, Real weight, Real weightThreshold) {
	this->nullStatus = false;
	this->location = location;
	this->lastUpdate = time;
	this->decayRate = decayRate;
	this->weight = weight;
	this->weightThreshold = weightThreshold;
}

/* Update the pheromone and return its status. Return "true" if the pheromone is still viable
 * and return "false" if the pheromone has faded below the threshold or is a NULL pheromone. */
bool PheromoneWaypoint::update(size_t time) {
	/* The waypoint is NULL. */
	if(nullStatus) {
		return false;
	}

	/* The waypoint is not NULL, but it may be deactivated after exponentialDecay() */
	else {
		/* Exponentially decay the weight value. */
		this->weight = exponentialDecay(time - this->lastUpdate);
		/* Reset the time (or current frame tick) for this update. */
		this->lastUpdate = time;

		/* Return the status of the waypoint after the exponential decay. */
		return isActive();
	}
}

/* Return "true" if the pheromone is still viable and return "false" if
 * the pheromone has faded below the threshold or is a NULL pheromone. */
bool PheromoneWaypoint::isActive() {
	/* The waypoint is NULL. */
	if(nullStatus) {
		return false;
	}

	/* If the waypoint's weight is below the assigned threshold, it is no longer active. */
	else {
		return (this->weight < this->weightThreshold) ? (false) : (true);
	}
}

/* Public getter function. Return the pheromone's location. */
CVector2 PheromoneWaypoint::getLocation() {
	return this->location;
}

/* Overload the C++ equals operator to allow direct assignment of PheromoneWaypoint objects. */
PheromoneWaypoint& PheromoneWaypoint::operator=(const PheromoneWaypoint& rhs) {
	if(this != &rhs) {
		this->location = rhs.location;
		this->lastUpdate = rhs.lastUpdate;
		this->decayRate = rhs.decayRate;
		this->weight = rhs.weight;
		this->weightThreshold = rhs.weightThreshold;
		this->nullStatus = rhs.nullStatus;
	}

	return *this;
}