#ifndef PHEROMONEWAYPOINT_H_
#define PHEROMONEWAYPOINT_H_

/* Make use of ARGoS cross platform representation of primitive data types. */
#include <argos3/core/utility/datatypes/datatypes.h>
/* ARGoS implementation of 2D vector and Cartesian coordinates. */
#include <argos3/core/utility/math/vector2.h>

using namespace argos;

/* This class represents a Cartesian coordinate that is marked by an iAnt robot as a prime search
 * location to search for food. In accordance with the iAnt CPFA model, a given pheromone waypoint
 * will fade over time at an exponential rate of decay. */
class PheromoneWaypoint {

private:

	CVector2 location;        // Cartesian coordinate on the arena map for the waypoint's location
	size_t   lastUpdate;      // the last tick (or simulation frame) when waypoint was updated
	Real     decayRate;       // rate of exponential decay for waypoint strength over time
	Real     weight;          // waypoint's strength, by default set to 1.0 and exponentially decayed
	Real     weightThreshold; // strength threshold to consider waypoint "inactive" or faded
	bool     nullStatus;      // internal flag to indicate if this waypoint is "NULL"

	/* Private helper function. Exponentially decay "weight" over "time" (in simulation ticks). */
	Real exponentialDecay(Real time);

public:

	/* Constructor/Destructor functions. */
	PheromoneWaypoint();
	PheromoneWaypoint(CVector2 location, size_t time, Real decayRate = 0.01, Real weight = 1.0, Real weightThreshold = 0.001);
	~PheromoneWaypoint() {}

	/* Public getter and setter functions. */
	bool update(size_t time); // update the pheromone's status
	bool isActive();          // return the pheromone's current status
	CVector2 getLocation();   // return the pheromone's position

	/* Operator overload functions. */
	PheromoneWaypoint& operator=(const PheromoneWaypoint& rhs);

};

#endif /* PHEROMONEWAYPOINT_H_ */