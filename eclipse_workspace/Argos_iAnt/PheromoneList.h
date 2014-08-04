#ifndef PHEROMONELIST_H_
#define PHEROMONELIST_H_

/* Implementation for pheromone location object. */
#include "PheromoneWaypoint.h"
/* Make use of ARGoS cross platform representation of primitive data types. */
#include <argos3/core/utility/datatypes/datatypes.h>
/* ARGoS implementation of 2D vector and Cartesian coordinates. */
#include <argos3/core/utility/math/vector2.h>

using namespace argos;
using namespace std;

/* This class maintains a central list of all currently known pheromone waypoints used by iAnt robots to
 * search for food. This list should primarily be maintained within the loop functions class of ARGoS. */
class PheromoneList {

private:

	vector<PheromoneWaypoint> pwList; // list of pheromone waypoints

public:

	PheromoneList() {}  // constructor currently not needed, an "empty" vector upon declaration is desired
	~PheromoneList() {} // destructor currently not needed

	void addPheromoneWaypoint(PheromoneWaypoint pw); // add a pheromone to the list
	PheromoneWaypoint getPheromoneWaypoint();        // get the first available pheromone from the list
	void update(size_t time);                        // update all pheromone's exponential decay

};

#endif /* PHEROMONELIST_H_ */