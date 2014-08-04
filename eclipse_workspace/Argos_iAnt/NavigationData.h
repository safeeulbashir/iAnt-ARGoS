#ifndef NAVIGATIONDATA_H_
#define NAVIGATIONDATA_H_

/* Access XML loading functionality. */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* ARGoS 2D vector class. */
#include <argos3/core/utility/math/vector2.h>
/* ARGOS angle definition class. */
#include <argos3/core/utility/math/angles.h>

/* Argos3 objects for robot components: actuators and sensors. (plug-ins) */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>

using namespace argos;

class NavigationData {

private:

	/* robot actuator and sensor components */
	CCI_DifferentialSteeringActuator *steeringActuator; // controls the robot's motor speeds
	CCI_FootBotProximitySensor       *proximitySensor;  // detects nearby objects to prevent collision
	CCI_FootBotMotorGroundSensor     *groundSensor;     // detects food items & nest (color changes)
	CCI_FootBotLightSensor           *lightSensor;      // detects nest-light for navigation control

	Real             distanceTolerance;   // distance to trigger collision detection
    CRange<CRadians> angleTolerance;      // angle tolerance range to go straight
    CVector2         arenaSize;           // rectangular grid size "GetX()" by "GetY()"
	CRange<Real>     forageRangeX;        // Cartesian X domain of arena [-x, x]
	CRange<Real>     forageRangeY;        // Cartesian Y range of arena [-y, y]
    CVector2         position;            // robot's current position in the arena
    CVector2         target;              // robot's current target in the arena
    CVector2         nestPosition;        // the position of the center of the nest
    Real             nestRadiusSquared;   // the nest area radius
    Real             searchStepSize;      // vector length for each search "step"
    Real             searchRadiusSquared; // food density search radius around robot
	Real             maxSpeed;            // maximum motor speed, configured in XML

public:

	NavigationData();
	~NavigationData();

	void Init(TConfigurationNode& node) {}
	void Reset() {}

	void     SetSensorsAndActuators(CCI_DifferentialSteeringActuator *steeringActuator,
			                        CCI_FootBotProximitySensor       *proximitySensor,
			                    	CCI_FootBotMotorGroundSensor     *groundSensor,
			                    	CCI_FootBotLightSensor           *lightSensor);
	bool     CollisionDetection();
	CRadians LawOfCosines(CVector2& A, CVector2& B, CVector2& C);
	Real     GetSignOfRotationAngle(CVector2& A, CVector2& B, CVector2& C);
	CVector2 GetVectorToLight();
	CVector2 GetVectorToPosition(const CVector2& targetPosition);
	void     SetWheelSpeed(const CVector2& heading);

};

#endif /* NAVIGATIONDATA_H_ */