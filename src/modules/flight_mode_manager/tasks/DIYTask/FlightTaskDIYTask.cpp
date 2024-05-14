#include "FlightTaskDIYTask.hpp"
#include <vector>

bool FlightTaskDIYTask::activate(const trajectory_setpoint_s &last_setpoint)
{
    bool ret = FlightTask::activate(last_setpoint);

    _position_setpoint(0) = _position(0);
    _position_setpoint(1) = _position(1);
    _position_setpoint(2) = _position(2);

    _target_position(0) = _position(0) + 10.0f;
    _target_position(1) = _position(1) + 10.0f;
    _target_position(2) = _position(2);

    _max_velocity = 1.0f;
    _max_acceleration = 0.5f;

    return ret;
}

void FlightTaskDIYTask::updateParams()
{
	// Copy previous param values to check if it changes after param update
	const float radius_prev = _param_mpc_radius.get();
    const float tolerance = 0.0001f;

	// Call updateParams in parent class to update parameters from child up until ModuleParam base class
	FlightTask::updateParams();

	// Compare param values and if they have changed, update the properties
	if (fabs(radius_prev - _param_mpc_radius.get()) > tolerance) {
		radius = _param_mpc_radius.get();
	}
}

bool FlightTaskDIYTask::update()
{
    switch (_phase) {
	case Phase::MoveToPosition:
	if (_arrivedAtPosition()) {
		_phase = Phase::CircleAroundPosition;
	} else {
		_moveToPosition();
	}
	break;

	case Phase::CircleAroundPosition:
	_circleAroundPosition();
	break;
    }

    return true;
}

bool FlightTaskDIYTask::_arrivedAtPosition(){
    if (fabsf(_position(0) - _target_position(0)) > 1.0f && fabsf(_position(1) - _target_position(1)) > 1.0f){
        return false;
    } else {
	    return true;
    }
}
void FlightTaskDIYTask::_moveToPosition(){
    if (PX4_ISFINITE(_position(0)) && PX4_ISFINITE(_position(1)) && PX4_ISFINITE(_position(2))) {
        _position_setpoint(0) = _target_position(0);
        _position_setpoint(1) = _target_position(1);
        _position_setpoint(2) = _target_position(2);
        PX4_INFO("first loop");
    } else {
	    PX4_INFO("Not Finite!");
    }
}

void FlightTaskDIYTask::_circleAroundPosition()
{

    radius = _param_mpc_radius.get();

    float origin_x = _target_position(0);
    float origin_y = _target_position(1);
    float origin_z = _target_position(2);

    if (fabsf(_position(0) - _position_setpoint(0)) < radius/2.0f &&
        fabsf(_position(1) - _position_setpoint(1)) < radius/2.0f) {

        _position_setpoint(0) = origin_x + radius * static_cast<float>(cos(angle * M_PI / 180.0));
        _position_setpoint(1) = origin_y + radius * static_cast<float>(sin(angle * M_PI / 180.0));
	    _position_setpoint(2) = origin_z;

        angle = fmod(angle + angle_increment, 360.0);
	    char angle_str[20]; // Allocate a buffer for the string representation of angle
        snprintf(angle_str, sizeof(angle_str), "%f", angle); // Convert angle to string
	    PX4_INFO("Angle: %s", angle_str);
    }
}

