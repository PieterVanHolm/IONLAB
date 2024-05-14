#include "FlightTaskContinuousYaw.hpp"

bool FlightTaskContinuousYaw::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);

	_position_setpoint(0) = _position(0);
	_position_setpoint(1) = _position(1);

	_origin_z = _position(2);

	_yawspeed_setpoint = 45.0f * 3.142f / 180.0f;
	_velocity_setpoint(2) = -1.0f;

	return ret;
}

bool FlightTaskContinuousYaw::update()
{
	float diff_z = _position(2) - _origin_z;

	if (diff_z <= -0.8f){
		_velocity_setpoint(2) = 1.0f;
		_yawspeed_setpoint = 45.0f * 3.142f / 180.0f * -1.0f;
	} else if (diff_z >= 0.0f){
		_velocity_setpoint(2) = -1.0f;
		_yawspeed_setpoint = 45.0f * 3.142f / 180.0f;
	}
	return true;
}
