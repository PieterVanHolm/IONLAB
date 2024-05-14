/**
 * @file FlightTaskDIYTask.hpp
 *
 * @author Pieter Van holm <Pieter.Van-Holm@student.isae-supero.fr>
 */

#pragma once

#include "FlightTask.hpp"
#include <px4_platform_common/module_params.h>
#include <parameters/param.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>

#include <vector>

class FlightTaskDIYTask : public FlightTask
{
public:
  FlightTaskDIYTask() = default;
  virtual ~FlightTaskDIYTask() = default;

  bool update() override;
  bool activate(const trajectory_setpoint_s &last_setpoint) override;
  void updateParams() override;

private:

  DEFINE_PARAMETERS_CUSTOM_PARENT(
		FlightTask,
		(ParamFloat<px4::params::MPC_RADIUS>) _param_mpc_radius
	)

  matrix::Vector3f _target_position;
  matrix::Vector3f _next_setpoint;
  matrix::Vector3f _previous_setpoint;
  float _max_velocity;
  float _max_acceleration;

  // Define parameters for the circle
  float radius{5.0f};
  double n = 16;
  double angle_increment = 360 / n;

  // Loop to update position setpoint and create circular motion
  double angle = 0;

  // Other private members for control logic

  // Define any helper functions needed for control logic

  enum class Phase {
        MoveToPosition,
        CircleAroundPosition
    };

  Phase _phase;

  bool _arrivedAtPosition();
  void _moveToPosition();
  void _circleAroundPosition();
};
