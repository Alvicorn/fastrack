/**
 * Dublin's car dynamics modeled by a 5D tracking model and 3D planning model
 * taken from the running example of https://arxiv.org/pdf/2102.07039.pdf
 */

#include "fastrack_core/t5D_p3D_dublins_car_dynamics.hpp"

namespace fastrack_core
{

DublinsCar5DTracking::DublinsCar5DTracking(Parameters const& params)
  : params_(params)
{
  dt = params_.dt;
}

StateVector DublinsCar5DTracking::dynamics(StateVector const& state,
  ControlVector const& control,
  DisturbanceVector const& disturbance) const
{
  StateVector dot(5);

  double theta = state(2);
  double v = state(3);
  double omega = state(4);

  double a = control(0);
  double alpha = control(1);

  double d_x = disturbance(0);
  double d_y = disturbance(1);
  double d_a = disturbance(2);
  double d_alpha = disturbance(3);

  dot[0] = (v * std::cos(theta)) + d_x;
  dot[1] = (v * std::sin(theta)) + d_y;
  dot[2] = omega;
  dot[3] = a + d_a;
  dot[4] = alpha + d_alpha;

  return dot;
}

DublinsCar3DPlanning::DublinsCar3DPlanning(Parameters const& params)
  : params_(params)
{
  dt = params_.dt;
}

StateVector DublinsCar3DPlanning::dynamics(StateVector const& state,
  ControlVector const& control,
  DisturbanceVector const& disturbance) const
{
  (void)disturbance;

  StateVector dot(3);

  double theta = state(2);

  double v = control(0);
  double omega = control(1);

  dot[0] = v * std::cos(theta);
  dot[1] = v * std::sin(theta);
  dot[2] = omega;

  return dot;
}

StateVector DublinsCarRelativeDynamics::dynamics_impl(StateVector const& relative_state,
  StateVector const& tracking_state,
  ControlVector const& tracking_control,
  StateVector const& planning_state,
  ControlVector const& planning_control,
  DisturbanceVector const& disturbance) const
{
  (void)planning_state;
  StateVector dot(5);

  double x_r = relative_state(0);
  double y_r = relative_state(1);
  double theta_r = relative_state(2);

  double v_t = tracking_state(3);
  double omega_t = tracking_state(4);

  double a = tracking_control(0);
  double alpha = tracking_control(1);

  double v_p = planning_control(0);
  double omega_p = planning_control(1);

  double d_x = disturbance(0);
  double d_y = disturbance(1);
  double d_a = disturbance(2);
  double d_alpha = disturbance(3);

  dot[0] = -v_p + (v_t * std::cos(theta_r)) + (omega_p * y_r) + d_x;
  dot[1] = (v_t * std::sin(theta_r)) - (omega_p * x_r) + d_y;
  dot[2] = omega_t - omega_p;
  dot[3] = a + d_a;
  dot[4] = alpha + d_alpha;

  return dot;
}

}  // namespace fastrack_core
