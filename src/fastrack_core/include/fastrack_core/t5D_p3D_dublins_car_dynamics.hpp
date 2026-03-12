/**
 * Dublin's car dynamics modeled by a 5D tracking model and 3D planning model
 * taken from the running example of https://arxiv.org/pdf/2102.07039.pdf
 */

#ifndef FASTRACK_CORE_T5D_P3D_DUBLINS_CAR_DYNAMICS_HPP
#define FASTRACK_CORE_T5D_P3D_DUBLINS_CAR_DYNAMICS_HPP

#include "fastrack_core/dynamics.hpp"
#include "fastrack_core/relative_dynamics.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace fastrack_core
{

class DublinsCar5DTracking final : public TrackingDynamics
{
  public:
  struct Parameters
  {
    // state bounds (inferred; not specified in the running example)
    double max_turn_rate = 1.0;
    double max_speed = 3.0;
    double min_speed = 0.0;

    // control bounds
    double min_acceleration = -0.5;
    double max_acceleration = 0.5;
    double max_angular_acceleration = 6.0;

    // disturbance bounds
    double max_d_x = 0.02;
    double max_d_y = 0.02;
    double max_d_alpha = 0.02;
    double max_d_a = 0.2;

    // timestep size
    double dt = 0.05;
  };

  DublinsCar5DTracking(Parameters const& params);

  /**
   * @brief Convert 5D dublin's car kinematics into dynamics.
   *
   * @param state StateVector Vector of the car's state: [x, y, theta, velocity, turn rate]
   * @param control ControlVector Vector of the car's controls: [acceleration, angular acceleration]
   * @param disturbance DisturbanceVector of the car: [x, y, acceleration, angular acceleration]
   * @return StateVector Tracking model dynamics
   */
  StateVector dynamics(StateVector const& state,
    ControlVector const& control,
    DisturbanceVector const& disturbance) const override;

  size_t getStateDimension() const override
  {
    return 5;
  }

  size_t getControlDimension() const override
  {
    return 2;
  }

  double getTimestep() const override
  {
    return dt;
  }

  /**
   * @brief Lower bounds of the state
   *
   * @return a vector of min_x, min_y, min_theta, min_velocity, min_turn_rate
   */
  std::vector<double> getStateLowerBounds() const override
  {
    return {-100, -100, -M_PI, params_.min_speed, -params_.max_turn_rate};
  };

  /**
   * @brief Upper bounds of the state
   *
   * @return a vector of max_x, max_y, max_theta, max_velocity, max_turn_rate
   */
  std::vector<double> getStateUpperBounds() const override
  {
    return {100, 100, M_PI, params_.max_speed, params_.max_turn_rate};
  };

  /**
   * @brief Lower bounds of the control
   *
   * @return a vector of min_acceleration, min_angular_acceleration
   */
  std::vector<double> getControlLowerBounds() const override
  {
    return {-params_.max_acceleration, -params_.max_angular_acceleration};
  };

  /**
   * @brief Upper bounds of the control
   *
   * @return a vector of max_acceleration, max_angular_acceleration
   */
  std::vector<double> getControlUpperBounds() const override
  {
    return {params_.max_acceleration, params_.max_angular_acceleration};
  };

  private:
  Parameters params_;
  double dt;
};

class DublinsCar3DPlanning final : public PlanningDynamics
{
  public:
  struct Parameters
  {
    double max_turn_rate = 1.0;
    double max_speed = 3.0;
    double min_speed = 0.0;
    double dt = 0.1;  // timestep size
  };

  DublinsCar3DPlanning(Parameters const& params);

  /**
   * @brief Convert 3D dublin's car kinematics into dynamics.
   *
   * @param state StateVector Vector of the car's state: [x, y, theta]
   * @param control ControlVector Vector of the car's controls: [velocity, turn_rate]
   * @return StateVector Tracking model dynamics
   */
  StateVector dynamics(StateVector const& state,
    ControlVector const& control,
    DisturbanceVector const& disturbance) const override;

  size_t getStateDimension() const override
  {
    return 3;
  }

  size_t getControlDimension() const override
  {
    return 2;
  }

  double getTimestep() const override
  {
    return dt;
  }

  /**
   * @brief Lower bounds of the state
   *
   * @return a vector of min_x, min_y, min_theta
   */
  std::vector<double> getStateLowerBounds() const override
  {
    return {-100, -100, -M_PI};
  };

  /**
   * @brief Upper bounds of the state
   *
   * @return a vector of max_x, max_y, max_theta
   */
  std::vector<double> getStateUpperBounds() const override
  {
    return {100, 100, M_PI};
  };

  /**
   * @brief Lower bounds of the control
   *
   * @return a vector of min_speed, min_turn_rate
   */
  std::vector<double> getControlLowerBounds() const override
  {
    return {params_.min_speed, -params_.max_turn_rate};
  };

  /**
   * @brief Upper bounds of the control
   *
   * @return a vector of max_speed, max_turn_rate
   */
  std::vector<double> getControlUpperBounds() const override
  {
    return {params_.max_speed, params_.max_turn_rate};
  };

  private:
  Parameters params_;
  double dt;
};

class DublinsCarRelativeDynamics final : public RelativeDynamics
{
  private:
  StateVector dynamics_impl(StateVector const& relative_state,
    StateVector const& tracking_state,
    ControlVector const& tracking_control,
    StateVector const& planning_state,
    ControlVector const& planning_control,
    DisturbanceVector const& disturbance) const override;
};

}  // namespace fastrack_core

#endif
