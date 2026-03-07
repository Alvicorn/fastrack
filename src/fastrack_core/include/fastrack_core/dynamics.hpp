#ifndef FASTRACK_CORE_DYNAMICS_HPP
#define FASTRACK_CORE_DYNAMICS_HPP

#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <vector>

namespace fastrack_core
{

using StateVector = Eigen::VectorXd;
using ControlVector = Eigen::VectorXd;
using DisturbanceVector = Eigen::VectorXd;

/**
 * @brief Base class for dynamical models
 */
class Dynamics
{
  public:
  virtual ~Dynamics() = default;

  /**
   * @brief Convert kinematics (state & control) to dynamics.
   *
   * @param state StateVector Vector of the car's state
   * @param control ControlVector Vector of the car's controls
   * @param disturbance DisturbanceVector of the car
   * @return StateVector Tracking model dynamics
   */
  virtual StateVector dynamics(
    StateVector const& state, ControlVector const& control, DisturbanceVector const& disturbance

  ) const = 0;
  virtual size_t getStateDimension() const = 0;
  virtual size_t getControlDimension() const = 0;

  virtual std::vector<double> getStateLowerBounds() const = 0;
  virtual std::vector<double> getStateUpperBounds() const = 0;
  virtual std::vector<double> getControlLowerBounds() const = 0;
  virtual std::vector<double> getControlUpperBounds() const = 0;
};

/**
 * @brief Tracking model with higher-order dynamics and disturbances.
 * The tracking model must have a higher fidelity than the planning model.
 */
class TrackingDynamics : public Dynamics
{
  public:
  virtual double getTimestep() const = 0;
};

/**
 * @brief Planning model is a simplified model of the tracking model
 */
class PlanningDynamics : public Dynamics
{
  public:
  virtual double getTimestep() const = 0;

  virtual StateVector toPlanningState(StateVector const& tracking_state) const
  {
    return tracking_state;
  }
};

}  // namespace fastrack_core

#endif
