/**
 * Base class for computing relative dynamics
 */

#ifndef FASTRACK_CORE_RELATIVE_DYNAMICS_HPP
#define FASTRACK_CORE_RELATIVE_DYNAMICS_HPP

#include "fastrack_core/dynamics.hpp"
#include <Eigen/Dense>
#include <cassert>
#include <functional>
#include <memory>
#include <stdexcept>
#include <vector>

namespace fastrack_core
{

class RelativeDynamics
{
  public:
  StateVector dynamics(Eigen::MatrixXd const& Q,
    Eigen::MatrixXd const& phi,
    StateVector const& tracking_state,
    ControlVector const& tracking_control,
    StateVector const& planning_state,
    ControlVector const& planning_control,
    DisturbanceVector const& disturbance) const
  {
    validateDimensions(Q, phi, tracking_state.size(), planning_state.size());
    fastrack_core::StateVector relative_state = phi * (tracking_state - Q * planning_state);
    return dynamics_impl(relative_state,
      tracking_state,
      tracking_control,
      planning_state,
      planning_control,
      disturbance);
  }

  private:
  void validateDimensions(
    Eigen::MatrixXd const& Q, Eigen::MatrixXd const& phi, Eigen::Index nt, Eigen::Index np) const
  {
    std::ostringstream msg;

    // check Q to have the dimensions nt x np
    if (Q.rows() != nt || Q.cols() != np) {
      msg << "tracking/planning state dimension mismatch with Q: "
          << "expected (" << Q.rows() << " x " << Q.cols() << "), "
          << "actually (" << nt << " x " << np << ")";
      throw std::invalid_argument(msg.str());
    }

    // check phi to have the right dimensions
    if (phi.rows() != nt || phi.cols() != nt) {
      msg << "tracking/planning state dimension mismatch with phi: "
          << "expected (" << phi.rows() << " x " << phi.cols() << "), "
          << "actually (" << nt << " x " << nt << ")";
      throw std::invalid_argument(msg.str());
    }
  }

  virtual StateVector dynamics_impl(StateVector const& relative_state,
    StateVector const& tracking_state,
    ControlVector const& tracking_control,
    StateVector const& planning_state,
    ControlVector const& planning_control,
    DisturbanceVector const& disturbance) const = 0;
};

}  // namespace fastrack_core

#endif
