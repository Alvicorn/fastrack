#include "dynamics_fixtures.hpp"
#include "fastrack_core/t5D_p3D_dublins_car_dynamics.hpp"
#include <Eigen/Dense>
#include <memory>
#include <stdexcept>

using namespace fastrack_core;
using namespace fastrack_core::testing;

// NOLINTBEGIN(misc-non-private-member-variables-in-classes)
class TestDublinsCarDynamics : public DynamicsTestFixture
{
  protected:
  void SetUp() override
  {
    DynamicsTestFixture::SetUp();

    DublinsCar5DTracking::Parameters params;

    // state bounds (inferred; not specified in the running example)
    params.max_turn_rate = 1.0;
    params.max_speed = 3.0;
    params.min_speed = 0.0;

    // control bounds
    params.min_acceleration = -0.5;
    params.max_acceleration = 0.5;
    params.max_angular_acceleration = 6.0;

    // disturbance bounds
    params.max_d_x = 0.02;
    params.max_d_y = 0.02;
    params.max_d_alpha = 0.02;
    params.max_d_a = 0.2;

    // timestep size
    params.dt = 0.05;

    tracking_dynamics = std::make_shared<DublinsCar5DTracking>(params);
    relative_dynamics = std::make_shared<DublinsCarRelativeDynamics>();

    // set phi
    Q = Eigen::MatrixXd(5, 3);
    Q << 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;

    phi = Eigen::MatrixXd::Zero(5, 5);
  }

  std::shared_ptr<DublinsCar5DTracking> tracking_dynamics;
  std::shared_ptr<DublinsCarRelativeDynamics> relative_dynamics;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd phi;
};

/**
 * @brief Test dimension getters
 */
TEST_F(TestDublinsCarDynamics, TestDimensions)
{
  EXPECT_EQ(tracking_dynamics->getStateDimension(), 5);
  EXPECT_EQ(tracking_dynamics->getControlDimension(), 2);
}

TEST_F(TestDublinsCarDynamics, TestNoControlNoDisturbance)
{
  double x = 1.0;
  double y = 2.0;
  double theta = 0.5;
  double v = 2.0;
  double omega = 0.1;

  StateVector state = (StateVector(5) << x, y, theta, v, omega).finished();
  ControlVector control = Eigen::VectorXd::Zero(2);
  DisturbanceVector disturbance = Eigen::VectorXd::Zero(4);

  auto dynamics = tracking_dynamics->dynamics(state, control, disturbance);

  double expected_dx = v * std::cos(theta);
  double expected_dy = v * std::sin(theta);
  double expected_omega = omega;
  double expected_a = 0.0;
  double expected_alpha = 0.0;

  EXPECT_NEAR(dynamics[0], expected_dx, test_tolerance);
  EXPECT_NEAR(dynamics[1], expected_dy, test_tolerance);
  EXPECT_NEAR(dynamics[2], expected_omega, test_tolerance);
  EXPECT_NEAR(dynamics[3], expected_a, test_tolerance);
  EXPECT_NEAR(dynamics[4], expected_alpha, test_tolerance);
}

TEST_F(TestDublinsCarDynamics, TestNoControlWithDisturbance)
{
  double x = 1.0;
  double y = 2.0;
  double theta = 0.5;
  double v = 2.0;
  double omega = 0.1;

  double d_x = 0.05;
  double d_y = 0.05;
  double d_a = 0.1;
  double d_alpha = 0.01;

  StateVector state = (StateVector(5) << x, y, theta, v, omega).finished();
  ControlVector control = Eigen::VectorXd::Zero(2);
  DisturbanceVector disturbance = (DisturbanceVector(4) << d_x, d_y, d_a, d_alpha).finished();

  auto dynamics = tracking_dynamics->dynamics(state, control, disturbance);

  double expected_dx = (v * std::cos(theta)) + d_x;
  double expected_dy = (v * std::sin(theta)) + d_y;
  double expected_omega = omega;
  double expected_a = 0.0 + d_a;
  double expected_alpha = 0.0 + d_alpha;

  EXPECT_NEAR(dynamics[0], expected_dx, test_tolerance);
  EXPECT_NEAR(dynamics[1], expected_dy, test_tolerance);
  EXPECT_NEAR(dynamics[2], expected_omega, test_tolerance);
  EXPECT_NEAR(dynamics[3], expected_a, test_tolerance);
  EXPECT_NEAR(dynamics[4], expected_alpha, test_tolerance);
}

TEST_F(TestDublinsCarDynamics, TestPositiveAcceleration)
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double v = 1.0;
  double omega = 0.0;

  double a = 1.0;
  double alpha = 0.0;

  StateVector state = (StateVector(5) << x, y, theta, v, omega).finished();
  ControlVector control = (ControlVector(2) << a, alpha).finished();
  DisturbanceVector disturbance = Eigen::VectorXd::Zero(4);

  auto dynamics = tracking_dynamics->dynamics(state, control, disturbance);

  EXPECT_NEAR(dynamics[0], 1.0, test_tolerance);  // v * cos(0) = 1
  EXPECT_NEAR(dynamics[1], 0.0, test_tolerance);
  EXPECT_NEAR(dynamics[2], 0.0, test_tolerance);
  EXPECT_NEAR(dynamics[3], 1.0, test_tolerance);
  EXPECT_NEAR(dynamics[4], 0.0, test_tolerance);
}

TEST_F(TestDublinsCarDynamics, TestCombinedInputs)
{
  double x = 1.0;
  double y = 2.0;
  double theta = M_PI / 4;
  double v = 3.0;
  double omega = 0.2;

  double a = 0.5;
  double alpha = 0.3;

  double d_x = 0.05;
  double d_y = 0.05;
  double d_a = 0.1;
  double d_alpha = 0.01;

  StateVector state = (StateVector(5) << x, y, theta, v, omega).finished();
  ControlVector control = (ControlVector(2) << a, alpha).finished();
  DisturbanceVector disturbance = (DisturbanceVector(4) << d_x, d_y, d_a, d_alpha).finished();

  auto dynamics = tracking_dynamics->dynamics(state, control, disturbance);

  double expected_dx = (v * std::cos(theta)) + d_x;
  double expected_dy = (v * std::sin(theta)) + d_y;
  double expected_omega = omega;
  double expected_a = a + d_a;
  double expected_alpha = alpha + d_alpha;

  EXPECT_NEAR(dynamics[0], expected_dx, test_tolerance);
  EXPECT_NEAR(dynamics[1], expected_dy, test_tolerance);
  EXPECT_NEAR(dynamics[2], expected_omega, test_tolerance);
  EXPECT_NEAR(dynamics[3], expected_a, test_tolerance);
  EXPECT_NEAR(dynamics[4], expected_alpha, test_tolerance);
}

TEST_F(TestDublinsCarDynamics, TestRelativeDynamicsWithMismatchQDimensions)
{
  StateVector ts = Eigen::VectorXd::Zero(5);
  ControlVector tc = Eigen::VectorXd::Zero(2);
  StateVector ps = Eigen::VectorXd::Zero(3);
  ControlVector pc = Eigen::VectorXd::Zero(2);
  DisturbanceVector d = Eigen::VectorXd::Zero(4);

  // mismatch row dimensions
  Eigen::MatrixXd Q_1 = Eigen::MatrixXd::Zero(5, 4);
  EXPECT_THROW(relative_dynamics->dynamics(Q_1, phi, ts, tc, ps, pc, d), std::invalid_argument);

  // mismatch column dimensions
  Eigen::MatrixXd Q_2 = Eigen::MatrixXd::Zero(4, 5);
  EXPECT_THROW(relative_dynamics->dynamics(Q_2, phi, ts, tc, ps, pc, d), std::invalid_argument);
}

TEST_F(TestDublinsCarDynamics, TestRelativeDynamicsWithMismatchPhiDimensions)
{
  StateVector ts = Eigen::VectorXd::Zero(5);
  ControlVector tc = Eigen::VectorXd::Zero(2);
  StateVector ps = Eigen::VectorXd::Zero(3);
  ControlVector pc = Eigen::VectorXd::Zero(2);
  DisturbanceVector d = Eigen::VectorXd::Zero(4);

  // mismatch row dimensions
  Eigen::MatrixXd phi_1 = Eigen::MatrixXd::Zero(4, 5);
  EXPECT_THROW(relative_dynamics->dynamics(Q, phi_1, ts, tc, ps, pc, d), std::invalid_argument);

  // mismatch column dimensions
  Eigen::MatrixXd phi_2 = Eigen::MatrixXd::Zero(5, 4);
  EXPECT_THROW(relative_dynamics->dynamics(Q, phi_2, ts, tc, ps, pc, d), std::invalid_argument);
}

TEST_F(TestDublinsCarDynamics, TestRelativeDynamics)
{
  // tracking state
  double x = 1.0;
  double y = 2.0;
  double theta = M_PI / 4;
  double v = 3.0;
  double omega = 0.2;
  StateVector ts = (StateVector(5) << x, y, theta, v, omega).finished();

  // tracking control
  double a = 0.5;
  double alpha = 0.3;
  ControlVector tc = (ControlVector(2) << a, alpha).finished();

  // disturbance
  double d_x = 0.05;
  double d_y = 0.05;
  double d_a = 0.1;
  double d_alpha = 0.01;
  DisturbanceVector d = (DisturbanceVector(4) << d_x, d_y, d_a, d_alpha).finished();

  // planning state
  double x_hat = 1.1;
  double y_hat = 2.2;
  double theta_hat = M_PI / 6;
  StateVector ps = (StateVector(3) << x_hat, y_hat, theta_hat).finished();

  // planning control
  double v_hat = 3.5;
  double omega_hat = 0.3;
  ControlVector pc = (ControlVector(2) << v_hat, omega_hat).finished();

  // mismatch row dimensions
  Eigen::MatrixXd phi_(5, 5);
  phi_ << std::cos(theta_hat), std::sin(theta_hat), 0, 0, 0, -std::sin(theta_hat),
    std::cos(theta_hat), 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;

  StateVector dynamics = relative_dynamics->dynamics(Q, phi_, ts, tc, ps, pc, d);

  // relative states
  StateVector relative_state = phi_ * (ts - Q * ps);
  double x_r = relative_state(0);
  double y_r = relative_state(1);
  double theta_r = relative_state(2);

  double x_dot_r = -v_hat + (v * std::cos(theta_r)) + (omega_hat * y_r) + d_x;
  double y_dot_r = (v * std::sin(theta_r)) - (omega_hat * x_r) + d_y;
  double theta_dot_r = omega - omega_hat;
  double v_dot = a + d_a;
  double omega_dot = alpha + d_alpha;

  EXPECT_NEAR(dynamics[0], x_dot_r, test_tolerance);
  EXPECT_NEAR(dynamics[1], y_dot_r, test_tolerance);
  EXPECT_NEAR(dynamics[2], theta_dot_r, test_tolerance);
  EXPECT_NEAR(dynamics[3], v_dot, test_tolerance);
  EXPECT_NEAR(dynamics[4], omega_dot, test_tolerance);
}
// NOLINTEND(misc-non-private-member-variables-in-classes)
