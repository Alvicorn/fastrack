#include <array>
#include <cmath>
#include <fastrack_demo/hybrid_tracking_controller.hpp>

namespace fastrack_demo
{
namespace dubins_car
{
struct PlannerState
{
  double x_hat, y_hat, theta_hat;
};
struct TrackerState
{
  double x, y, theta, v, omega;
};
struct RelativeState
{
  double xr, yr, theta_r, v, omega;
};
struct PlannerControl
{
  double v_hat, omega_hat;
};
struct TrackerControl
{
  double a, alpha;
};

struct DubinsCarParameters
{
  double min_v;
  double min_v_hat;
  double max_v;
  double max_v_hat;
  double max_omega;
  double max_omega_hat;
  double max_a;
  double max_alpha;
  double dt;
  double safety_threshold;
};

class DubinsCarHybridTrackingController : public HybridTrackingController<DubinsCarParameters,
                                            TrackerState,
                                            PlannerState,
                                            RelativeState,
                                            TrackerControl,
                                            PlannerControl>
{
  public:
  using Base = HybridTrackingController<DubinsCarParameters,
    TrackerState,
    PlannerState,
    RelativeState,
    TrackerControl,
    PlannerControl>;
  using Base::Base;

  private:
  bool useOptimalTrackingController(TrackerState const& s, PlannerState const& p) override
  {
    RelativeState r = computeRelativeState(s, p);
    double V = valueFunction(r);
    return V > params_.safety_threshold;
  }

  RelativeState computeRelativeState(TrackerState const& s, PlannerState const& p) override
  {
    RelativeState r;
    double dx = s.x - p.x_hat;
    double dy = s.y - p.y_hat;
    double cos_t = cos(-p.theta_hat);
    double sin_t = sin(-p.theta_hat);
    r.xr = (cos_t * dx) - (sin_t * dy);
    r.yr = (sin_t * dx) + (cos_t * dy);
    r.theta_r = wrapAngle(s.theta - p.theta_hat);
    r.v = s.v;
    r.omega = s.omega;
    return r;
  }

  RelativeState relativeDynamics(
    RelativeState const& r, PlannerControl const& up, TrackerControl const& us) override
  {
    // ignore disturbance for now
    RelativeState rdot;
    rdot.xr = -up.v_hat + (r.v * cos(r.theta_r)) + (up.omega_hat * r.yr);
    rdot.yr = (r.v * sin(r.theta_r)) - (up.omega_hat * r.xr);
    rdot.theta_r = r.omega - up.omega_hat;
    rdot.v = us.a;
    rdot.omega = us.alpha;
    return rdot;
  }

  TrackerControl optimalTrackingController(RelativeState const& r) override
  {
    auto gradV = gradValueFunction(r);

    double best_cost = std::numeric_limits<double>::infinity();
    TrackerControl best_u{0.0, 0.0};

    // brute-force search over accelerations
    for (double a = -params_.max_a; a <= params_.max_a; a += 0.2) {
      for (double alpha = -params_.max_alpha; alpha <= params_.max_alpha; alpha += 0.2) {
        TrackerControl us{a, alpha};
        PlannerControl up = plannerController(gradV);
        RelativeState rdot = relativeDynamics(r, up, us);
        double cost = (gradV[0] * rdot.xr) + (gradV[1] * rdot.yr) + (gradV[2] * rdot.theta_r) +
                      (gradV[3] * rdot.v) + (gradV[4] * rdot.omega);

        if (cost < best_cost) {
          best_cost = cost;
          best_u = us;
        }
      }
    }
    return best_u;
  }

  TrackerControl performanceController(TrackerState const& s, PlannerState const& p) override
  {
    // simple PD toward planner
    double dx = p.x_hat - s.x;
    double dy = p.y_hat - s.y;
    double target_theta = atan2(dy, dx);
    double angle_error = wrapAngle(target_theta - s.theta);

    TrackerControl u;
    u.a = 0.5;  // accelerate forward
    u.alpha = std::clamp(angle_error, -params_.max_alpha, params_.max_alpha);
    return u;
  }

  PlannerControl plannerController(std::array<double, 5> const& gradV)
  {
    PlannerControl u;
    double gx = gradV[0];
    double gy = gradV[1];
    double norm = std::sqrt(gx * gx + gy * gy) + 1e-6;
    u.v_hat = params_.max_v_hat * (gx / norm);
    u.omega_hat = params_.max_omega_hat;
    return u;
  }

  double valueFunction(RelativeState const& r)
  {
    // Euclidean norm in position space
    return std::sqrt(r.xr * r.xr + r.yr * r.yr);
  }

  std::array<double, 5> gradValueFunction(RelativeState const& r)
  {
    double norm = std::sqrt(r.xr * r.xr + r.yr * r.yr) + 1e-6;
    return {r.xr / norm, r.yr / norm, 0.0, 0.0, 0.0};
  }

  static double wrapAngle(double a)
  {
    while (a > M_PI)
      a -= 2 * M_PI;
    while (a < -M_PI)
      a += 2 * M_PI;
    return a;
  }
};

}  // namespace dubins_car
}  // namespace fastrack_demo
