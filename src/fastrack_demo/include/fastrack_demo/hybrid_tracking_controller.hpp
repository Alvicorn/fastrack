
namespace fastrack_demo
{

template <typename ParamsT, typename TS, typename PS, typename RS, typename TC, typename PC>
class HybridTrackingController
{
  public:
  explicit HybridTrackingController(ParamsT& params)
    : params_(params)
  {
  }
  virtual ~HybridTrackingController() = default;

  TC computeControl(const TS& s, const PS& p)
  {
    if (useOptimalTrackingController(s, p)) {
      RS r = computeRelativeState(s, p);
      return optimalTrackingController(r);
    }
    return performanceController(s, p);
  }

  protected:
  ParamsT& params_;

  private:
  virtual RS computeRelativeState(const TS& s, const PS& p) = 0;
  virtual RS relativeDynamics(const RS& r, const PC& up, const TC& us) = 0;

  virtual TC optimalTrackingController(const RS& r) = 0;
  virtual TC performanceController(const TS& s, const PS& p) = 0;

  virtual bool useOptimalTrackingController(const TS& s, const PS& p) = 0;
};

}  // namespace fastrack_demo
