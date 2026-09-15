#pragma once

#include <cstddef>

#include "a2w_fastlio_common/registration.hpp"

namespace a2w_fastlio_common
{

struct NanoGicpRegistrationConfig
{
  double maximum_correspondence_distance_m{2.0};
  int thread_count{0};
  int correspondence_randomness{20};
  int maximum_iterations{64};
  double transformation_epsilon{1e-3};
  double rotation_epsilon{1e-3};
  int regularization_method{3};
  double fitness_score_max_range_m{1.0};
  std::size_t minimum_points{20U};
};

class NanoGicpRegistration final : public FineRegistration
{
public:
  explicit NanoGicpRegistration(NanoGicpRegistrationConfig config = {});

  RegistrationResult align(
    const CloudConstPtr & source, const CloudConstPtr & target,
    const std::optional<Pose3d> & initial_guess) const override;

private:
  NanoGicpRegistrationConfig config_;
};

}  // namespace a2w_fastlio_common
