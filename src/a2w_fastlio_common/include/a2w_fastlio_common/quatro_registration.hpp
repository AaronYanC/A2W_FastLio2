#pragma once

#include <cstddef>

#include "a2w_fastlio_common/registration.hpp"

namespace a2w_fastlio_common
{

struct QuatroRegistrationConfig
{
  double fpfh_normal_radius_m{0.5};
  double fpfh_radius_m{0.8};
  double noise_bound_m{0.25};
  double rotation_gnc_factor{1.4};
  double rotation_cost_threshold{1e-4};
  int rotation_max_iterations{100};
  bool estimate_scale{false};
  bool optimized_matching{true};
  double descriptor_distance_threshold{30.0};
  int maximum_correspondences{500};
  std::size_t minimum_points{20U};
};

class QuatroRegistration final : public CoarseRegistration
{
public:
  explicit QuatroRegistration(QuatroRegistrationConfig config = {});

  RegistrationResult align(
    const CloudConstPtr & source, const CloudConstPtr & target,
    const std::optional<Pose3d> & initial_guess) const override;

private:
  QuatroRegistrationConfig config_;
};

}  // namespace a2w_fastlio_common
