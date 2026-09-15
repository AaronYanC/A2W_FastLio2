#pragma once

#include <cstddef>

#include "a2w_fastlio_common/place_recognition.hpp"

namespace a2w_fastlio_common
{

struct ScanContextConfig
{
  std::size_t rings{20U};
  std::size_t sectors{60U};
  double max_radius_m{80.0};
  double sensor_height_m{2.0};
};

class ScanContextPlaceRecognition final : public PlaceRecognition
{
public:
  explicit ScanContextPlaceRecognition(ScanContextConfig config);

  ScanDescriptor describe(const CloudConstPtr & cloud) const override;

private:
  ScanContextConfig config_;
};

}  // namespace a2w_fastlio_common
