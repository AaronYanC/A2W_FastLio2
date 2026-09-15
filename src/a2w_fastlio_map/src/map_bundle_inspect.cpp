#include <iostream>
#include <stdexcept>

#include "a2w_fastlio_map/map_bundle_reader.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: map_bundle_inspect BUNDLE_ROOT\n";
    return 2;
  }
  try {
    const auto bundle = a2w_fastlio_map::MapBundleReader{}.read(argv[1]);
    std::cout << "schema=" << bundle.data.metadata.schema << '\n'
              << "bundle_uuid=" << bundle.data.metadata.bundle_uuid << '\n'
              << "keyframe_count=" << bundle.data.keyframes.size() << '\n'
              << "hardware_validation_status="
              << bundle.data.metadata.hardware_validation_status << '\n';
  } catch (const std::exception & error) {
    std::cerr << "Map Bundle verification failed: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
