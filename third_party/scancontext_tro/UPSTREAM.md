# Scan Context provenance

- Upstream: `https://github.com/engcang/scancontext_tro`
- Revision: `c8ef5b496a159cdfd7fa4761121178f25cd0a6bb`
- License: CC BY-NC-SA 4.0; see `LICENSE`
- Original algorithm files: `Scancontext.cpp`, `Scancontext.h`

This curated snapshot keeps the polar maximum-height descriptor core used by this project. It does
not include a ROS node, catkin package, bag support, launch file, or another FAST-LIO frontend.

Local integration changes:

- moved symbols into `scancontext_tro`;
- replaced fixed ring, sector, radius, and sensor-height constants with an immutable configuration;
- removed ROS/PCL-conversions includes not required by the algorithm core;
- exposed one ROS-independent descriptor-generation function using the project's PCL point alias.

The polar binning and maximum-height encoding equations are unchanged.
