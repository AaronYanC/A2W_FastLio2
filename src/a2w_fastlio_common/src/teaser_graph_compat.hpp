#pragma once

#include <vector>

// TEASER++ v1.0 graph.cc uses the unqualified legacy spelling `vector`.
// Its public API remains std::vector; this forced include preserves the pinned
// submodule while making that translation unit independently compilable.
using std::vector;
