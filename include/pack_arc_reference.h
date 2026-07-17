#pragma once

#include "IMotionReference.h"

#include <string>
#include <vector>

namespace wbc_deploy {

/// Pack WBC Arc reference (39-D) from an ``IMotionReference`` source.
std::vector<float> pack_arc_reference(
  const IMotionReference& ref,
  float env_origin_z = 0.0f);

}  // namespace wbc_deploy
