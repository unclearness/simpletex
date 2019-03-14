/*
 * Copyright (C) 2019, unclearness
 * All rights reserved.
 */

#pragma once

#include <cassert>

#include "Eigen/Geometry"
#include "include/log.h"

namespace simpletex {

// borrow from glm
// radians
template <typename genType>
genType radians(genType degrees) {
  // "'radians' only accept floating-point input"
  assert(std::numeric_limits<genType>::is_iec559);

  return degrees * static_cast<genType>(0.01745329251994329576923690768489);
}

// degrees
template <typename genType>
genType degrees(genType radians) {
  // "'degrees' only accept floating-point input"
  assert(std::numeric_limits<genType>::is_iec559);

  return radians * static_cast<genType>(57.295779513082320876798154814105);
}
}  // namespace simpletex
