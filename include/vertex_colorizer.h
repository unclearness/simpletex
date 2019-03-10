/*
 * Copyright (C) 2019, unclearness
 * All rights reserved.
 */

#pragma once

#include "include/visibility_tester.h"

namespace simpletex {

class VertexColorizer {
 public:
  VertexColorizer();
  ~VertexColorizer();
  bool Colorize(const VisibilityInfo& info, Mesh* mesh) const;
};

}  // namespace simpletex
