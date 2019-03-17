/*
 * Copyright (C) 2019, unclearness
 * All rights reserved.
 */

#pragma once

#include "include/visibility_tester.h"

namespace simpletex {

struct ViewPatch {
  int kf_id;
  std::vector<int> face_ids;
  Eigen::Vector2i kf_image_min;  // min in original keyframe image
  Image3b patch;                 // cropped and masked keyframe image
  Image1b mask;                  // cropped mask for keyframe image
};

class ViewPatchGenerator {
 public:
  ViewPatchGenerator();
  ~ViewPatchGenerator();
  bool SimpleGenerate(const VisibilityInfo& info,
                      const std::vector<std::shared_ptr<Keyframe>>& keyframes,
                      const Mesh& mesh,
                      std::vector<ViewPatch>* view_patch_list) const;
};

bool GenerateAtlsUv(const std::vector<ViewPatch>& view_patch_list, Mesh* mesh);

}  // namespace simpletex
