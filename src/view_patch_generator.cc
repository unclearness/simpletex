/*
 * Copyright (C) 2019, unclearness
 * All rights reserved.
 */

#include "include/view_patch_generator.h"

#include "src/timer.h"

namespace simpletex {
ViewPatchGenerator::ViewPatchGenerator(){};
ViewPatchGenerator::~ViewPatchGenerator(){};

bool ViewPatchGenerator::SimpleGenerate(
    const VisibilityInfo& info,
    const std::vector<std::shared_ptr<Keyframe>>& keyframes, const Mesh& mesh,
    std::vector<ViewPatch>* view_patch_list) const {
  assert(view_patch_list != nullptr);

  Timer<> timer;
  timer.Start();
  // const auto& vertices = mesh_->vertices();
  // const auto& face_normals = mesh_->face_normals();
  const auto& faces = mesh.vertex_indices();
  int face_num = static_cast<int>(faces.size());

  view_patch_list->clear();
  view_patch_list->resize(face_num);

#if defined(_OPENMP) && defined(SIMPLETEX_USE_OPENMP)
#pragma omp parallel for schedule(dynamic, 1)
#endif
  for (int i = 0; i < face_num; i++) {
    const auto& face_info = info.face_info_list[i];

    // unseen face
    if (face_info.visible_keyframes.empty()) {
      continue;
    }

    // todo: add option for selection
    int best_index = face_info.min_viewing_angle_index;
    const auto& best_info = face_info.visible_keyframes[best_index];

    auto& p = (*view_patch_list)[i];
    p.kf_id = best_info.kf_id;
    p.face_ids.push_back(i);
    p.kf_image_min = best_info.bmin;

    Eigen::Vector2i patch_size =
        best_info.bmax - best_info.bmin + Eigen::Vector2i(1, 1);
    // all valid
    p.mask.Init(patch_size[0], patch_size[1], 255);

    p.patch.Init(patch_size[0], patch_size[1], 0);

    // const std::shared_ptr<Keyframe> keyframe =
    const auto keyframe =
        *std::find_if(keyframes.begin(), keyframes.end(),
                      [&](const auto& a) { return a->id == p.kf_id; });

    // todo: make region copy
    for (int jj = best_info.bmin[1]; jj < best_info.bmax[1] + 1; jj++) {
      for (int ii = best_info.bmin[0]; ii < best_info.bmax[0] + 1; ii++) {
        for (int c = 0; c < 3; c++) {
          p.patch.at(ii - best_info.bmin[0], jj - best_info.bmin[1], c) =
              keyframe->color.at(ii, jj, c);
        }
      }
    }

  }

  timer.End();
  LOGI("SimpleGenerate: %.1f msecs\n", timer.elapsed_msec());
  return true;
}

bool GenerateAtlsUv(const std::vector<ViewPatch>& view_patch_list, Mesh* mesh) {
  return false;
}

}  // namespace simpletex
