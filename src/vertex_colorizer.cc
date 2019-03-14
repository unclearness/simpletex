/*
 * Copyright (C) 2019, unclearness
 * All rights reserved.
 */

#include "include/vertex_colorizer.h"

namespace simpletex {

VertexColorizer::VertexColorizer() {}
VertexColorizer::~VertexColorizer() {}
bool VertexColorizer::Colorize(const VisibilityInfo& info, Mesh* mesh) const {
  if (!info.has_vertex_stat) {
    LOGE("no vertex stat\n");
    return false;
  }

  assert(info.vertex_info_list.size() == mesh->vertices().size());

  std::vector<Eigen::Vector3f> vertex_colors;

  for (size_t i = 0; i < info.vertex_info_list.size(); i++) {
    vertex_colors.push_back(info.vertex_info_list[i].min_viewing_angle_color);
  }

  mesh->set_vertex_colors(vertex_colors);

  return true;
}

}  // namespace simpletex
