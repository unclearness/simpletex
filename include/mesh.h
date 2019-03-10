/*
 * Copyright (C) 2019, unclearness
 * All rights reserved.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "include/common.h"
#include "include/image.h"

namespace simpletex {

struct MeshStats {
  glm::vec3 center;
  glm::vec3 bb_min;
  glm::vec3 bb_max;
};

class Mesh {
  std::vector<glm::vec3> vertices_;
  std::vector<glm::vec3> vertex_colors_;    // optional, RGB order
  std::vector<glm::ivec3> vertex_indices_;  // face

  std::vector<glm::vec3> normals_;       // normal per vertex
  std::vector<glm::vec3> face_normals_;  // normal per face
  std::vector<glm::ivec3> normal_indices_;

  std::vector<glm::vec2> uv_;
  std::vector<glm::ivec3> uv_indices_;

  std::string diffuse_texname_;
  std::string diffuse_texpath_;
  Image3b diffuse_tex_;
  MeshStats stats_;

 public:
  Mesh();
  ~Mesh();
  Mesh(const Mesh& src);
  void Clear();

  // get average normal per vertex from face normal
  // caution: this does not work for cube with 8 vertices unless vertices are
  // splitted (24 vertices)
  void CalcNormal();

  void CalcFaceNormal();
  void CalcStats();
  void Rotate(const glm::mat3& R);
  void Translate(const glm::vec3& t);
  void Transform(const glm::mat3& R, const glm::vec3& t);
  void Scale(float scale);
  void Scale(float x_scale, float y_scale, float z_scale);
  const std::vector<glm::vec3>& vertices() const;
  const std::vector<glm::vec3>& vertex_colors() const;
  const std::vector<glm::ivec3>& vertex_indices() const;
  const std::vector<glm::vec3>& normals() const;
  const std::vector<glm::vec3>& face_normals() const;
  const std::vector<glm::ivec3>& normal_indices() const;
  const std::vector<glm::vec2>& uv() const;
  const std::vector<glm::ivec3>& uv_indices() const;
  const MeshStats& stats() const;
  const Image3b& diffuse_tex() const;

  void set_vertices(const std::vector<glm::vec3>& vertices);
  void set_vertex_colors(const std::vector<glm::vec3>& vertex_colors);
  void set_vertex_indices(const std::vector<glm::ivec3>& vertex_indices);
  void set_normals(const std::vector<glm::vec3>& normals);
  void set_face_normals(const std::vector<glm::vec3>& face_normals);
  void set_normal_indices(const std::vector<glm::ivec3>& normal_indices);
  void set_uv(const std::vector<glm::vec2>& uv);
  void set_uv_indices(const std::vector<glm::ivec3>& uv_indices);
  void set_diffuse_tex(const Image3b& diffuse_tex);

#ifdef SIMPLETEX_USE_TINYOBJLOADER
  bool LoadObj(const std::string& obj_path, const std::string& mtl_dir);
#endif
  bool LoadPly(const std::string& ply_path);
  bool WritePly(const std::string& ply_path) const;
};

// make cube with 24 vertices
std::shared_ptr<Mesh> MakeCube(const glm::vec3& length, const glm::mat3& R,
                               const glm::vec3& t);
std::shared_ptr<Mesh> MakeCube(const glm::vec3& length);
std::shared_ptr<Mesh> MakeCube(float length, const glm::mat3& R,
                               const glm::vec3& t);
std::shared_ptr<Mesh> MakeCube(float length);

void SetRandomVertexColor(std::shared_ptr<Mesh> mesh, int seed = 0);

}  // namespace simpletex
