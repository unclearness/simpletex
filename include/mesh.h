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
  Eigen::Vector3f center;
  Eigen::Vector3f bb_min;
  Eigen::Vector3f bb_max;
};

class Mesh {
  std::vector<Eigen::Vector3f> vertices_;
  std::vector<Eigen::Vector3f> vertex_colors_;   // optional, RGB order
  std::vector<Eigen::Vector3i> vertex_indices_;  // face

  std::vector<Eigen::Vector3f> normals_;       // normal per vertex
  std::vector<Eigen::Vector3f> face_normals_;  // normal per face
  std::vector<Eigen::Vector3i> normal_indices_;

  std::vector<Eigen::Vector2f> uv_;
  std::vector<Eigen::Vector3i> uv_indices_;

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
  void Rotate(const Eigen::Matrix3f& R);
  void Translate(const Eigen::Vector3f& t);
  void Transform(const Eigen::Matrix3f& R, const Eigen::Vector3f& t);
  void Scale(float scale);
  void Scale(float x_scale, float y_scale, float z_scale);
  const std::vector<Eigen::Vector3f>& vertices() const;
  const std::vector<Eigen::Vector3f>& vertex_colors() const;
  const std::vector<Eigen::Vector3i>& vertex_indices() const;
  const std::vector<Eigen::Vector3f>& normals() const;
  const std::vector<Eigen::Vector3f>& face_normals() const;
  const std::vector<Eigen::Vector3i>& normal_indices() const;
  const std::vector<Eigen::Vector2f>& uv() const;
  const std::vector<Eigen::Vector3i>& uv_indices() const;
  const MeshStats& stats() const;
  const Image3b& diffuse_tex() const;

  void set_vertices(const std::vector<Eigen::Vector3f>& vertices);
  void set_vertex_colors(const std::vector<Eigen::Vector3f>& vertex_colors);
  void set_vertex_indices(const std::vector<Eigen::Vector3i>& vertex_indices);
  void set_normals(const std::vector<Eigen::Vector3f>& normals);
  void set_face_normals(const std::vector<Eigen::Vector3f>& face_normals);
  void set_normal_indices(const std::vector<Eigen::Vector3i>& normal_indices);
  void set_uv(const std::vector<Eigen::Vector2f>& uv);
  void set_uv_indices(const std::vector<Eigen::Vector3i>& uv_indices);
  void set_diffuse_tex(const Image3b& diffuse_tex);

#ifdef SIMPLETEX_USE_TINYOBJLOADER
  bool LoadObj(const std::string& obj_path, const std::string& mtl_dir);
#endif
  bool LoadPly(const std::string& ply_path);
  bool WritePly(const std::string& ply_path) const;
#ifdef SIMPLETEX_USE_STB
  bool WriteObj(const std::string& obj_dir, const std::string& obj_basename,
                const std::string& mtl_basename = "",
                const std::string& tex_basename = "") const;
#endif
};

// make cube with 24 vertices
std::shared_ptr<Mesh> MakeCube(const Eigen::Vector3f& length,
                               const Eigen::Matrix3f& R,
                               const Eigen::Vector3f& t);
std::shared_ptr<Mesh> MakeCube(const Eigen::Vector3f& length);
std::shared_ptr<Mesh> MakeCube(float length, const Eigen::Matrix3f& R,
                               const Eigen::Vector3f& t);
std::shared_ptr<Mesh> MakeCube(float length);

void SetRandomVertexColor(std::shared_ptr<Mesh> mesh, int seed = 0);

}  // namespace simpletex
