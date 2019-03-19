/*
 * Copyright (C) 2019, unclearness
 * All rights reserved.
 */

#include "include/mesh.h"

#include <fstream>
#include <random>
#include <sstream>

#ifdef SIMPLETEX_USE_TINYOBJLOADER
#ifdef SIMPLETEX_TINYOBJLOADER_IMPLEMENTATION
#define TINYOBJLOADER_IMPLEMENTATION
#endif
#include "tinyobjloader/tiny_obj_loader.h"
#endif

namespace {
template <typename T>
void CopyVec(const std::vector<T>& src, std::vector<T>* dst) {
  dst->clear();
  std::copy(src.begin(), src.end(), std::back_inserter(*dst));
}

std::vector<std::string> Split(const std::string& s, char delim) {
  std::vector<std::string> elems;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    if (!item.empty()) {
      elems.push_back(item);
    }
  }
  return elems;
}
}  // namespace

namespace simpletex {

Mesh::Mesh() {}
Mesh::Mesh(const Mesh& src) {
  CopyVec(src.vertices_, &vertices_);
  CopyVec(src.vertex_colors_, &vertex_colors_);
  CopyVec(src.vertex_indices_, &vertex_indices_);

  CopyVec(src.normals_, &normals_);
  CopyVec(src.face_normals_, &face_normals_);
  CopyVec(src.normal_indices_, &normal_indices_);

  CopyVec(src.uv_, &uv_);
  CopyVec(src.uv_indices_, &uv_indices_);

  diffuse_texname_ = src.diffuse_texname_;
  diffuse_texpath_ = src.diffuse_texpath_;
  src.diffuse_tex_.CopyTo(&diffuse_tex_);
  stats_ = src.stats_;
}
Mesh::~Mesh() {}

const std::vector<Eigen::Vector3f>& Mesh::vertices() const { return vertices_; }
const std::vector<Eigen::Vector3f>& Mesh::vertex_colors() const {
  return vertex_colors_;
}
const std::vector<Eigen::Vector3i>& Mesh::vertex_indices() const {
  return vertex_indices_;
}

const std::vector<Eigen::Vector3f>& Mesh::normals() const { return normals_; }
const std::vector<Eigen::Vector3f>& Mesh::face_normals() const {
  return face_normals_;
}
const std::vector<Eigen::Vector3i>& Mesh::normal_indices() const {
  return normal_indices_;
}

const std::vector<Eigen::Vector2f>& Mesh::uv() const { return uv_; }
const std::vector<Eigen::Vector3i>& Mesh::uv_indices() const {
  return uv_indices_;
}

const MeshStats& Mesh::stats() const { return stats_; }

const Image3b& Mesh::diffuse_tex() const { return diffuse_tex_; }

void Mesh::CalcStats() {
  stats_.bb_min = Eigen::Vector3f(std::numeric_limits<float>::max(),
                                  std::numeric_limits<float>::max(),
                                  std::numeric_limits<float>::max());
  stats_.bb_max = Eigen::Vector3f(std::numeric_limits<float>::lowest(),
                                  std::numeric_limits<float>::lowest(),
                                  std::numeric_limits<float>::lowest());

  if (vertex_indices_.empty()) {
    return;
  }

  double sum[3] = {0.0, 0.0, 0.0};  // use double to avoid overflow
  for (const auto& v : vertices_) {
    for (int i = 0; i < 3; i++) {
      sum[i] += v[i];

      if (v[i] < stats_.bb_min[i]) {
        stats_.bb_min[i] = v[i];
      }

      if (stats_.bb_max[i] < v[i]) {
        stats_.bb_max[i] = v[i];
      }
    }
  }

  for (int i = 0; i < 3; i++) {
    stats_.center[i] = static_cast<float>(sum[i] / vertices_.size());
  }
}

void Mesh::Rotate(const Eigen::Matrix3f& R) {
  for (auto& v : vertices_) {
    v = R * v;
  }
  for (auto& n : normals_) {
    n = R * n;
  }
  for (auto& fn : face_normals_) {
    fn = R * fn;
  }
  CalcStats();
}

void Mesh::Translate(const Eigen::Vector3f& t) {
  for (auto& v : vertices_) {
    v = v + t;
  }
  CalcStats();
}

void Mesh::Transform(const Eigen::Matrix3f& R, const Eigen::Vector3f& t) {
  Rotate(R);
  Translate(t);
}

void Mesh::Scale(float scale) { Scale(scale, scale, scale); }

void Mesh::Scale(float x_scale, float y_scale, float z_scale) {
  for (auto& v : vertices_) {
    v[0] = v[0] * x_scale;
    v[1] = v[1] * y_scale;
    v[2] = v[2] * z_scale;
  }
}

void Mesh::Clear() {
  vertices_.clear();
  vertex_colors_.clear();
  vertex_indices_.clear();  // face

  normals_.clear();
  normal_indices_.clear();

  uv_.clear();
  uv_indices_.clear();

  diffuse_tex_.Clear();
}

void Mesh::CalcNormal() {
  CalcFaceNormal();

  normals_.clear();
  normal_indices_.clear();

  std::copy(vertex_indices_.begin(), vertex_indices_.end(),
            std::back_inserter(normal_indices_));

  Eigen::Vector3f zero{0.0f, 0.0f, 0.0f};
  normals_.resize(vertices_.size(), zero);

  std::vector<int> add_count(vertices_.size(), 0);

  for (size_t i = 0; i < vertex_indices_.size(); i++) {
    const auto& face = vertex_indices_[i];
    for (int j = 0; j < 3; j++) {
      int idx = face[j];
      normals_[idx] += face_normals_[i];
      add_count[idx]++;
    }
  }

  // get average normal
  // caution: this does not work for cube
  // https://answers.unity.com/questions/441722/splitting-up-verticies.html
  for (size_t i = 0; i < vertices_.size(); i++) {
    normals_[i] /= static_cast<float>(add_count[i]);
    normals_[i].normalize();
  }
}

void Mesh::CalcFaceNormal() {
  face_normals_.clear();
  face_normals_.resize(vertex_indices_.size());

  for (size_t i = 0; i < vertex_indices_.size(); i++) {
    const auto& f = vertex_indices_[i];
    Eigen::Vector3f v1 = (vertices_[f[1]] - vertices_[f[0]]).normalized();
    Eigen::Vector3f v2 = (vertices_[f[2]] - vertices_[f[0]]).normalized();
    face_normals_[i] = v1.cross(v2).normalized();
  }
}

void Mesh::set_vertices(const std::vector<Eigen::Vector3f>& vertices) {
  CopyVec(vertices, &vertices_);
}

void Mesh::set_vertex_colors(
    const std::vector<Eigen::Vector3f>& vertex_colors) {
  CopyVec(vertex_colors, &vertex_colors_);
}

void Mesh::set_vertex_indices(
    const std::vector<Eigen::Vector3i>& vertex_indices) {
  CopyVec(vertex_indices, &vertex_indices_);
}

void Mesh::set_normals(const std::vector<Eigen::Vector3f>& normals) {
  CopyVec(normals, &normals_);
}

void Mesh::set_face_normals(const std::vector<Eigen::Vector3f>& face_normals) {
  CopyVec(face_normals, &face_normals_);
}

void Mesh::set_normal_indices(
    const std::vector<Eigen::Vector3i>& normal_indices) {
  CopyVec(normal_indices, &normal_indices_);
}

void Mesh::set_uv(const std::vector<Eigen::Vector2f>& uv) { CopyVec(uv, &uv_); }

void Mesh::set_uv_indices(const std::vector<Eigen::Vector3i>& uv_indices) {
  CopyVec(uv_indices, &uv_indices_);
}

void Mesh::set_diffuse_tex(const Image3b& diffuse_tex) {
  diffuse_tex.CopyTo(&diffuse_tex_);
}

#ifdef SIMPLETEX_USE_TINYOBJLOADER
bool Mesh::LoadObj(const std::string& obj_path, const std::string& mtl_dir) {
  Clear();

  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  tinyobj::attrib_t attrib;
  std::string err_str, warn_str;
  bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn_str, &err_str,
                              obj_path.c_str(), mtl_dir.c_str());

  if (!err_str.empty()) {  // `err` may contain warning message.
    LOGE("%s\n", err_str.c_str());
  }

  if (!ret) {
    return false;
  }

  if (materials.size() != 1) {
    LOGE("Doesn't support obj materials num %d. Must be 1\n",
         static_cast<int>(materials.size()));
    return false;
  }

  size_t face_num = 0;
  for (size_t s = 0; s < shapes.size(); s++) {
    face_num += shapes[s].mesh.num_face_vertices.size();
  }
  vertex_indices_.resize(face_num);  // face
  uv_indices_.resize(face_num);
  normal_indices_.resize(face_num);

  vertices_.resize(attrib.vertices.size() / 3);
  normals_.resize(attrib.normals.size() / 3);
  uv_.resize(attrib.texcoords.size() / 2);
  vertex_colors_.resize(attrib.colors.size() / 3);

  size_t face_offset = 0;
  // Loop over shapes
  for (size_t s = 0; s < shapes.size(); s++) {
    // Loop over faces(polygon)
    size_t index_offset = 0;

    for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
      int fv = shapes[s].mesh.num_face_vertices[f];

      if (fv != 3) {
        LOGE("Doesn't support face num %d. Must be 3\n", fv);
        return false;
      }

      // Loop over vertices in the face.
      for (int v = 0; v < fv; v++) {
        // access to vertex
        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
        tinyobj::real_t vx = attrib.vertices[3 * idx.vertex_index + 0];
        tinyobj::real_t vy = attrib.vertices[3 * idx.vertex_index + 1];
        tinyobj::real_t vz = attrib.vertices[3 * idx.vertex_index + 2];

        vertex_indices_[face_offset][v] = idx.vertex_index;

        vertices_[idx.vertex_index][0] = vx;
        vertices_[idx.vertex_index][1] = vy;
        vertices_[idx.vertex_index][2] = vz;

        if (!attrib.normals.empty()) {
          tinyobj::real_t nx = attrib.normals[3 * idx.normal_index + 0];
          tinyobj::real_t ny = attrib.normals[3 * idx.normal_index + 1];
          tinyobj::real_t nz = attrib.normals[3 * idx.normal_index + 2];

          normal_indices_[face_offset][v] = idx.normal_index;
          normals_[idx.normal_index][0] = nx;
          normals_[idx.normal_index][1] = ny;
          normals_[idx.normal_index][2] = nz;
        }

        if (!attrib.texcoords.empty()) {
          tinyobj::real_t tx = attrib.texcoords[2 * idx.texcoord_index + 0];
          tinyobj::real_t ty = attrib.texcoords[2 * idx.texcoord_index + 1];

          uv_indices_[face_offset][v] = idx.texcoord_index;
          uv_[idx.texcoord_index][0] = tx;
          uv_[idx.texcoord_index][1] = ty;
        }
        // Optional: vertex colors
        if (!attrib.colors.empty()) {
          tinyobj::real_t red = attrib.colors[3 * idx.vertex_index + 0];
          tinyobj::real_t green = attrib.colors[3 * idx.vertex_index + 1];
          tinyobj::real_t blue = attrib.colors[3 * idx.vertex_index + 2];

          vertex_colors_[idx.vertex_index][0] = red;
          vertex_colors_[idx.vertex_index][1] = green;
          vertex_colors_[idx.vertex_index][2] = blue;
        }
      }
      index_offset += fv;
      face_offset++;

      // per-face material
      shapes[s].mesh.material_ids[f];
    }
  }

  CalcFaceNormal();

  if (normals_.empty()) {
    CalcNormal();
  }

  CalcStats();

  diffuse_texname_ = materials[0].diffuse_texname;
  diffuse_texpath_ = mtl_dir + diffuse_texname_;

  std::ifstream ifs(diffuse_texpath_);
  if (ifs.is_open()) {
#ifdef SIMPLETEX_USE_STB
    ret = diffuse_tex_.Load(diffuse_texpath_);
#else
    LOGW("define simpletex_USE_STB to load diffuse texture.\n");
#endif
  } else {
    LOGW("diffuse texture doesn't exist %s\n", diffuse_texpath_.c_str());
  }

  return ret;
}
#endif

bool Mesh::LoadPly(const std::string& ply_path) {
  std::ifstream ifs(ply_path);
  std::string str;
  if (ifs.fail()) {
    LOGE("couldn't open ply: %s\n", ply_path.c_str());
    return false;
  }

  getline(ifs, str);
  if (str != "ply") {
    LOGE("ply first line is wrong: %s\n", str.c_str());
    return false;
  }
  getline(ifs, str);
  if (str.find("ascii") == std::string::npos) {
    LOGE("only ascii ply is supported: %s\n", str.c_str());
    return false;
  }

  bool ret = false;
  int vertex_num = 0;
  while (getline(ifs, str)) {
    if (str.find("element vertex") != std::string::npos) {
      std::vector<std::string> splitted = Split(str, ' ');
      if (splitted.size() == 3) {
        vertex_num = std::atoi(splitted[2].c_str());
        ret = true;
        break;
      }
    }
  }
  if (!ret) {
    LOGE("couldn't find element vertex\n");
    return false;
  }

  ret = false;
  int face_num = 0;
  while (getline(ifs, str)) {
    if (str.find("element face") != std::string::npos) {
      std::vector<std::string> splitted = Split(str, ' ');
      if (splitted.size() == 3) {
        face_num = std::atoi(splitted[2].c_str());
        ret = true;
        break;
      }
    }
  }
  if (!ret) {
    LOGE("couldn't find element face\n");
    return false;
  }

  while (getline(ifs, str)) {
    if (str.find("end_header") != std::string::npos) {
      break;
    }
  }

  vertices_.resize(vertex_num);
  int vertex_count = 0;
  while (getline(ifs, str)) {
    std::vector<std::string> splitted = Split(str, ' ');
    vertices_[vertex_count][0] =
        static_cast<float>(std::atof(splitted[0].c_str()));
    vertices_[vertex_count][1] =
        static_cast<float>(std::atof(splitted[1].c_str()));
    vertices_[vertex_count][2] =
        static_cast<float>(std::atof(splitted[2].c_str()));
    vertex_count++;
    if (vertex_count >= vertex_num) {
      break;
    }
  }

  vertex_indices_.resize(face_num);
  int face_count = 0;
  while (getline(ifs, str)) {
    std::vector<std::string> splitted = Split(str, ' ');
    vertex_indices_[face_count][0] = std::atoi(splitted[1].c_str());
    vertex_indices_[face_count][1] = std::atoi(splitted[2].c_str());
    vertex_indices_[face_count][2] = std::atoi(splitted[3].c_str());

    face_count++;
    if (face_count >= face_num) {
      break;
    }
  }

  ifs.close();

  CalcNormal();

  CalcStats();

  return true;
}

bool Mesh::WritePly(const std::string& ply_path) const {
  std::ofstream ofs(ply_path);
  std::string str;
  if (ofs.fail()) {
    LOGE("couldn't open ply: %s\n", ply_path.c_str());
    return false;
  }

  bool has_vertex_color = !vertex_colors_.empty();
  if (has_vertex_color) {
    assert(vertices_.size() == vertex_colors_.size());
  }

  ofs << "ply" << std::endl;
  ofs << "format ascii 1.0" << std::endl;
  ofs << "element vertex " + std::to_string(vertices_.size()) << std::endl;
  ofs << "property float x\n"
         "property float y\n"
         "property float z\n";
  if (has_vertex_color) {
    ofs << "property uchar red\n"
           "property uchar green\n"
           "property uchar blue\n"
           "property uchar alpha\n";
  }
  ofs << "element face " + std::to_string(vertex_indices_.size()) << std::endl;
  ofs << "property list uchar int vertex_indices" << std::endl;
  ofs << "end_header" << std::endl;

  for (size_t i = 0; i < vertices_.size(); i++) {
    ofs << vertices_[i][0] << " " << vertices_[i][1] << " " << vertices_[i][2]
        << " ";
    if (has_vertex_color) {
      ofs << static_cast<int>(std::round(vertex_colors_[i][0])) << " "
          << static_cast<int>(std::round(vertex_colors_[i][1])) << " "
          << static_cast<int>(std::round(vertex_colors_[i][2])) << " 255 ";
    }
    ofs << std::endl;
  }

  for (size_t i = 0; i < vertex_indices_.size(); i++) {
    ofs << "3 " << vertex_indices_[i][0] << " " << vertex_indices_[i][1] << " "
        << vertex_indices_[i][2] << " " << std::endl;
  }

  ofs.close();

  return true;
}

bool Mesh::WriteObj(const std::string& obj_dir, const std::string& obj_basename,
                    const std::string& mtl_basename,
                    const std::string& tex_basename) const {
  std::string mtl_name = mtl_basename + ".tml";
  if (mtl_basename.empty()) {
    mtl_name = obj_basename + ".mtl";
  }
  std::string mtl_path = obj_dir + "/" + mtl_name;

  std::string tex_name = tex_basename + ".png";
  if (tex_basename.empty()) {
    tex_name = obj_basename + ".png";
  }
  std::string tex_path = obj_dir + "/" + tex_name;

  std::string obj_path = obj_dir + "/" + obj_basename + ".obj";

  // write obj
  {
    std::ofstream ofs(obj_path);
    if (ofs.fail()) {
      LOGE("couldn't open obj path: %s\n", obj_path.c_str());
      return false;
    }

    ofs << "mtllib ./" << mtl_name << std::endl << std::endl;

    // vertices
    for (const auto& v : vertices_) {
      ofs << "v " << v.x() << " " << v.y() << " " << v.z() << " 1.0"
          << std::endl;
    }

    // uv
    for (const auto& vt : uv_) {
      ofs << "vt " << vt.x() << " " << vt.y() << " 0" << std::endl;
    }

    // vertex normals
    for (const auto& vn : normals_) {
      ofs << "vn " << vn.x() << " " << vn.y() << " " << vn.z() << std::endl;
    }

    // indices
    bool write_uv_indices = !uv_indices_.empty();
    bool write_normal_indices = !normal_indices_.empty();
    for (size_t i = 0; i < vertex_indices_.size(); i++) {
      ofs << "f";
      for (int j = 0; j < 3; j++) {
        ofs << " " << std::to_string(vertex_indices_[i][j] + 1);
        if (!write_uv_indices && !write_normal_indices) {
          continue;
        }
        ofs << "/";
        if (write_uv_indices) {
          ofs << std::to_string(uv_indices_[i][j] + 1);
        }
        ofs << "/" << std::to_string(normal_indices_[i][j] + 1);
      }
      ofs << std::endl;
    }

    ofs.close();
  }

  // write mtl
  {
    std::ofstream ofs(mtl_path);
    if (ofs.fail()) {
      LOGE("couldn't open mtl path: %s\n", mtl_path.c_str());
      return false;
    }

    ofs << "property float x\n"
           "property float y\n"
           "property float z\n"
           "newmtl Textured\n"
           "Ka 1.000 1.000 1.000\n"
           "Kd 1.000 1.000 1.000\n"
           "Ks 0.000 0.000 0.000\n"
           "d 1.0\n"
           "illum 2\n";
    ofs << "map_Kd " + tex_name << std::endl;

    ofs.close();
  }

  // write texture
  diffuse_tex_.WritePng(tex_path);

  return true;
}

std::shared_ptr<Mesh> MakeCube(const Eigen::Vector3f& length,
                               const Eigen::Matrix3f& R,
                               const Eigen::Vector3f& t) {
  std::shared_ptr<Mesh> cube(new Mesh);
  std::vector<Eigen::Vector3f> vertices(24);
  std::vector<Eigen::Vector3i> vertex_indices(12);
  std::vector<Eigen::Vector3f> vertex_colors(24);

  const float h_x = length.x() / 2;
  const float h_y = length.y() / 2;
  const float h_z = length.z() / 2;

  vertices[0] = Eigen::Vector3f(-h_x, h_y, -h_z);
  vertices[1] = Eigen::Vector3f(h_x, h_y, -h_z);
  vertices[2] = Eigen::Vector3f(h_x, h_y, h_z);
  vertices[3] = Eigen::Vector3f(-h_x, h_y, h_z);
  vertex_indices[0] = Eigen::Vector3i(0, 2, 1);
  vertex_indices[1] = Eigen::Vector3i(0, 3, 2);

  vertices[4] = Eigen::Vector3f(-h_x, -h_y, -h_z);
  vertices[5] = Eigen::Vector3f(h_x, -h_y, -h_z);
  vertices[6] = Eigen::Vector3f(h_x, -h_y, h_z);
  vertices[7] = Eigen::Vector3f(-h_x, -h_y, h_z);
  vertex_indices[2] = Eigen::Vector3i(4, 5, 6);
  vertex_indices[3] = Eigen::Vector3i(4, 6, 7);

  vertices[8] = vertices[1];
  vertices[9] = vertices[2];
  vertices[10] = vertices[6];
  vertices[11] = vertices[5];
  vertex_indices[4] = Eigen::Vector3i(8, 9, 10);
  vertex_indices[5] = Eigen::Vector3i(8, 10, 11);

  vertices[12] = vertices[0];
  vertices[13] = vertices[3];
  vertices[14] = vertices[7];
  vertices[15] = vertices[4];
  vertex_indices[6] = Eigen::Vector3i(12, 14, 13);
  vertex_indices[7] = Eigen::Vector3i(12, 15, 14);

  vertices[16] = vertices[0];
  vertices[17] = vertices[1];
  vertices[18] = vertices[5];
  vertices[19] = vertices[4];
  vertex_indices[8] = Eigen::Vector3i(16, 17, 18);
  vertex_indices[9] = Eigen::Vector3i(16, 18, 19);

  vertices[20] = vertices[3];
  vertices[21] = vertices[2];
  vertices[22] = vertices[6];
  vertices[23] = vertices[7];
  vertex_indices[10] = Eigen::Vector3i(20, 22, 21);
  vertex_indices[11] = Eigen::Vector3i(20, 23, 22);

  // set default color
  for (int i = 0; i < 24; i++) {
    vertex_colors[i][0] = (-vertices[i][0] + h_x) / length.x() * 255;
    vertex_colors[i][1] = (-vertices[i][1] + h_y) / length.y() * 255;
    vertex_colors[i][2] = (-vertices[i][2] + h_z) / length.z() * 255;
  }

  cube->set_vertices(vertices);
  cube->set_vertex_indices(vertex_indices);
  cube->set_vertex_colors(vertex_colors);

  cube->Transform(R, t);

  cube->CalcNormal();

  return cube;
}

std::shared_ptr<Mesh> MakeCube(const Eigen::Vector3f& length) {
  const Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
  const Eigen::Vector3f t(0.0f, 0.0f, 0.0f);
  return MakeCube(length, R, t);
}

std::shared_ptr<Mesh> MakeCube(float length, const Eigen::Matrix3f& R,
                               const Eigen::Vector3f& t) {
  Eigen::Vector3f length_xyz{length, length, length};
  return MakeCube(length_xyz, R, t);
}

std::shared_ptr<Mesh> MakeCube(float length) {
  const Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
  const Eigen::Vector3f t(0.0f, 0.0f, 0.0f);
  return MakeCube(length, R, t);
}

void SetRandomVertexColor(std::shared_ptr<Mesh> mesh, int seed) {
  std::mt19937 mt(seed);
  std::uniform_int_distribution<int> random_color(0, 255);

  std::vector<Eigen::Vector3f> vertex_colors(mesh->vertices().size());
  for (auto& vc : vertex_colors) {
    vc[0] = static_cast<float>(random_color(mt));
    vc[1] = static_cast<float>(random_color(mt));
    vc[2] = static_cast<float>(random_color(mt));
  }

  mesh->set_vertex_colors(vertex_colors);
}

}  // namespace simpletex
