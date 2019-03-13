/*
 * Copyright (C) 2019, unclearness
 * All rights reserved.
 */

#include <filesystem>
#include <fstream>
#include <iomanip>  // std::setw(int), std::setfill(char)
#include <ios>      // std::left, std::right
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#ifdef SIMPLETEX_USE_TINYOBJLOADER
#ifndef SIMPLETEX_TINYOBJLOADER_IMPLEMENTATION
#define TINYOBJLOADER_IMPLEMENTATION
#endif
#include "tinyobjloader/tiny_obj_loader.h"
#undef TINYOBJLOADER_IMPLEMENTATION
#endif

#ifdef SIMPLETEX_USE_STB
#pragma warning(push)
#pragma warning(disable : 4100)
#ifndef SIMPLETEX_STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#endif
#include "stb/stb_image.h"
#pragma warning(pop)
#undef STB_IMAGE_IMPLEMENTATION

#pragma warning(push)
#pragma warning(disable : 4996)
#ifndef SIMPLETEX_STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#endif
#include "stb/stb_image_write.h"
#pragma warning(pop)
#undef STB_IMAGE_WRITE_IMPLEMENTATION
#endif

#include "include//visibility_tester.h"
#include "include/vertex_colorizer.h"

// http://redwood-data.org/indoor/fileformat.html
namespace {

typedef std::pair<int, int> IntPair;

struct FramedTransformation {
  int id1_;
  int id2_;
  int frame_;
  glm::mat4 transformation_;
  FramedTransformation(int id1, int id2, int f, glm::mat4 t)
      : id1_(id1), id2_(id2), frame_(f), transformation_(t) {}
};

struct RGBDTrajectory {
  std::vector<FramedTransformation> data_;
  int index_;

  void LoadFromFile(std::string filename) {
    data_.clear();
    index_ = 0;
    int id1, id2, frame;
    glm::mat4 trans;
    FILE* f = fopen(filename.c_str(), "r");
    if (f != NULL) {
      char buffer[1024];
      while (fgets(buffer, 1024, f) != NULL) {
        if (strlen(buffer) > 0 && buffer[0] != '#') {
          sscanf(buffer, "%d %d %d", &id1, &id2, &frame);
          fgets(buffer, 1024, f);
          sscanf(buffer, "%f %f %f %f", &trans[0][0], &trans[0][1],
                 &trans[0][2], &trans[0][3]);
          fgets(buffer, 1024, f);
          sscanf(buffer, "%f %f %f %f", &trans[1][0], &trans[1][1],
                 &trans[1][2], &trans[1][3]);
          fgets(buffer, 1024, f);
          sscanf(buffer, "%f %f %f %f", &trans[2][0], &trans[2][1],
                 &trans[2][2], &trans[2][3]);
          fgets(buffer, 1024, f);
          sscanf(buffer, "%f %f %f %f", &trans[3][0], &trans[3][1],
                 &trans[3][2], &trans[3][3]);

          data_.push_back(FramedTransformation(id1, id2, frame, trans));
        }
      }
      fclose(f);
    }
  }
  void SaveToFile(std::string filename) {
    FILE* f = fopen(filename.c_str(), "w");
    for (size_t i = 0; i < data_.size(); i++) {
      glm::mat4& trans = data_[i].transformation_;
      fprintf(f, "%d\t%d\t%d\n", data_[i].id1_, data_[i].id2_, data_[i].frame_);
      fprintf(f, "%.8f %.8f %.8f %.8f\n", trans[0][0], trans[0][1], trans[0][2],
              trans[0][3]);
      fprintf(f, "%.8f %.8f %.8f %.8f\n", trans[1][0], trans[1][1], trans[1][2],
              trans[1][3]);
      fprintf(f, "%.8f %.8f %.8f %.8f\n", trans[2][0], trans[2][1], trans[2][2],
              trans[2][3]);
      fprintf(f, "%.8f %.8f %.8f %.8f\n", trans[3][0], trans[3][1], trans[3][2],
              trans[3][3]);
    }
    fclose(f);
  }
};

void LoadKeyframeId(const std::string& index_path,
                    std::vector<int>* keyframe_id) {
  std::ifstream ifs(index_path);
  std::string str;
  if (ifs.fail()) {
    return;
  }
  while (getline(ifs, str)) {
    keyframe_id->push_back(std::atoi(str.c_str()));
  }
}

void LoadImagePath(const std::string& image_dir,
                   const std::vector<int>& keyframe_id,
                   std::vector<std::string>* image_path) {
  std::vector<std::string> all_file_names;
  namespace fs = std::experimental::filesystem;
  fs::path p(image_dir.c_str());  // start point
  std::for_each(fs::directory_iterator(p), fs::directory_iterator(),
                //  recursive ver
                //  std::for_each(sys::recursive_directory_iterator(p),
                //  sys::recursive_directory_iterator(),
                [&](const fs::path& p) {
                  if (fs::is_regular_file(p)) {  // if file
                    // std::cout << "file: " << p.filename() << std::endl;
                    all_file_names.push_back(p.filename().string());
                  } else if (fs::is_directory(p)) {  // if dir
                    // std::cout << "dir.: " << p.string() << std::endl;
                  }
                });

  for (size_t i = 0; i < keyframe_id.size(); i++) {
    std::stringstream ss;
    ss << std::setfill('0') << std::right << std::setw(7) << keyframe_id[i];
    std::string prefix = ss.str();
    std::string path;
    for (auto& fn : all_file_names) {
      if (fn.find(prefix) == 0) {
        path = image_dir + fn;
        break;
      }
    }
    image_path->push_back(path);
  }
}

}  // namespace

int main(void) {
  std::string data_dir = "../data/fountain_all/";
  std::string pose_path = data_dir + "fountain_key.log";
  std::string index_path = data_dir + "key.txt";
  std::string image_dir = data_dir + "vga/";
  std::string input_mesh_path =
      data_dir + "fountain_from_kinectfusion_text.ply";
  std::string output_mesh_path = data_dir + "fountain_simpletex.ply";

  int width = 1280 / 2;
  int height = 1024 / 2;
  glm::vec2 focal_length(1050.0f / 2, 1050.0f / 2);
  glm::vec2 principal_point(639.5f / 2, 511.5f / 2);

  // load pose
  RGBDTrajectory keyframe_pose;
  keyframe_pose.LoadFromFile(pose_path);
  // keyframe_pose.SaveToFile(data_dir + "out.txt");

  // load image id
  std::vector<int> keyframe_id;
  LoadKeyframeId(index_path, &keyframe_id);
  // for (auto& id : keyframe_id) {
  //  printf("%d\n", id);
  //}

  // get image path
  std::vector<std::string> keyframe_image_path;
  LoadImagePath(image_dir, keyframe_id, &keyframe_image_path);

  // load mesh
  std::shared_ptr<simpletex::Mesh> input_mesh =
      std::make_shared<simpletex::Mesh>();
  input_mesh->LoadPly(input_mesh_path);
  input_mesh->CalcNormal();

  // convert to simpletex::Keyframe
  simpletex::VisibilityTesterOption option;
  option.use_mask = false;
  option.use_depth = false;
  simpletex::VisibilityTester tester(option);
  simpletex::VisibilityInfo info;
  std::vector<std::shared_ptr<simpletex::Keyframe>> keyframes(
      keyframe_id.size());
  for (size_t i = 0; i < keyframe_pose.data_.size(); i++) {
    keyframes[i] = std::make_shared<simpletex::Keyframe>();
    keyframes[i]->id = keyframe_id[i];

    glm::mat4 c2w =
        glm::transpose(keyframe_pose.data_[i].transformation_);  // transpose
    keyframes[i]->camera = std::make_shared<simpletex::PinholeCamera>(
        width, height, simpletex::Pose(c2w), principal_point, focal_length);

    keyframes[i]->color_path = keyframe_image_path[i];
    keyframes[i]->color.Load(keyframes[i]->color_path);
  }

  tester.set_mesh(input_mesh);
  tester.PrepareMesh();

  info.vertex_info_list.resize(input_mesh->vertices().size());
  info.face_info_list.resize(input_mesh->vertex_indices().size());

  tester.Test(keyframes, &info);

  simpletex::VertexColorizer vertex_colorizer;
  std::shared_ptr<simpletex::Mesh> output_mesh =
      std::make_shared<simpletex::Mesh>(*input_mesh.get());
  vertex_colorizer.Colorize(info, output_mesh.get());

  output_mesh->WritePly(output_mesh_path);

  return 0;
}
