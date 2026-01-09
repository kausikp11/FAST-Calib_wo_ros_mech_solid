#define PCL_NO_PRECOMPILE
#pragma once
#include <iostream>
#include <string>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>

#define TARGET_NUM_CIRCLES 4
#define DEBUG 1
#define GEOMETRY_TOLERANCE 0.08

// 相机参数结构体
struct CameraIntrinsics {
    std::string cam_model;
    int cam_width;
    int cam_height;
    double fx;
    double fy;
    double cx;
    double cy;
    double k1;
    double k2;
    double p1;
    double p2;

    cv::Mat getCameraMatrix() {
        return (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    }

    cv::Mat getDistCoeffs() {
        return (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, 0);
    }
};

namespace Common {
  struct Point {
    PCL_ADD_POINT4D;
    float intensity = 0.0f;
    std::uint16_t ring = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(Common::Point,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
);

struct InputDataInfo {
    std::string base_path;
    std::string bag_file;
    std::string img_file;
    std::string pcd_dir;
    std::vector<std::string> pcd_txt_files;
};

struct Parameters {
    CameraIntrinsics camera_intrinsics;
    double marker_size = 0.2;
    // Aruco中心距离标靶中心的width距离
    double delta_width_qr_center = 0.55;
    // Aruco中心距离标靶中心的height距离
    double delta_height_qr_center = 0.55;
    double delta_width_circles = 0.5;
    double delta_height_circles = 0.4;
    uint32_t min_detected_markers = 3;
    double circle_radius = 0.12;
    std::vector<double> crop_min_xyz = {-1.6, 1.0, -1.0};
    std::vector<double> crop_max_xyz = {0.5, 3.5, 1.0};
    double voxel_downsample_size = 0.005;
    double plane_dist_threshold = 0.05;
    // 标定板平面点云法向量估计搜索半径
    double target_normal_radius = 0.1;
    // 标定板平面点云边界检测搜索半径
    double target_boundary_radius = 0.1;
    // 标定板平面点云边界检测角度阈值
    double target_boundary_angle_thres = M_PI / 2;

    double cluster_tolerance = 0.1;
    int    min_cluster_size  = 5;
    int    max_cluster_size  = 650;
};

inline void comb(int N, int K, std::vector<std::vector<int>> &groups) {
  int upper_factorial = 1;
  int lower_factorial = 1;

  for (int i = 0; i < K; i++) {
    upper_factorial *= (N - i);
    lower_factorial *= (K - i);
  }
  int n_permutations = upper_factorial / lower_factorial;

  if (DEBUG)
    LOG(INFO) << N << " centers found. Iterating over " << n_permutations
         << " possible sets of candidates" << std::endl;

  std::string bitmask(K, 1);  // K leading 1's
  bitmask.resize(N, 0);       // N-K trailing 0's

  // print integers and permute bitmask
  do {
    std::vector<int> group;
    for (int i = 0; i < N; ++i)  // [0..N-1] integers
    {
      if (bitmask[i]) {
        group.push_back(i);
      }
    }
    groups.push_back(group);
  } while (std::prev_permutation(bitmask.begin(), bitmask.end()));

  assert(groups.size() == n_permutations);
}

// typedef pcl::PointXYZI PointT;
// typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
// typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudPtr;

using PointT = Common::Point;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloud::Ptr;

// Forward declaration (implemented in qr_detect_utils.cpp)
void sortPatternCenters(
    pcl::PointCloud<PointT>::Ptr in_pc,
    pcl::PointCloud<PointT>::Ptr out_v,
    const std::string &axis_mode);

class Square
{
private:
  PointT _center;
  std::vector<PointT> _candidates;
  float _target_width;
  float _target_height;
  float _target_diagonal;

public:
  Square(const std::vector<PointT> &candidates,
         float width,
         float height)
      : _candidates(candidates),
        _target_width(width),
        _target_height(height)
  {
    _target_diagonal = std::sqrt(width * width + height * height);

    _center.x = _center.y = _center.z = 0.0f;
    for (const auto &p : _candidates)
    {
      _center.x += p.x;
      _center.y += p.y;
      _center.z += p.z;
    }

    const float inv = 1.0f / _candidates.size();
    _center.x *= inv;
    _center.y *= inv;
    _center.z *= inv;
  }

  inline float distance(const PointT &a, const PointT &b) const
  {
    return std::sqrt(
        (a.x - b.x) * (a.x - b.x) +
        (a.y - b.y) * (a.y - b.y) +
        (a.z - b.z) * (a.z - b.z));
  }

  inline bool is_valid() const
  {
    if (_candidates.size() != 4)
      return false;

    pcl::PointCloud<PointT>::Ptr candidates_cloud =
        std::make_shared<pcl::PointCloud<PointT>>();
    for (const auto &p : _candidates)
      candidates_cloud->push_back(p);

    // 1. Diagonal consistency check
    const float half_diag = _target_diagonal * 0.5f;
    for (const auto &p : _candidates)
    {
      float d = distance(_center, p);
      if (std::fabs(d - half_diag) / half_diag >
          GEOMETRY_TOLERANCE * 2.0f)
      {
        return false;
      }
    }

    // 2. Sort corners counter-clockwise
    pcl::PointCloud<PointT>::Ptr sorted_centers =
        std::make_shared<pcl::PointCloud<PointT>>();
    sortPatternCenters(candidates_cloud, sorted_centers, "camera");

    // 3. Side length checks
    float s01 = distance(sorted_centers->points[0], sorted_centers->points[1]);
    float s12 = distance(sorted_centers->points[1], sorted_centers->points[2]);
    float s23 = distance(sorted_centers->points[2], sorted_centers->points[3]);
    float s30 = distance(sorted_centers->points[3], sorted_centers->points[0]);

    bool pattern1 =
        std::fabs(s01 - _target_width) / _target_width < GEOMETRY_TOLERANCE &&
        std::fabs(s12 - _target_height) / _target_height < GEOMETRY_TOLERANCE &&
        std::fabs(s23 - _target_width) / _target_width < GEOMETRY_TOLERANCE &&
        std::fabs(s30 - _target_height) / _target_height < GEOMETRY_TOLERANCE;

    bool pattern2 =
        std::fabs(s01 - _target_height) / _target_height < GEOMETRY_TOLERANCE &&
        std::fabs(s12 - _target_width) / _target_width < GEOMETRY_TOLERANCE &&
        std::fabs(s23 - _target_height) / _target_height < GEOMETRY_TOLERANCE &&
        std::fabs(s30 - _target_width) / _target_width < GEOMETRY_TOLERANCE;

    if (!pattern1 && !pattern2)
      return false;

    // 4. Perimeter sanity check
    float perimeter = s01 + s12 + s23 + s30;
    float ideal = 2.0f * (_target_width + _target_height);

    if (std::fabs(perimeter - ideal) / ideal > GEOMETRY_TOLERANCE)
      return false;

    return true;
  }
};


// define macro to new a PointCloudPtr
#define MAKE_POINTCLOUD() std::make_shared<PointCloud>()
