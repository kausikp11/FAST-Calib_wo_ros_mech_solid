/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIDAR_DETECT_HPP
#define LIDAR_DETECT_HPP
#define PCL_NO_PRECOMPILE

#include <glog/logging.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_base.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "struct.hpp"

class LidarDetect
{
private:
  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
  double circle_radius_;
  double voxel_down_size_;
  double plane_dist_threshold_;

  double target_normal_radius_;
  double target_boundary_radius_;
  double target_boundary_angle_thres_;
  double cluster_tolerance_;
  int min_cluster_size_, max_cluster_size_;
  double delta_width_circles_, delta_height_circles_,circle_tolerance_;

  // 存储中间结果的点云
  PointCloudPtr filtered_cloud_;
  PointCloudPtr plane_cloud_;
  PointCloudPtr aligned_cloud_;
  PointCloudPtr edge_cloud_;
  PointCloudPtr center_z0_cloud_;
  std::vector<pcl::PointIndices> cluster_indices_;
  std::vector<PointCloudPtr> cluster_clouds_;

public:
  LidarDetect(Parameters &params)
      : filtered_cloud_(MAKE_POINTCLOUD()),
        plane_cloud_(MAKE_POINTCLOUD()),
        aligned_cloud_(MAKE_POINTCLOUD()),
        edge_cloud_(MAKE_POINTCLOUD()),
        center_z0_cloud_(MAKE_POINTCLOUD())
  {
    x_min_ = params.crop_min_xyz[0];
    x_max_ = params.crop_max_xyz[0];
    y_min_ = params.crop_min_xyz[1];
    y_max_ = params.crop_max_xyz[1];
    z_min_ = params.crop_min_xyz[2];
    z_max_ = params.crop_max_xyz[2];

    voxel_down_size_ = params.voxel_downsample_size;

    circle_radius_ = params.circle_radius;

    cluster_tolerance_ = params.cluster_tolerance;
    min_cluster_size_ = params.min_cluster_size;
    max_cluster_size_ = params.max_cluster_size;
    delta_width_circles_ = params.delta_width_circles;
    delta_height_circles_ = params.delta_height_circles;
    circle_tolerance_ = params.circle_tolerance;

    LOG(INFO) << "LidarDetect initialized, x_min: " << x_min_
              << ", x_max: " << x_max_ << ", y_min: " << y_min_
              << ", y_max: " << y_max_ << ", z_min: " << z_min_
              << ", z_max: " << z_max_ << ", circle_radius: " << circle_radius_
              << ", voxel_down_size: " << voxel_down_size_;

    plane_dist_threshold_ = params.plane_dist_threshold;
    LOG(INFO) << "Plane fitting params, plane_dist_threshold: "
              << plane_dist_threshold_;

    target_normal_radius_ = params.target_normal_radius;
    target_boundary_radius_ = params.target_boundary_radius;
    target_boundary_angle_thres_ = params.target_boundary_angle_thres;
    LOG(INFO) << "Target params, target_normal_radius: "
              << target_normal_radius_
              << ", target_boundary_radius: " << target_boundary_radius_
              << ", target_boundary_angle_thres: "
              << target_boundary_angle_thres_;
  }

  void detect_lidar(PointCloudPtr cloud, PointCloudPtr center_cloud)
  {
    // 1. X、Y、Z方向滤波
    filtered_cloud_->reserve(cloud->size());

    pcl::PassThrough<PointT> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_min_, x_max_); // 设置X轴范围
    pass_x.filter(*filtered_cloud_);

    pcl::PassThrough<PointT> pass_y;
    pass_y.setInputCloud(filtered_cloud_);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_min_, y_max_); // 设置Y轴范围
    pass_y.filter(*filtered_cloud_);

    pcl::PassThrough<PointT> pass_z;
    pass_z.setInputCloud(filtered_cloud_);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_min_, z_max_); // 设置Z轴范围
    pass_z.filter(*filtered_cloud_);

    LOG(INFO) << "PassThrough filtered cloud size: " << filtered_cloud_->size();

    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(filtered_cloud_);
    voxel_filter.setLeafSize(voxel_down_size_, voxel_down_size_,
                             voxel_down_size_);
    voxel_filter.filter(*filtered_cloud_);
    LOG(INFO) << "VoxelGrid filtered cloud size: " << filtered_cloud_->size();

    // 2. 平面分割
    plane_cloud_->reserve(filtered_cloud_->size());

    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> plane_segmentation;
    plane_segmentation.setModelType(pcl::SACMODEL_PLANE);
    plane_segmentation.setMethodType(pcl::SAC_RANSAC);
    plane_segmentation.setDistanceThreshold(
        plane_dist_threshold_); // 平面分割阈值
    plane_segmentation.setInputCloud(filtered_cloud_);
    plane_segmentation.segment(*plane_inliers, *plane_coefficients);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(filtered_cloud_);
    extract.setIndices(plane_inliers);
    extract.filter(*plane_cloud_);
    LOG(INFO) << "Plane cloud size: " << plane_cloud_->size();

    // 3. 平面点云对齐
    aligned_cloud_->reserve(plane_cloud_->size());

    Eigen::Vector3d normal(plane_coefficients->values[0],
                           plane_coefficients->values[1],
                           plane_coefficients->values[2]);
    normal.normalize();
    Eigen::Vector3d z_axis(0, 0, 1);

    Eigen::Vector3d axis = normal.cross(z_axis);
    double angle = acos(normal.dot(z_axis));

    Eigen::AngleAxisd rotation(angle, axis);
    Eigen::Matrix3d R = rotation.toRotationMatrix();
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = R;

    // 应用旋转矩阵，将平面对齐到 Z=0 平面
    float average_z = 0.0;
    pcl::transformPointCloud(*plane_cloud_, *aligned_cloud_, transform);
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*aligned_cloud_, centroid);
    average_z = centroid[2];

    Eigen::Vector3d eulers_rad = R.eulerAngles(2, 1, 0);
    Eigen::Vector3d eulers_deg = eulers_rad * 180.0 / M_PI;
    LOG(INFO) << "Plane fitting euler angles(degree): "
              << eulers_deg.transpose() << ", average z: " << average_z;

    // set aligned_cloud_ z coordinate to 0
    for (auto &point : *aligned_cloud_)
    {
      point.z = 0;
    }

    // 4. 提取边缘点
    edge_cloud_->reserve(aligned_cloud_->size());

    pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normal_estimator.setInputCloud(aligned_cloud_);
    normal_estimator.setRadiusSearch(target_normal_radius_); // 设置法线估计的搜索半径
    normal_estimator.compute(*normals);

    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary>
        boundary_estimator;
    boundary_estimator.setInputCloud(aligned_cloud_);
    boundary_estimator.setInputNormals(normals);
    boundary_estimator.setRadiusSearch(target_boundary_radius_);        // 设置边界检测的搜索半径
    boundary_estimator.setAngleThreshold(target_boundary_angle_thres_); // 设置角度阈值
    boundary_estimator.compute(boundaries);

    for (size_t i = 0; i < aligned_cloud_->size(); ++i)
    {
      if (boundaries.points[i].boundary_point > 0)
      {
        edge_cloud_->push_back(aligned_cloud_->points[i]);
      }
    }
    LOG(INFO) << "Edge cloud size: " << edge_cloud_->size();

    // 5. 对边缘点进行聚类
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(edge_cloud_);

    // std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(edge_cloud_);
    ec.extract(cluster_indices_);
    LOG(INFO) << "Number of edge clusters: " << cluster_indices_.size();

    // 6. 对每个聚类进行圆拟合
    center_z0_cloud_->reserve(4);
    Eigen::Matrix3d R_inv = R.inverse();

    // 对每个聚类进行圆拟合
    for (size_t i = 0; i < cluster_indices_.size(); ++i)
    {
      PointCloudPtr cluster(new pcl::PointCloud<PointT>);
      for (const auto &idx : cluster_indices_[i].indices)
      {
        cluster->push_back(edge_cloud_->points[idx]);
      }
      cluster_clouds_.push_back(cluster);

      // 圆拟合
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      pcl::SACSegmentation<PointT> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_CIRCLE2D);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.01); // 设置距离阈值
      // seg.setMaxIterations(5000);      // 设置最大迭代次数
      seg.setInputCloud(cluster);
      seg.segment(*inliers, *coefficients);

      if (inliers->indices.size() > 0)
      {
        // 计算拟合误差
        double error = 0.0;
        for (const auto &idx : inliers->indices)
        {
          double dx = cluster->points[idx].x - coefficients->values[0];
          double dy = cluster->points[idx].y - coefficients->values[1];
          double distance =
              sqrt(dx * dx + dy * dy) - circle_radius_; // 距离误差
          error += abs(distance);
        }
        error /= inliers->indices.size();

        // 如果拟合误差较小，则认为是一个圆洞
        if (error < 0.025)
        {
          // 将恢复后的圆心坐标添加到点云中
          PointT center_point;
          center_point.x = coefficients->values[0];
          center_point.y = coefficients->values[1];
          center_point.z = 0.0;
          center_z0_cloud_->push_back(center_point);

          // 将圆心坐标逆变换回原始坐标系
          Eigen::Vector3d aligned_point(center_point.x, center_point.y,
                                        center_point.z + average_z);
          Eigen::Vector3d original_point = R_inv * aligned_point;

          PointT center_point_origin;
          center_point_origin.x = original_point.x();
          center_point_origin.y = original_point.y();
          center_point_origin.z = original_point.z();
          center_cloud->points.push_back(center_point_origin);
        }
      }
    }

    center_cloud->width = 1;
    center_cloud->height = center_cloud->points.size();
  }
  // In lidar_detect.hpp, inside class LidarDetect public section:

  // Mechanical LiDAR mode: neighbor-gap based edge extraction + circle RANSAC
  void detect_mech_lidar(PointCloudPtr cloud, PointCloudPtr center_cloud)
  {
    // 1. PassThrough in X/Y/Z using existing limits
    filtered_cloud_->clear();
    filtered_cloud_->reserve(cloud->size());

    pcl::PassThrough<PointT> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_min_, x_max_);
    pass_x.filter(*filtered_cloud_);

    pcl::PassThrough<PointT> pass_y;
    pass_y.setInputCloud(filtered_cloud_);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_min_, y_max_);
    pass_y.filter(*filtered_cloud_);

    pcl::PassThrough<PointT> pass_z;
    pass_z.setInputCloud(filtered_cloud_);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_min_, z_max_);
    pass_z.filter(*filtered_cloud_);

    LOG(INFO) << "[mech] Depth filtered cloud size: " << filtered_cloud_->size();

    // 2. Plane segmentation to get ground plane and normal
    plane_cloud_->clear();
    plane_cloud_->reserve(filtered_cloud_->size());

    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> plane_segmentation;
    plane_segmentation.setModelType(pcl::SACMODEL_PLANE);
    plane_segmentation.setMethodType(pcl::SAC_RANSAC);
    plane_segmentation.setDistanceThreshold(plane_dist_threshold_);
    plane_segmentation.setMaxIterations(5000);
    plane_segmentation.setInputCloud(filtered_cloud_);
    plane_segmentation.segment(*plane_inliers, *plane_coefficients);

    pcl::ExtractIndices<PointT> extract_plane;
    extract_plane.setInputCloud(filtered_cloud_);
    extract_plane.setIndices(plane_inliers);
    extract_plane.filter(*plane_cloud_);

    LOG(INFO) << "[mech] Plane cloud size: " << plane_cloud_->size();

    // 3. Neighbor-gap edge extraction per ring (Common::Point::ring in ROS code)
    // Here we assume PointT is compatible with Common::Point (has .ring if needed).
    edge_cloud_->clear();
    edge_cloud_->reserve(filtered_cloud_->size());

    std::unordered_map<unsigned int, std::vector<int>> ring2indices;
    ring2indices.reserve(64);

    for (int i = 0; i < static_cast<int>(filtered_cloud_->size()); ++i)
    {
      const auto &pt = filtered_cloud_->points[i];
      ring2indices[pt.ring].push_back(i); // requires PointT to have 'ring'
      // LOG(INFO) << "ring number: " << i << " output: " << pt.ring;
    }

    const auto &c = plane_coefficients->values; // [a,b,c,d]
    Eigen::Vector3d n(c[0], c[1], c[2]);
    double norm_n = n.norm();
    Eigen::Vector3d normal = n / norm_n;

    const double neighbor_gap_threshold = 0.10; //change
    const int min_points_per_ring = 2;//chane

    for (auto &kv : ring2indices)
    {
      auto &idx_vec = kv.second;
      if (static_cast<int>(idx_vec.size()) < min_points_per_ring)
        continue;

      for (size_t k = 1; k + 1 < idx_vec.size(); ++k)
      {
        const auto &p_prev = filtered_cloud_->points[idx_vec[k - 1]];
        const auto &p_cur = filtered_cloud_->points[idx_vec[k]];
        const auto &p_next = filtered_cloud_->points[idx_vec[k + 1]];

        double dist_plane = std::fabs(
                                c[0] * p_cur.x + c[1] * p_cur.y + c[2] * p_cur.z + c[3]) /
                            norm_n;
        if (dist_plane >= 0.03)
          continue;

        double dx1 = double(p_cur.x) - double(p_prev.x);
        double dy1 = double(p_cur.y) - double(p_prev.y);
        double dz1 = double(p_cur.z) - double(p_prev.z);
        double dist_prev = std::sqrt(dx1 * dx1 + dy1 * dy1 + dz1 * dz1);

        double dx2 = double(p_cur.x) - double(p_next.x);
        double dy2 = double(p_cur.y) - double(p_next.y);
        double dz2 = double(p_cur.z) - double(p_next.z);
        double dist_next = std::sqrt(dx2 * dx2 + dy2 * dy2 + dz2 * dz2);

        if (dist_prev > neighbor_gap_threshold ||
            dist_next > neighbor_gap_threshold)
        {
          PointT edge_pt;
          edge_pt.x = p_cur.x;
          edge_pt.y = p_cur.y;
          edge_pt.z = p_cur.z;
          edge_pt.ring = p_cur.ring; // keep ring
          edge_cloud_->push_back(edge_pt);
        }
      }
    }

    LOG(INFO) << "[mech] Extracted " << edge_cloud_->size()
              << " edge points (neighbor distance).";

    // 4. Align edge points to Z=0 plane
    aligned_cloud_->clear();
    aligned_cloud_->reserve(edge_cloud_->size());

    Eigen::Vector3d z_axis(0, 0, 1);
    Eigen::Vector3d axis = normal.cross(z_axis);
    double angle = std::acos(normal.dot(z_axis));
    Eigen::AngleAxisd rotation(angle, axis);
    Eigen::Matrix3d R_align = rotation.toRotationMatrix();

    float average_z = 0.0f;
    int cnt = 0;
    for (const auto &pt : *edge_cloud_)
    {
      Eigen::Vector3d point(pt.x, pt.y, pt.z);
      Eigen::Vector3d aligned_point = R_align * point;
      PointT aligned_pt;
      aligned_pt.x = static_cast<float>(aligned_point.x());
      aligned_pt.y = static_cast<float>(aligned_point.y());
      aligned_pt.z = 0.0f;
      aligned_pt.ring = pt.ring; // or keep original ring if you prefer
      aligned_cloud_->push_back(aligned_pt);

      average_z += float(aligned_point.z());
      ++cnt;
    }
    if (cnt > 0)
      average_z /= cnt;

    // 5. RANSAC circle detection on aligned XY plane
    pcl::PointCloud<PointT>::Ptr xy_cloud(new pcl::PointCloud<PointT>(*aligned_cloud_));
    LOG(INFO) << "[mech] Start circle detection, initial cloud size = "
              << xy_cloud->size();

    pcl::SACSegmentation<PointT> circle_segmentation;
    circle_segmentation.setModelType(pcl::SACMODEL_CIRCLE2D);
    circle_segmentation.setMethodType(pcl::SAC_RANSAC);
    circle_segmentation.setDistanceThreshold(plane_dist_threshold_); 
    circle_segmentation.setOptimizeCoefficients(true);
    circle_segmentation.setMaxIterations(5000);
    circle_segmentation.setRadiusLimits(circle_radius_ - circle_tolerance_,
                                        circle_radius_ + circle_tolerance_);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ExtractIndices<PointT> extract2;

    center_z0_cloud_->clear();

    while (xy_cloud->size() > 3)
    {
      LOG(INFO) << "[mech] RANSAC on cloud of size " << xy_cloud->size();

      circle_segmentation.setInputCloud(xy_cloud);
      circle_segmentation.segment(*inliers, *coefficients);
      LOG(INFO) << "[mech] circle r = " << coefficients->values[2]
          << ", inliers = " << inliers->indices.size();

      if (inliers->indices.empty())
      {
        LOG(INFO) << "[mech] No more circles can be found, stop.";
        break;
      }
      if (coefficients->values[2] < circle_radius_ - circle_tolerance_ ||
    coefficients->values[2] > circle_radius_ + circle_tolerance_)
{
    // Reject weak / partial circle
    LOG(INFO) << "[mech] coeff";
    break;
}

      if (int(inliers->indices.size()) < 20) //5
      {
        LOG(INFO) << "[mech] Circle inliers too few (" << inliers->indices.size()
                  << "), stop.";
        break;
      }

      PointT center_point;
      center_point.x = coefficients->values[0];
      center_point.y = coefficients->values[1];
      center_point.z = 0.0f;
      center_z0_cloud_->push_back(center_point);

      extract2.setInputCloud(xy_cloud);
      extract2.setIndices(inliers);
      extract2.setNegative(true);
      pcl::PointCloud<PointT>::Ptr remaining(new pcl::PointCloud<PointT>);
      extract2.filter(*remaining);
      xy_cloud.swap(remaining);
      inliers->indices.clear();
    }
    LOG(INFO) << "[mech] Detected " << center_z0_cloud_->size()
              << " raw circle candidates before geometry filter.";

    // // 6. Geometric consistency check (select exactly 4 circles)
    // std::vector<std::vector<int>> groups;
    // comb(center_z0_cloud_->size(), TARGET_NUM_CIRCLES, groups);

    // std::vector<double> groups_scores(groups.size(), -1.0);

    // for (size_t i = 0; i < groups.size(); ++i)
    // {
    //   std::vector<PointT> candidates;
    //   candidates.reserve(TARGET_NUM_CIRCLES);

    //   for (int idx : groups[i])
    //   {
    //     const auto &cpt = center_z0_cloud_->points[idx];
    //     PointT p;
    //     p.x = cpt.x;
    //     p.y = cpt.y;
    //     p.z = cpt.z;
    //     p.ring = 0;      // safe default
    //     p.intensity = 0; // safe default
    //     candidates.push_back(p);
    //   }

    //   Square square_candidate(
    //       candidates,
    //       delta_width_circles_,
    //       delta_height_circles_);

    //   groups_scores[i] = square_candidate.is_valid() ? 1.0 : -1.0;
    // }

    // // Select best valid candidate
    // int best_candidate_idx = -1;
    // double best_candidate_score = -1.0;

    // for (size_t i = 0; i < groups_scores.size(); ++i)
    // {
    //   if (best_candidate_score == 1.0 && groups_scores[i] == 1.0)
    //   {
    //     LOG(ERROR)
    //         << "[mech] Multiple candidate groups match target geometry. "
    //         << "Check thresholds.";
    //     return;
    //   }
    //   if (groups_scores[i] > best_candidate_score)
    //   {
    //     best_candidate_score = groups_scores[i];
    //     best_candidate_idx = static_cast<int>(i);
    //   }
    // }

    // if (best_candidate_idx == -1)
    // {
    //   LOG(WARNING)
    //       << "[mech] No circle group matches target geometry.";
    //   return;
    // }

    // // 7. Transform selected 4 centers back to original LiDAR frame
    // Eigen::Matrix3d R_inv = R_align.inverse();
    // center_cloud->clear();

    // for (int idx : groups[best_candidate_idx])
    // {
    //   const auto &cpt = center_z0_cloud_->points[idx];

    //   Eigen::Vector3d aligned_point(
    //       cpt.x,
    //       cpt.y,
    //       cpt.z + average_z);

    //   Eigen::Vector3d original_point = R_inv * aligned_point;

    //   PointT center_point_origin;
    //   center_point_origin.x = original_point.x();
    //   center_point_origin.y = original_point.y();
    //   center_point_origin.z = original_point.z();
    //   center_cloud->push_back(center_point_origin);
    // }

    // center_cloud->width = 1;
    // center_cloud->height = center_cloud->points.size();

    // 6. Transform selected centers back to original coordinates
  Eigen::Matrix3d R_inv = R_align.inverse();
  center_cloud->clear();
  for (const auto& cpt : *center_z0_cloud_) {
    Eigen::Vector3d aligned_point(cpt.x, cpt.y, cpt.z + average_z);
    Eigen::Vector3d original_point = R_inv * aligned_point;

    PointT center_point_origin;
    center_point_origin.x = original_point.x();
    center_point_origin.y = original_point.y();
    center_point_origin.z = original_point.z();
    center_cloud->push_back(center_point_origin);
  }

  center_cloud->width = 1;
  center_cloud->height = center_cloud->points.size();
  }

  // 获取中间结果的点云
  PointCloudPtr getFilteredCloud() const { return filtered_cloud_; }
  PointCloudPtr getPlaneCloud() const { return plane_cloud_; }
  PointCloudPtr getAlignedCloud() const { return aligned_cloud_; }
  PointCloudPtr getEdgeCloud() const { return edge_cloud_; }
  PointCloudPtr getCenterZ0Cloud() const { return center_z0_cloud_; }
  std::vector<PointCloudPtr> getClusterClouds() const { return cluster_clouds_; }
};

typedef std::shared_ptr<LidarDetect> LidarDetectPtr;

#endif
