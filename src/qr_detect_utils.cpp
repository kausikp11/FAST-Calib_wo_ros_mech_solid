#include "qr_detect_utils.hpp"

#include <pcl/common/centroid.h>

void sortPatternCenters(PointCloudPtr in_pc, PointCloudPtr out_v,
                        const std::string &axis_mode)
{
  if (in_pc->size() != 4)
  {
    std::cerr << "[sortPatternCenters] Number of " << axis_mode
              << " center points to be sorted is not 4." << std::endl;
    return;
  }

  PointCloudPtr work_pc = MAKE_POINTCLOUD();

  // Coordinate transformation (LiDAR -> Camera)
  if (axis_mode == "lidar")
  {
    for (const auto &p : *in_pc)
    {
      PointT pt;
      pt.x = -p.y; // LiDAR Y -> Cam -X
      pt.y = -p.z; // LiDAR Z -> Cam -Y
      pt.z = p.x;  // LiDAR X -> Cam Z
      work_pc->push_back(pt);
    }
  }
  else
  {
    *work_pc = *in_pc;
  }

  // --- Sorting based on the local coordinate system of the pattern ---
  // 1. Calculate the centroid of the points
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*work_pc, centroid);
  // PointT ref_origin(centroid[0], centroid[1], centroid[2]);
  PointT ref_origin;
  ref_origin.x = centroid[0];
  ref_origin.y = centroid[1];
  ref_origin.z = centroid[2];
  // If you care about ring:
  ref_origin.ring = 0; // or some meaningful value

  // 2. Project points to the XY plane relative to the centroid and calculate
  // angles
  std::vector<std::pair<float, int>> proj_points;
  for (size_t i = 0; i < work_pc->size(); ++i)
  {
    const auto &p = work_pc->points[i];
    Eigen::Vector3f rel_vec(p.x - ref_origin.x, p.y - ref_origin.y,
                            p.z - ref_origin.z);
    proj_points.emplace_back(atan2(rel_vec.y(), rel_vec.x()), i);
  }

  // 3. Sort points based on the calculated angle
  std::sort(proj_points.begin(), proj_points.end());

  // 4. Output the sorted points into the result vector 'out_v'
  out_v->resize(4);
  for (int i = 0; i < 4; ++i)
  {
    (*out_v)[i] = work_pc->points[proj_points[i].second];
  }

  // 5. Verify the order (ensure it's counter-clockwise) and fix if necessary
  const auto &p0 = out_v->points[0];
  const auto &p1 = out_v->points[1];
  const auto &p2 = out_v->points[2];
  Eigen::Vector3f v01(p1.x - p0.x, p1.y - p0.y, 0);
  Eigen::Vector3f v12(p2.x - p1.x, p2.y - p1.y, 0);
  if (v01.cross(v12).z() > 0)
  {
    std::swap((*out_v)[1], (*out_v)[3]);
  }

  // 6. If the original input was in the lidar frame, transform the sorted
  // points back
  if (axis_mode == "lidar")
  {
    for (auto &point : out_v->points)
    {
      float x_new = point.z;  // Cam Z -> LiDAR X
      float y_new = -point.x; // Cam -X -> LiDAR Y
      float z_new = -point.y; // Cam -Y -> LiDAR Z
      point.x = x_new;
      point.y = y_new;
      point.z = z_new;
    }
  }
}

