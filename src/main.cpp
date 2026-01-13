#define PCL_NO_PRECOMPILE
#define FLANN_NO_SERIALIZATION

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <iostream>
#include <string>
#include <filesystem>

#include "lidar_detect.hpp"

#include <pcl/registration/transformation_estimation_svd.h>

#include "io_utils.hpp"
#include "struct.hpp"
#include "print_utils.hpp"
#include "data_preprocess.hpp"
#include "qr_detect.hpp"
#include "utils.hpp"

// 定义命令行参数
DEFINE_string(cam_intrinsic_file, "", "Path to camera intrinsic parameters file");
DEFINE_string(data_dir, "", "Path to input data directory");
DEFINE_string(output_dir, "", "Path to output directory");
// ---------------- Parameters override flags ----------------
DEFINE_double(marker_size, -1.0, "Marker size in meters (default: use struct value)");
DEFINE_double(delta_width_qr_center, -1.0, "QR center delta width (m)");
DEFINE_double(delta_height_qr_center, -1.0, "QR center delta height (m)");
DEFINE_double(delta_width_circles, -1.0, "Circle delta width (m)");
DEFINE_double(delta_height_circles, -1.0, "Circle delta height (m)");
DEFINE_uint32(min_detected_markers, 0, "Minimum detected markers");
DEFINE_double(circle_radius, -1.0, "Circle radius (m)");
DEFINE_double(voxel_downsample_size, -1.0, "Voxel grid size (m)");
DEFINE_double(plane_dist_threshold, -1.0, "Plane distance threshold (m)");
DEFINE_double(circle_tolerance, -1.0, "Circle size tolerance (m)");

// Vector flags as CSV
DEFINE_string(crop_min_xyz, "", "Crop min xyz as CSV, e.g. \"3,-2,-2\"");
DEFINE_string(crop_max_xyz, "", "Crop max xyz as CSV, e.g. \"5,2.5,2\"");

static bool parseCSV3(const std::string &s, std::vector<double> &out)
{
    if (s.empty())
        return false;
    out.clear();

    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ','))
    {
        out.push_back(std::stod(item));
    }
    return out.size() == 3;
}

// 可选：添加参数验证函数
static bool ValidateDataDir(const char *flagname, const std::string &value)
{
    if (!value.empty())
    {
        return true;
    }
    std::cout << "Invalid value for --" << flagname << ": cannot be empty" << std::endl;
    return false;
}
DEFINE_validator(data_dir, &ValidateDataDir);

int main(int argc, char **argv)
{
    // 初始化 gflags
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    // set glog level to INFO
    FLAGS_minloglevel = google::GLOG_INFO;
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_logbufsecs = 0;
    LOG(INFO) << "Begin program";

    // 打印欢迎信息
    std::cout << "=== Program Configuration ===" << std::endl;
    std::cout << "Camera intrinsic file: " << FLAGS_cam_intrinsic_file << std::endl;
    std::cout << "Data directory: " << FLAGS_data_dir << std::endl;
    std::cout << "Output directory: " << FLAGS_output_dir << std::endl;
    std::cout << "=============================" << std::endl;

    // 检查必要参数
    if (FLAGS_cam_intrinsic_file.empty())
    {
        std::cout << "Warning: Camera intrinsic file not specified" << std::endl;
    }

    if (FLAGS_output_dir.empty())
    {
        std::cout << "Warning: Output directory not specified" << std::endl;
    }

    CameraIntrinsics cam_params;
    cam_params = readCameraIntrinsics(FLAGS_cam_intrinsic_file);
    printCameraIntrinsics(cam_params);

    Parameters params;
    params.camera_intrinsics = cam_params;
    // ---------------- Apply gflag overrides ----------------
    if (FLAGS_marker_size > 0)
        params.marker_size = FLAGS_marker_size;

    if (FLAGS_delta_width_qr_center > 0)
        params.delta_width_qr_center = FLAGS_delta_width_qr_center;

    if (FLAGS_delta_height_qr_center > 0)
        params.delta_height_qr_center = FLAGS_delta_height_qr_center;

    if (FLAGS_delta_width_circles > 0)
        params.delta_width_circles = FLAGS_delta_width_circles;

    if (FLAGS_delta_height_circles > 0)
        params.delta_height_circles = FLAGS_delta_height_circles;

    if (FLAGS_min_detected_markers > 0)
        params.min_detected_markers = FLAGS_min_detected_markers;

    if (FLAGS_circle_radius > 0)
        params.circle_radius = FLAGS_circle_radius;

    if (FLAGS_voxel_downsample_size > 0)
        params.voxel_downsample_size = FLAGS_voxel_downsample_size;

    if (FLAGS_plane_dist_threshold > 0)
        params.plane_dist_threshold = FLAGS_plane_dist_threshold;

    if (FLAGS_circle_tolerance > 0)
        params.circle_tolerance = FLAGS_circle_tolerance;

    // Vector overrides
    parseCSV3(FLAGS_crop_min_xyz, params.crop_min_xyz);
    parseCSV3(FLAGS_crop_max_xyz, params.crop_max_xyz);

    LOG(INFO) << "Final Parameters:";
    LOG(INFO) << " marker_size = " << params.marker_size;
    LOG(INFO) << " delta_width_qr_center = " << params.delta_width_qr_center;
    LOG(INFO) << " delta_height_qr_center = " << params.delta_height_qr_center;
    LOG(INFO) << " delta_width_circles = " << params.delta_width_circles;
    LOG(INFO) << " delta_height_circles = " << params.delta_height_circles;
    LOG(INFO) << " min_detected_markers = " << params.min_detected_markers;
    LOG(INFO) << " circle_radius = " << params.circle_radius;
    LOG(INFO) << " voxel_downsample_size = " << params.voxel_downsample_size;
    LOG(INFO) << " plane_dist_threshold = " << params.plane_dist_threshold;
    LOG(INFO) << " circle_tolerance = " << params.circle_tolerance;
    LOG(INFO) << " crop_min_xyz = ["
              << params.crop_min_xyz[0] << ", "
              << params.crop_min_xyz[1] << ", "
              << params.crop_min_xyz[2] << "]";
    LOG(INFO) << " crop_max_xyz = ["
              << params.crop_max_xyz[0] << ", "
              << params.crop_max_xyz[1] << ", "
              << params.crop_max_xyz[2] << "]";

    std::vector<InputDataInfo> all_input_data = listDataPairs(FLAGS_data_dir);
    std::cout << "Found " << all_input_data.size() << " .bag and .png file pairs in " << FLAGS_data_dir << std::endl;

    // 将所有帧的点云拼接到一起
    DataPreprocessPtr dataPreprocessPtr;
    dataPreprocessPtr = std::make_shared<DataPreprocess>(all_input_data);

    // 点云的圆心检测
    LidarDetectPtr lidarDetectPtr;
    lidarDetectPtr = std::make_shared<LidarDetect>(params);

    // QR码检测
    QRDetectPtr qrDetectPtr;
    qrDetectPtr = std::make_shared<QRDetect>(params);

    for (int data_idx = 0; data_idx < 1; data_idx++)
    {
        std::string data_pair_dir = FLAGS_output_dir + "/data_pair_" + std::to_string(data_idx);
        std::filesystem::create_directories(data_pair_dir);
        LOG(INFO) << "Processing data pair " << data_idx + 1 << " / " << all_input_data.size();

        // step1: lidar pointcloud detection -------------------------------------------------
        PointCloudPtr lidar_center_cloud = MAKE_POINTCLOUD();
        lidar_center_cloud->reserve(4);
        lidarDetectPtr->detect_mech_lidar(dataPreprocessPtr->cloud_inputs[data_idx], lidar_center_cloud);

        // save some lidar detection results
        std::string edge_cloud_file = data_pair_dir + "/edge_cloud_" + std::to_string(data_idx) + ".pcd";
        std::string center_cloud_file = data_pair_dir + "/center_cloud_" + std::to_string(data_idx) + ".pcd";
        std::string plane_cloud_file = data_pair_dir + "/plane_cloud_" + std::to_string(data_idx) + ".pcd";
        std::string center_z0_cloud_file = data_pair_dir + "/center_z0_cloud_" + std::to_string(data_idx) + ".pcd";
        pcl::io::savePCDFileASCII(edge_cloud_file, *lidarDetectPtr->getEdgeCloud());
        LOG(INFO) << "center_cloud_:";
        pcl::io::savePCDFileASCII(center_cloud_file, *lidar_center_cloud);
        LOG(INFO) << "center_cloud_:";
        pcl::io::savePCDFileASCII(plane_cloud_file, *lidarDetectPtr->getPlaneCloud());
        LOG(INFO) << "center_cloud_:";
        pcl::io::savePCDFileASCII(center_z0_cloud_file, *lidarDetectPtr->getCenterZ0Cloud());
        LOG(INFO) << "center_cloud_:";

        std::vector<PointCloudPtr> cluster_clouds = lidarDetectPtr->getClusterClouds();
        for (size_t i = 0; i < cluster_clouds.size(); i++)
        {
            std::string cluster_cloud_file = data_pair_dir + "/cluster_cloud_" + std::to_string(data_idx) + "_" + std::to_string(i) + ".pcd";
            pcl::io::savePCDFileASCII(cluster_cloud_file, *cluster_clouds[i]);
        }

        LOG(INFO) << "Lidar Detection completed ";

        // step2: qr code detection -------------------------------------------------
        PointCloudPtr qr_center_cloud = MAKE_POINTCLOUD();
        qrDetectPtr->detect_qr(dataPreprocessPtr->img_inputs[data_idx], qr_center_cloud);
        std::string qr_img_file = data_pair_dir + "/qr_img_" + std::to_string(data_idx) + ".png";
        cv::imwrite(qr_img_file, qrDetectPtr->imageCopy_);
        LOG(INFO) << "QR Detection completed ";

        // step3: estimate pose from lidar points and points in camera coordinate --------
        // 对QR和LiDAR检测到的圆心进行排序（得到点对关系）
        PointCloudPtr qr_centers = MAKE_POINTCLOUD();
        PointCloudPtr lidar_centers = MAKE_POINTCLOUD();
        sortPatternCenters(lidar_center_cloud, lidar_centers, "lidar");
        sortPatternCenters(qr_center_cloud, qr_centers, "camera");
        std::string sort_centers_lidar_file = data_pair_dir + "/sort_centers_lidar_" + std::to_string(data_idx) + ".pcd";
        std::string sort_centers_qr_file = data_pair_dir + "/sort_centers_qr_" + std::to_string(data_idx) + ".pcd";
        pcl::io::savePCDFileASCII(sort_centers_lidar_file, *lidar_centers);
        pcl::io::savePCDFileASCII(sort_centers_qr_file, *qr_centers);

        // 直接使用SVD计算求解变换矩阵
        Eigen::Matrix4f transformation;
        pcl::registration::TransformationEstimationSVD<PointT, PointT> svd;
        svd.estimateRigidTransformation(*lidar_centers, *qr_centers, transformation);

        std::cout << transformation;

        // 计算RMSE误差
        PointCloudPtr transformed_cloud = MAKE_POINTCLOUD();
        pcl::transformPointCloud(*lidar_centers, *transformed_cloud, transformation);
        double rmse = computeRMSE(qr_centers, transformed_cloud);
        LOG(INFO) << "Calibration result RMSE: " << rmse;

        // 使用标定结果对点云进行染色
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        PointCloudPtr cloud_input_downsample = MAKE_POINTCLOUD();
        cloud_input_downsample = dataPreprocessPtr->getPointCloud(data_idx, params.voxel_downsample_size);
        colorPointCloudWithImage(cloud_input_downsample, transformation, cam_params.getCameraMatrix(), cam_params.getDistCoeffs(), dataPreprocessPtr->img_inputs[data_idx], colored_cloud);
        std::string colored_cloud_file = data_pair_dir + "/colored_cloud_" + std::to_string(data_idx) + ".pcd";
        pcl::io::savePCDFileASCII(colored_cloud_file, *colored_cloud);

        LOG(INFO) << "Processed data pair " << data_idx + 1 << " / " << all_input_data.size() << " successfully" << std::endl;
    }

    std::cout << "Processing completed!" << std::endl;

    return 0;
}
