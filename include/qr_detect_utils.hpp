#pragma once
#include "struct.hpp"

#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>


void sortPatternCenters(PointCloudPtr pc,
                        PointCloudPtr v,
                        const std::string& axis_mode = "camera");