//
// Created by 廖礼洲 on 2022/1/12.
//

#ifndef LIO_SAM_SAVEPOINTANDIMAGE_H
#define LIO_SAM_SAVEPOINTANDIMAGE_H

#include <fstream>
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

typedef pcl::PointXYZI PointType;

class SavePointAndImage {
public:
    // 声明储存文件的文件夹
    std::string save_directory;
    std::string ScanDirectory;
    std::string ImageDirectory;



    int curr_node_idx = 0;


    float scan_period = 0.1;
    double timeLaser;
    double timeImage;


public:
    SavePointAndImage();
    // callback

};


#endif //LIO_SAM_SAVEPOINTANDIMAGE_H
