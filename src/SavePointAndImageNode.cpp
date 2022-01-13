//
// Created by 廖礼洲 on 2022/1/13.
//
#include "SavePointAndImage.h"

std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
std::queue<sensor_msgs::ImageConstPtr> ImageBuf;

SavePointAndImage SPII;
std::mutex mutex_lock;

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

void imageCloudHandler(const sensor_msgs::ImageConstPtr &ImageMsg)
{
    mutex_lock.lock();
    ImageBuf.push(ImageMsg);
    mutex_lock.unlock();
}

void SPI() {
    while(1) {
        while (!pointCloudBuf.empty() && !ImageBuf.empty()) {
            //进行时间同步
            mutex_lock.lock();

            while (!ImageBuf.empty() && ImageBuf.front()->header.stamp.toSec() < pointCloudBuf.front()->header.stamp.toSec()){
                ImageBuf.pop();
            }

            if (ImageBuf.empty()) {
                mutex_lock.unlock();
                break;
            }

            //时间
            SPII.timeLaser = pointCloudBuf.front()->header.stamp.toSec();
            SPII.timeImage = ImageBuf.front()->header.stamp.toSec();
            if (abs(SPII.timeLaser - SPII.timeImage) > 2) {
                ROS_WARN("time stamp unaligned error, pls check your data");
                break;
            }
            // 将当前点云数据储存到*thisKeyFrame中
            pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *thisKeyFrame);
            pointCloudBuf.pop();
            // 将当前帧的点云数据储存到cv_ptr
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(ImageBuf.front(), sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            mutex_lock.unlock();

            if (SPII.curr_node_idx <= 1000) {
                std::string curr_node_idx_str = std::to_string(SPII.curr_node_idx);
                pcl::io::savePCDFileBinary(SPII.ScanDirectory + curr_node_idx_str + ".pcd", *thisKeyFrame); // scan
                cv::imwrite(SPII.ImageDirectory + curr_node_idx_str + ".jpg", cv_ptr->image);
                std::cout << "当前输出第 " << SPII.curr_node_idx << " 帧" << std::endl;
                SPII.curr_node_idx++;
            }
        }
        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "SavePointAndImage");



    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud = nh.subscribe <sensor_msgs::PointCloud2> ("/velodyne_points",100, laserCloudHandler);
    ros::Subscriber subImageCloud = nh.subscribe <sensor_msgs::Image> ("/cam03/image_raw",100, imageCloudHandler);

    std::thread savepointimage {SPI};

    ROS_INFO("\033[1;32m----> Save PointCloud and Images Started.\033[0m");

    //callback多线程
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}