/*
 * @Description: ROS node for sensor measurement pre-processing
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */
 /*
 *subscribes to the cloud topic and then pre-processes the raw data from a Velodyne sensor.
 */
 // including the necessary libraries.
#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[]) {
// initializes Google's logging library.
    google::InitGoogleLogging(argv[0]);
// creates a variable for logging to use, and sets it to the path of where logs will be saved.
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
// sets some flags that are used in debugging.
    FLAGS_alsologtostderr = 1;

//starts the node with ROS (Robot Operating System).
    ros::init(argc, argv, "data_pretreat_node");
//create a variable for our NodeHandle object which is what we'll use to interact with other nodes on our network.
    ros::NodeHandle nh;

//set up an input parameter called "cloud_topic" which will be used as a topic name when publishing data from this 
//node later on in order to make sure that all messages sent from this node are received by other nodes on our network so they can process them accordingly.
    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");


    // subscribe to
    // a. raw Velodyne measurement
    // b. raw GNSS/IMU measurement
    // publish
    // a. undistorted Velodyne measurement
    // b. lidar pose in map frame

//subscribing and publishing messages using DataPretreatFlow objects which are created using nh as their handle and cloud_topic as their topic name respectively.
    std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::make_shared<DataPretreatFlow>(nh, cloud_topic);

//start processing raw Velodyne measurements at 100Hz while waiting for ROS to respond back with success or failure before continuing onto the next loop iteration 
//until there's no more work left to do (which means ros::ok() returns false).
    // pre-process lidar point cloud at 100Hz:
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        data_pretreat_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}