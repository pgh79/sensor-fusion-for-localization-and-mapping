/*
 * @Description: frontend node for lio localization
 * @Author: Ge Yao
 * @Date: 2021-01-02 10:47:23
 */
/*
*The code is a front-end node for the lio localization library.
 *It subscribes to undistorted lidar measurements, GNSS position, and publishes relative pose estimation and map matching estimation.
*/

#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/matching/front_end/matching_flow.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[]) {
//starts by initializing the logging system.
    google::InitGoogleLogging(argv[0]);
//sets up a few flags for the program to run with.
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

//ros::init() is called and passed in argc and argv as parameters.
    ros::init(argc, argv, "lio_matching_node");
//This function creates an instance of ros::NodeHandle which will be used throughout the rest of this code snippet.
    ros::NodeHandle nh;

    // subscribe to:
    //     a. undistorted lidar measurements
    //     b. GNSS position

    // publish:
    //     a. relative pose estimation
    //     b. map matching estimation
    // this provides input to sliding window backend

//a subscription to undistorted lidar measurements is set up on nh (the NodeHandle created earlier).
//Then publishing relative pose estimation and map matching estimation are done on matching_flow_ptr (a shared pointer 
//that was made from nh) using Run().
    std::shared_ptr<MatchingFlow> matching_flow_ptr = std::make_shared<MatchingFlow>(nh);


//rate() is called every time while looping through until all data has been processed or an error occurs.
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        matching_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}