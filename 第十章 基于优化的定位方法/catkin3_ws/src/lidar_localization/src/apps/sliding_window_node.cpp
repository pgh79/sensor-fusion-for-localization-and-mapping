/*
 * @Description: backend node for lio localization
 * @Author: Ge Yao
 * @Date: 2021-01-02 10:47:23
 */
#include <ros/ros.h>

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/matching/back_end/sliding_window_flow.hpp"
#include "lidar_localization/saveOdometry.h"

#include "glog/logging.h"

using namespace lidar_localization;

bool save_odometry = false;

bool SaveOdometryCb(saveOdometry::Request &request, saveOdometry::Response &response) {
    save_odometry = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char *argv[]) {
// starts by initializing the logging library.
    google::InitGoogleLogging(argv[0]);
// sets up a variable for the directory where logs will be saved and another variable to log to stderr.
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

//creates an instance of ros::init which is used to initialize ROS.
// sets up a ROS node to subscribe to and publish messages from the lidar odometry and map matching odometry topics.
    ros::init(argc, argv, "sliding_window_node");
// creates a NodeHandle object that will be used throughout the program.
    ros::NodeHandle nh;

/*
*subscribes to: a) lidar odometry, which is published by ROS in real-time; 
*b) map matching odometry, which is published by ROS every time it detects changes in the environment.
*/
    //
    // subscribe to:
    // 
    // a. lidar odometry
    // b. map matching odometry

    //
    // publish:
    // 
    // a. optimized key frame sequence as trajectory
//publishes an optimized key frame sequence as trajectory with a shared pointer of type SlidingWindowFlow
    std::shared_ptr<SlidingWindowFlow> sliding_window_flow_ptr = std::make_shared<SlidingWindowFlow>(nh);

    // register service for optimized trajectory save:
//creating a service that will be called when the user use the save_odometry command
    ros::ServiceServer service = nh.advertiseService("save_odometry", SaveOdometryCb);

// creates a rate object and starts looping while waiting for ROS to respond.
    ros::Rate rate(10);
    while (ros::ok()) {
// call all the callbacks waiting to be called at that point in time.
//  If ROS responds, it means that there is an action to take so the code calls sliding_window_flow_ptr->Run() 
// which will run all of its functions in order until it reaches SaveOptimizedTrajectory().
        ros::spinOnce();

        sliding_window_flow_ptr->Run();
// If save_odometry was set to true, then the sliding window flow pointer would call sliding window flow ptr->SaveOptimizedTrajectory() 
// and if not, it would continue running until it finishes its job or reaches its limit (10).
        if (save_odometry) {
            save_odometry = false;
            sliding_window_flow_ptr->SaveOptimizedTrajectory();
        }

        rate.sleep();
    }

    return EXIT_SUCCESS;
}