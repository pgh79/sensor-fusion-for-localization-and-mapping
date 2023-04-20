/*
 * @Description: LIO localization frontend workflow, implementation
 * @Author: Ge Yao
 * @Date: 2021-01-02 10:47:23
 */
#include "lidar_localization/matching/front_end/matching_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

MatchingFlow::MatchingFlow(ros::NodeHandle& nh) {
    //
    // subscribers:
    // 
    // a. undistorted Velodyne measurement: 
    // 创建类CloudSubscriber的指针cloud_sub_ptr_，并实例化。
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
    // b. lidar pose in map frame:
    gnss_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);

    //
    // publishers:
    // 
    // a. global point cloud map:
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
    // b. local point cloud map:
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);
    // c. current scan:
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "/map", 100);
    
    // d. estimated lidar pose in map frame, lidar frontend:
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_odometry", "/map", "/lidar", 100);
    // e. estimated lidar pose in map frame, map matching:
    map_matching_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/map_matching_odometry", "/map", "/lidar", 100);

    matching_ptr_ = std::make_shared<Matching>();
}

bool MatchingFlow::Run() {
    // update global map if necessary:
    /*
        starts by checking if the global map has been updated.
    If it has a new global map, then the code will publish the new global map to all subscribers.
    Next, it checks if there is a subscriber. if trur, the Matching class use GetGlobalMap() method to get new global map, the global_map_pub_ptr_ publish global map.
    */ 
    if (
        matching_ptr_->HasNewGlobalMap() && 
        global_map_pub_ptr_->HasSubscribers()
    ) {
        global_map_pub_ptr_->Publish( matching_ptr_->GetGlobalMap() );
    }

// same as global map, update local map
    // update local map if necessary:
    if (
        matching_ptr_->HasNewLocalMap() && 
        local_map_pub_ptr_->HasSubscribers()
    ) {
        local_map_pub_ptr_->Publish( matching_ptr_->GetLocalMap() );
    }

    // read inputs:
    ReadData();

// verifies the data is not empty and is valid, otherwise skip matching and continue to do nothing
    while( HasData() ) {
        if (!ValidData()) {
            LOG(INFO) << "Invalid data. Skip matching" << std::endl;
            continue;
        }

// publish data if matching update succeeded
        if ( UpdateMatching() ) {
            PublishData();
        }
    }

    return true;
}

/*
starts by reading the lidar measurements and reference pose into a buffer.
 The code then returns true, indicating that it has successfully read data from the pipe.
*/

bool MatchingFlow::ReadData() {
    // pipe lidar measurements and reference pose into buffer:
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);

    return true;
}

/*
starts by checking if the cloud data buffer is empty. 
 If it is, then the function returns false.

 it checks if the matching pointer has been initialized.
 If it has not been initialized, then the function returns true.
 
 Finally, it checks to see if there are any GNSS data buffers in memory and 
 returns true or false depending on whether they are empty or not respectively.
*/
bool MatchingFlow::HasData() {
    if ( cloud_data_buff_.empty() )
        return false;
    
    if ( matching_ptr_->HasInited() )
        return true;
    
    if ( gnss_data_buff_.empty() )
        return false;
        
    return true;
}

/*

*/
bool MatchingFlow::ValidData() {
// current_cloud_data_ stores the cloud buffer's first element
    current_cloud_data_ = cloud_data_buff_.front();

// verifies whether the matching object reference is initialized 
// if so, pop off the first element of cloud data buff off the stack
// and clear gnss data buffer
// then return true

    if  ( matching_ptr_->HasInited() ) {
        cloud_data_buff_.pop_front();
        gnss_data_buff_.clear();
        return true;
    }

// if map is not initialized, current_gnss_data_ stores the first element of gnss_data_buff_ ??
    current_gnss_data_ = gnss_data_buff_.front();

/*
the following lines are checking the diff time between stored cloud data time and stored gnss data time, 
and check whether it is between -0.05 ~ 0.05 as valid data
if so , return true or false
then gnss and cloud buffer pop off the first element off the stack
*/
    double diff_time = current_cloud_data_.time - current_gnss_data_.time;

    //
    // here assumes that the freq. of lidar measurements is 10Hz:
    //
    if (diff_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

/*
    starts by checking if the matching_ptr has been initialized.
 If not, it will try to initialize using a scan context query.
    @param: current_cloud_data_
    passed to : SetScanContextPose
    if it worked, then call GetInitPose(), no param is passed. 
    Matrix4f init_pose is created to store the output.

    evaluate deviation from gnss/imu（measured）:
    1. pose diff between init_pose and current_gnss_data_.pose
    2. norm()

    scan Context succeeded. printf deviation;

    If scan context failed to set, then fall back to set GNSS/IMU pose which call SetGNSSPose passing current_gnss_data_.pose;
*/
bool MatchingFlow::UpdateMatching() {
    if (!matching_ptr_->HasInited()) {
        // first try to init using scan context query:
        if (
            matching_ptr_->SetScanContextPose(current_cloud_data_)
        ) {
            Eigen::Matrix4f init_pose = matching_ptr_->GetInitPose();

            // evaluate deviation from GNSS/IMU:
            float deviation = (
                init_pose.block<3, 1>(0, 3) - current_gnss_data_.pose.block<3, 1>(0, 3)
            ).norm();

            // prompt:
            LOG(INFO) << "Scan Context Localization Init Succeeded. Deviation between GNSS/IMU: " 
                      << deviation
                      << std::endl;
        } 
        // if failed, fall back to GNSS/IMU init:
        else {
            matching_ptr_->SetGNSSPose(current_gnss_data_.pose);

            LOG(INFO) << "Scan Context Localization Init Failed. Fallback to GNSS/IMU." 
                      << std::endl;
        }
    }

// returns the return value (perhaps bool value) of Update
// input current_cloud_data_, whether successfully updated laser_odometry_, map_matching_odometry_
    return matching_ptr_->Update(
        current_cloud_data_, 
        laser_odometry_, map_matching_odometry_
    );
}

// current_cloud_data_: pcl library, with timestamp, xyz, intensity etc.
// How to sync timestamp??  Set a standard time?

// if the UpdateMatching succeeded, then Publish timestap_synced and updated laser_odometry_, map_matching_odometry_
// Also publish the output of GetCurrentScan()

bool MatchingFlow::PublishData() {
    const double &timestamp_synced = current_cloud_data_.time;

    laser_odom_pub_ptr_->Publish(laser_odometry_, timestamp_synced);
    map_matching_odom_pub_ptr_->Publish(map_matching_odometry_, timestamp_synced);

    current_scan_pub_ptr_->Publish(matching_ptr_->GetCurrentScan());

    return true;
}

} // namespace lidar_localization