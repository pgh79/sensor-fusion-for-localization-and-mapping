/*
 * @Description: lio localization backend workflow, implementation
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
 //  implementation for the sliding window flow.
#include "lidar_localization/matching/back_end/sliding_window_flow.hpp"

#include "glog/logging.h"

// include the file manager header file which is needed to manage files on disk.
#include "lidar_localization/tools/file_manager.hpp"
// include global definition header file with its default settings
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

// Starts by declaring a variable of type ros::NodeHandle. nh as the name of the network handle
// This is the NodeHandle that will be passed to the constructor of this class.
SlidingWindowFlow::SlidingWindowFlow(
    ros::NodeHandle& nh
) {
    //
    // subscribers:
    //
    // a. lidar odometry: 
 // declares a variable called laser_odom_sub_ptr_. This is an instance of std::make_shared, which will be passed to the constructor of this class.
 // The constructor for this object takes three parameters
    // creating an instance of OdometrySubscriber with a topic name "/laser_odom" and buffer size 100000 (the default). 
    // receive updates on its state. NetworkHandle::Name(), Path(), MessageSize(). 
    laser_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/laser_odometry", 100000);
    // b. map matching odometry:
    map_matching_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/map_matching_odometry", 100000);
    // c. IMU measurement, for pre-integration:
    // IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    imu_raw_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu/extract", 1000000);
// "/synced_imu" as the path, 100000 as the number of bytes per message sent
// After this call returns true or false if successful or not respectively, we assign std::make_shared to imu_synced_sub_ptr_.
    imu_synced_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/synced_imu", 100000);
    // d. GNSS position:
    gnss_pose_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);

    // 
    //  publishers:
    // 
    // a. current lidar key frame:(key database)
    key_frame_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh, "/key_frame", "/map", 100);
    // b. current reference GNSS frame:
    key_gnss_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh, "/key_gnss", "/map", 100);
    // c. optimized odometry:
    /* OdometryPublisher(ros::NodeHandle& nh, 
                      std::string topic_name, 
                      std::string base_frame_id,
                      std::string child_frame_id,
                      int buff_size);
    OdometryPublisher() = default;
    */
    optimized_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/optimized_odometry", "/map", "/lidar", 100);
    // d. optimized trajectory:
    optimized_trajectory_pub_ptr_ = std::make_shared<KeyFramesPublisher>(nh, "/optimized_trajectory", "/map", 100);
    // e. lidar frame
    // TFBroadCaster(std::string frame_id, std::string child_frame_id);
    laser_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("/map", "/velo_link");

    //
    // backend:
    //
    /*
        SlidingWindow::SlidingWindow() {
            InitWithConfig();
        }    
    */ 
    sliding_window_ptr_ = std::make_shared<SlidingWindow>();
}

bool SlidingWindowFlow::Run() {
    // load messages into buffer:
    if ( !ReadData() )
        return false;
    
    while( HasData() ) {
    // make sure all the measurements are synced:
        if ( !ValidData() )
            continue;

        UpdateBackEnd();
        PublishData();
    }

    return true;
}

bool SlidingWindowFlow::SaveOptimizedTrajectory() {
    sliding_window_ptr_ -> SaveOptimizedTrajectory();

    return true;
}

bool SlidingWindowFlow::ReadData() {
    // a. lidar odometry:
/*
* void ParseData(std::deque<CloudData>& deque_cloud_data);
*/
    laser_odom_sub_ptr_->ParseData(laser_odom_data_buff_);
    // b. map matching odometry:
    map_matching_odom_sub_ptr_->ParseData(map_matching_odom_data_buff_);
    // c. IMU measurement, for pre-integration:
    imu_raw_sub_ptr_->ParseData(imu_raw_data_buff_);
    imu_synced_sub_ptr_->ParseData(imu_synced_data_buff_);
    // d. GNSS position:
    gnss_pose_sub_ptr_->ParseData(gnss_pose_data_buff_);

    return true;
}

/*
* checks if there is any data in the buffer before continuing with reading more data.
* If there is no data, then it will return false and stop reading from the buffer.
* The while loop will continue to check if there is any data in the buffer until it finds some or until it reaches an error condition.
*/
bool SlidingWindowFlow::HasData() {
    if (
        laser_odom_data_buff_.empty() ||
        map_matching_odom_data_buff_.empty() ||
        imu_synced_data_buff_.empty() ||
        gnss_pose_data_buff_.empty() 
    ) {
        return false;
    }

    return true;
}

/*
* return true or false depending on whether or not there was valid information in one of these buffers 
* that can be used for calculating an estimate of where we are located at this moment in time.
*/
bool SlidingWindowFlow::ValidData() {
    current_laser_odom_data_ = laser_odom_data_buff_.front();
    current_map_matching_odom_data_ = map_matching_odom_data_buff_.front();
    current_imu_data_ = imu_synced_data_buff_.front();
    current_gnss_pose_data_ = gnss_pose_data_buff_.front();

    double diff_map_matching_odom_time = current_laser_odom_data_.time - current_map_matching_odom_data_.time;
    double diff_imu_time = current_laser_odom_data_.time - current_imu_data_.time;
    double diff_gnss_pose_time = current_laser_odom_data_.time - current_gnss_pose_data_.time;

    if ( diff_map_matching_odom_time < -0.05 || diff_imu_time < -0.05 || diff_gnss_pose_time < -0.05 ) {
        laser_odom_data_buff_.pop_front();
        return false;
    }

    if ( diff_map_matching_odom_time > 0.05 ) {
        map_matching_odom_data_buff_.pop_front();
        return false;
    }

    if ( diff_imu_time > 0.05 ) {
        imu_synced_data_buff_.pop_front();
        return false;
    }

    if ( diff_gnss_pose_time > 0.05 ) {
        gnss_pose_data_buff_.pop_front();
        return false;
    }

// buff popped the first element off stack, and return true to show that current_laser_odom_data_ can be used.
    laser_odom_data_buff_.pop_front();
    map_matching_odom_data_buff_.pop_front();
    imu_synced_data_buff_.pop_front();
    gnss_pose_data_buff_.pop_front();

    return true;
}

/*
* checking to see if the IMU data buffer is empty.
 If it is, then we know that there are no more updates and it's time for us to update our state.
 1. imu raw data buffer is not empty && 2. imu raw buff's first element's time is less than current imu data's time
 && 3.  We do this by calling UpdateIMUPreIntegration() on the sliding_window_ptr object.
 This function will return true if all of the IMU data has been processed, or false otherwise.
*/ 

    //
    // backend:
    //
//    std::shared_ptr<SlidingWindow> sliding_window_ptr_;
/*
class SlidingWindow {
  public:
    SlidingWindow();

    bool UpdateIMUPreIntegration(const IMUData &imu_data);
    ...
    }
*/
/*
class SlidingWindowFlow {
public:
    SlidingWindowFlow(ros::NodeHandle& nh);

    bool Run();
    bool SaveOptimizedTrajectory();
    
  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateIMUPreIntegration(void);
    ...
    }
*/
bool SlidingWindowFlow::UpdateIMUPreIntegration(void) {
    while (
        !imu_raw_data_buff_.empty() && 
        imu_raw_data_buff_.front().time < current_imu_data_.time && 
        sliding_window_ptr_->UpdateIMUPreIntegration(imu_raw_data_buff_.front())
    ) {
        imu_raw_data_buff_.pop_front();
    }

    return true;
}

bool SlidingWindowFlow::UpdateBackEnd() {
/*
starts by initializing the odometry state.
 If it is not initialized, then a new pose will be calculated and stored in odom_init_pose.
 The current laser position data is multiplied with the inverse of the odometry init pose to get the current lidar odometry frame in map frame.
*/
    static bool odometry_inited = false;
    static Eigen::Matrix4f odom_init_pose = Eigen::Matrix4f::Identity();

    if ( !odometry_inited ) {
        // the origin of lidar odometry frame in map frame as init pose: 
        // lidar_odom_data_.pose projection in odom_init_pose, this is in gnss map frame
        odom_init_pose = current_gnss_pose_data_.pose * current_laser_odom_data_.pose.inverse();

        odometry_inited = true;
    }
    
    // update IMU pre-integration:
    UpdateIMUPreIntegration();
    
    // current lidar odometry in map frame:
    current_laser_odom_data_.pose = odom_init_pose * current_laser_odom_data_.pose;

    // optimization is carried out in map frame:
    return sliding_window_ptr_->Update(
        current_laser_odom_data_, 
        current_map_matching_odom_data_,
        current_imu_data_,
        current_gnss_pose_data_
    );
}

bool SlidingWindowFlow::PublishData() {
    if ( sliding_window_ptr_->HasNewKeyFrame() ) {        
        KeyFrame key_frame;

        sliding_window_ptr_->GetLatestKeyFrame(key_frame);
        key_frame_pub_ptr_->Publish(key_frame);

        sliding_window_ptr_->GetLatestKeyGNSS(key_frame);
        key_gnss_pub_ptr_->Publish(key_frame);
    }

    if ( sliding_window_ptr_->HasNewOptimized() ) {
        KeyFrame key_frame;
        sliding_window_ptr_->GetLatestOptimizedOdometry(key_frame);
        optimized_odom_pub_ptr_->Publish(key_frame.pose, key_frame.time);

        // publish lidar TF:
        laser_tf_pub_ptr_->SendTransform(key_frame.pose, key_frame.time);
    }

    return true;
}

} // namespace lidar_localization