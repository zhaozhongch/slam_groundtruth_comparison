#pragma once

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>

class FST{
public:
    FST(ros::NodeHandle& nh);
    void SlamPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msgs);
    void ReadGTPoseFromCsv();
    void Find();
    void CheckTime();
    const inline std::vector<geometry_msgs::PoseStamped> get_samestamped_pose_groundtruth(){
        return same_gt_;
    };
    const inline std::vector<geometry_msgs::PoseStamped> get_samestamped_pose_slam(){
        return same_slam_;
    }
    const inline int get_match_count(){
        return match_count_;
    }
    bool keep_spin_ = true;
private:
    /**subscriber will subscribe the pose from ground truth and slam then try to find the msg that share the similar(smaller than 3 ms) timestamp**/
    ros::Subscriber sub_slam_;
    std::vector<geometry_msgs::PoseStamped> all_gt_, all_slam_, same_gt_, same_slam_ ;
    std::string groundtruth_csv_file_address_;
    int do_registration_threshold_; 
    int msg_count_ = 0;
    int msg_count_before_ = 0;
    int match_count_ = 0;
    double same_timestamp_threshold_ = 0.003; //1ms as offset threshold
    double last_timestamp_msg_received_ = 0;
    double max_wait_time_ = 8.0;
    double time_no_new_msg = 0.0;
};
