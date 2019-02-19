#pragma once

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>

class fst{
public:
    fst(ros::NodeHandle& nh);
    void slam_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msgs);
    void read_gt_pose_from_csv();
    void find();
    const inline std::vector<geometry_msgs::PoseStamped> get_samestamped_pose_groundtruth(){
        return same_gt;
    };
    const inline std::vector<geometry_msgs::PoseStamped> get_samestamped_pose_slam(){
        return same_slam;
    }
    bool keep_spin = true;
private:
    /**subscriber will subscribe the pose from ground truth and slam then try to find the msg that share the similar(smaller than 3 ms) timestamp**/
    ros::Subscriber sub_slam;
    std::vector<geometry_msgs::PoseStamped> all_gt, all_slam, same_gt, same_slam;
    std::string groundtruth_csv_file_address_;
    int do_registration_threshold;
    int msg_count = 0;
    double same_timestamp_threshold = 0.001; //1ms as offset threshold
};
