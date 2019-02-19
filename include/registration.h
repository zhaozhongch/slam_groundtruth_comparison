#pragma once

#include "ros/ros.h"
#include <map>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include "tf/transform_datatypes.h"
#include <iomanip>
#include <unistd.h> 

//synchronize data coming from slam and ground truth(gt)

class registration{
public:
    registration(ros::NodeHandle &n);
    void calculate_transformation_registration();
    void apply_registration();
    void pub_marker();
    void set_slam_pose(const std::vector<geometry_msgs::PoseStamped>& slam_pose);
    void set_gt_pose(const std::vector<geometry_msgs::PoseStamped>& gt_pose);
    double show_error();
    std::string visualization_choice_;
private:
    ros::Publisher pub_MA;//pub_markerArray
    ros::NodeHandle nh;
    double error = 0;
    int do_registration_threshold;
    int syn_count = 0;
    bool regi_done = false;

    std::vector<geometry_msgs::PoseStamped> poses_gt;
    std::vector<geometry_msgs::PoseStamped> poses_slam;
    std::vector<geometry_msgs::PoseStamped> poses_slam_after_registration;
    visualization_msgs::Marker marker1, marker2;

    geometry_msgs::Pose optimal_transformation;

    geometry_msgs::Point getAverage3Dpoint(const std::vector<geometry_msgs::PoseStamped>& poses);
    Eigen::Matrix4d point3D2MatrixForm_Q(geometry_msgs::Point point);
    Eigen::Matrix4d point3D2MatrixForm_P(geometry_msgs::Point point);
    void set_marker_basics(visualization_msgs::Marker& marker);
    void set_marker_namespace(visualization_msgs::Marker& marker, std::string name);
    void set_marker_pose(visualization_msgs::Marker& marker, geometry_msgs::PoseStamped pose);
    void set_marker_id(visualization_msgs::Marker& marker, int id);
    void set_marker_color(visualization_msgs::Marker& marker, std::string color);
};