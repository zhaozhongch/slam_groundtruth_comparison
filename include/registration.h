#pragma once

#include "ros/ros.h"
#include <map>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>
#include "tf/transform_datatypes.h"
#include <iomanip>
#include <unistd.h> 

//synchronize data coming from slam and ground truth(gt)

class Registration{
public:
    Registration(ros::NodeHandle &n);
    void CalculateTransformationRegistration();
    void ApplyRegistration();
    void PubMarker();
    double ShowError();
    void set_slam_pose(const std::vector<geometry_msgs::PoseStamped>& slam_pose);
    void set_gt_pose(const std::vector<geometry_msgs::PoseStamped>& gt_pose);
    void set_match_count(const int match_count);
private:
    ros::Publisher pub_MA_;//pub_markerArray
    ros::Publisher pub_cost_;
    ros::NodeHandle nh_;
    double error_ = 0;
    int match_count_;
    bool regi_done_ = false;

    std::vector<geometry_msgs::PoseStamped> poses_gt_;
    std::vector<geometry_msgs::PoseStamped> poses_slam_;
    std::vector<geometry_msgs::PoseStamped> poses_slam_after_registration_;
    visualization_msgs::Marker marker1_, marker2_;

    geometry_msgs::Pose optimal_transformation_;

    geometry_msgs::Point get_average_3d_point(const std::vector<geometry_msgs::PoseStamped>& poses);
    Eigen::Matrix4d Point3D2MatrixForm_Q(geometry_msgs::Point point);
    Eigen::Matrix4d Point3D2MatrixForm_P(geometry_msgs::Point point);
    void set_marker_basics(visualization_msgs::Marker& marker);
    void set_marker_namespace(visualization_msgs::Marker& marker, std::string name);
    void set_marker_pose(visualization_msgs::Marker& marker, geometry_msgs::PoseStamped pose);
    void set_marker_id(visualization_msgs::Marker& marker, int id);
    void set_marker_color(visualization_msgs::Marker& marker, std::string color);
};