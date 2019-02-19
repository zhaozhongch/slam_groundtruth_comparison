#include <iostream>
#include "registration.h"
#include "find_same_timestamp.h"
#include "std_msgs/Float64.h"

/****
 * name gt stands for the groundtruth
 ****/

// void f64Callback(const std_msgs::Float64::ConstPtr& msgs){
//     ROS_INFO("I got %f", msgs->data);
// }

int main(int nargs, char *args[]){

    ros::init(nargs,args,"syn_slam_gt");

    ros::NodeHandle nh;

    //ros::Subscriber sub_test = nh.subscribe("test", 100, f64Callback);

    fst match(nh);
    ros::Rate r(500);
    bool keep_spin = true;
    while(keep_spin){
        keep_spin = match.keep_spin; //when we get the required number of keyframe pose, we'll stop spin
        ros::spinOnce();
        r.sleep();
    }

    match.read_gt_pose_from_csv();

    match.find(); // find the poses from groundtruth and slam that share the same timestamp 

    registration regi(nh);

    regi.set_slam_pose(match.get_samestamped_pose_slam());
    regi.set_gt_pose(match.get_samestamped_pose_groundtruth());

    regi.calculate_transformation_registration();
    regi.apply_registration();

    if(regi.visualization_choice_.compare(std::string("visualize")) == 0)
        regi.pub_marker();
    regi.show_error();
}