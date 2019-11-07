#include <iostream>
#include "registration.h"
#include "find_same_timestamp.h"
#include "std_msgs/Float64.h"

/****
 * name gt stands for the groundtruth
 ****/

int main(int nargs, char *args[]){

    ros::init(nargs,args,"syn_slam_gt");

    ros::NodeHandle nh;

    FST match(nh);
    ros::Rate r(500);
    bool keep_spin = true;
    while(keep_spin && ros::ok()){
        match.CheckTime();
        keep_spin = match.keep_spin_; //when we get the required number of keyframe pose, we'll stop spin
        ros::spinOnce();
        r.sleep();
    }

    if(!ros::ok()){
        ROS_INFO("exit...........");
        return 0;
    }

    match.ReadGTPoseFromCsv();

    match.Find(); // find the poses from groundtruth and slam that share the same timestamp 

    Registration regi(nh);

    regi.set_slam_pose(match.get_samestamped_pose_slam());
    regi.set_gt_pose(match.get_samestamped_pose_groundtruth());
    regi.set_match_count(match.get_match_count());

    regi.CalculateTransformationRegistration();
    regi.ApplyRegistration();

    if(regi.visualization_choice_.compare(std::string("visualize")) == 0)
        regi.PubMarker();
    regi.ShowError();
}