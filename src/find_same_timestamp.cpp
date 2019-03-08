#include "find_same_timestamp.h"

using namespace std;

FST::FST(ros::NodeHandle& nh){
    sub_slam_ = nh.subscribe("keyframe_pose", 100, &FST::SlamPoseCallback, this);

    if(nh.getParam("do_registration_threshold", do_registration_threshold_)){
        ROS_INFO("when synchronized message number is more than %d, we will do registration. Modify this number in launch file", do_registration_threshold_);
    }
    else{
        do_registration_threshold_ = 500;
        ROS_INFO("don't know when to do registration, set default as %d", do_registration_threshold_);
    }

    if(nh.getParam("groundtruth_csv_file_address_", groundtruth_csv_file_address_)){
        ROS_INFO("get the file address");
    }
    else{
        ROS_INFO("no file address input");
    }
};

void FST::SlamPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msgs){
    all_slam_.push_back(*msgs);
    
    msg_count_++;
    ROS_INFO("get msgs %d", msg_count_);
    
    if(msg_count_ >= do_registration_threshold_){
        ROS_INFO("start findding the same timestamp of two message");
        keep_spin_ = false;
        sub_slam_.shutdown();//do not subscribe message anymore
    }
};

void FST::ReadGTPoseFromCsv(){
    bool skip_first_row  = true;
    int  sequence         = 0;
    ifstream groundtruth_file(groundtruth_csv_file_address_.c_str());
    if(!groundtruth_file){
        ROS_FATAL("cannot find the file that contains groundtruth");
        exit(0);
    }
    else{
        string one_row_gt;
        while(getline(groundtruth_file,one_row_gt)){
            if(skip_first_row) {skip_first_row = false; continue;}
            istringstream temp_one_row_gt(one_row_gt);
            geometry_msgs::PoseStamped groundtruth; 
            string string_gt;
            while(getline(temp_one_row_gt, string_gt, ',')){
                switch(sequence){
                    case 0:{
                        //I convert the data time forcely twice, which is not good, but in this case it will be OK
                        //I convert a timestamp with nanosecond to int second part and int nanosecond part
                        double msg_timestamp = (double)atof(string_gt.c_str());//directly to integer will beyond the int limit, we can use longlong int
                        msg_timestamp = 1e-9 * msg_timestamp;
                        double sec, nsec;
                        nsec = modf(msg_timestamp, &sec);
                        groundtruth.header.stamp.sec = int(sec);
                        groundtruth.header.stamp.nsec = int(1e9 * nsec);
                        sequence++;
                    }
                        break;
                    case 1:{
                        //the case 1,2,3 is position x,y,z
                        groundtruth.pose.position.x = (double)atof(string_gt.c_str());
                        sequence++;
                    }
                        break;
                    case 2:{
                        groundtruth.pose.position.y = (double)atof(string_gt.c_str());
                        sequence++;
                    }
                        break;
                    case 3:{
                        groundtruth.pose.position.z = (double)atof(string_gt.c_str());
                        sequence++;
                    }
                        break;
                    case 4:{
                        //the case 4,5,6,7 is quternion w,x,y,z
                        groundtruth.pose.orientation.w = (double)atof(string_gt.c_str());
                        sequence++;
                    }
                        break;
                    case 5:{
                        groundtruth.pose.orientation.x = (double)atof(string_gt.c_str());
                        sequence++;
                    }
                        break;
                    case 6:{
                        groundtruth.pose.orientation.y = (double)atof(string_gt.c_str());
                        sequence++;
                    }
                        break;
                    case 7:{
                        groundtruth.pose.orientation.z = (double)atof(string_gt.c_str());
                        sequence++;
                    }
                        break;
                    default:
                        break;
                }
            }
            sequence = 0;
            all_gt_.push_back(groundtruth);
        }
    }    
};

void FST::Find(){
    std::vector<geometry_msgs::PoseStamped>::iterator it_gt, it_slam;

    geometry_msgs::PoseStamped one_gt, one_slam;

    int from_last = 0;
    int count     = 0;
    std::cout<<"all slam size "<<all_slam_.size()<<", gt size "<<all_gt_.size()<<std::endl;
    for(it_slam = all_slam_.begin(); it_slam != all_slam_.end(); it_slam++){
        one_slam = *it_slam;
        double slam_pose_time = one_slam.header.stamp.sec + 1e-9 * one_slam.header.stamp.nsec;
        for(it_gt = all_gt_.begin() + from_last; it_gt != all_gt_.end(); it_gt++){
            count++;
            one_gt = *it_gt;
            double gt_pose_time = one_gt.header.stamp.sec + 1e-9 * one_gt.header.stamp.nsec;
            //if the timestamp of slam and ground truth is smaller than a certain value, we think they have the same timestamp
            if(std::abs(slam_pose_time - gt_pose_time)<same_timestamp_threshold_){
                //std::cout<<"find same time"<<std::endl;
                match_count_++;
                same_gt_.push_back(one_gt);
                same_slam_.push_back(one_slam);
                from_last = count;
                break;
            }
        }
        if(count >= all_gt_.size())
            count = from_last; //sometimes the groundtruth doesn't have data when the keyframe starts, or groundtruth is lost at some part, or keyframe and groundtruth just won't be in the same_timestamp_threshold, whcih means some keyframes cannot find the corresponding groundtruth that shares the same timestamp so the count should go back to from_last
        if(same_gt_.size() == do_registration_threshold_)
            break;
    }
    std::cout<<"among "<<do_registration_threshold_<<" keyframes, we found "<<match_count_<<" keyframes that has correspondence in groundtruth data"<<std::endl;
}
