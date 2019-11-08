#include "find_same_timestamp.h"

using namespace std;

FST::FST(ros::NodeHandle& nh){
    if(nh.getParam("same_timestamp_threshold_", same_timestamp_threshold_)){
        ROS_INFO("timestamp difference between slam and groundtruth samller than %f will be seen as the same", same_timestamp_threshold_);
    }
    else{
        ROS_INFO("You didn't set the same_timestamp_threshold, default is 3ms");
    }

    if(nh.getParam("sub_topic_name_", sub_topic_name_)){
        ROS_INFO("get the topic name %s", sub_topic_name_.c_str());
    }
    else{
        sub_topic_name_ = std::string("/geopose_update");
        ROS_INFO("no topic name given, the default is /geopose_update");
    }

    if(nh.getParam("do_registration_threshold_", do_registration_threshold_)){
        //set a large threshold or a threshold smaller than 1 will lead the data_registration use all the msg it receives after slam stops
        if(do_registration_threshold_<1){
            ROS_INFO("register all the message the system receives after the slam stops");
            do_registration_threshold_ = 999999;
        }
        else{
            ROS_INFO("when synchronized message number is more than %d, we will do registration. Modify this number in launch file", do_registration_threshold_);
        }
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

    if(nh.getParam("groundtruth_format_", groundtruth_format_)){
        ROS_INFO("get the groundtruth format (Only support euroc format and tum format), it is %s", groundtruth_format_.c_str());
    }
    else{
        groundtruth_format_ = std::string("tum_format");
        ROS_INFO("didn't input the ground truth format, default is tum format");
    }



    sub_slam_ = nh.subscribe(sub_topic_name_, 100, &FST::SlamPoseCallback, this); //normally the topic name is keyframe_pose
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
            if(groundtruth_format_ == "tum_format"){
                while(getline(temp_one_row_gt, string_gt, ' ')){
                    switch(sequence){
                        case 0:{
                            double msg_timestamp = (double)atof(string_gt.c_str());
                            //std::cout<<"timestamp of image "<<msg_timestamp<<std::endl;
                            msg_timestamp = 1e9 * msg_timestamp;
                            uint64_t int_time = uint64_t(msg_timestamp);
                            groundtruth.header.stamp.sec =  int_time/1e9;;
                            groundtruth.header.stamp.nsec = int_time%1000000000;
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
                            //the case 4,5,6,7 is quternion x,y,z,w for tum
                            groundtruth.pose.orientation.x = (double)atof(string_gt.c_str());
                            sequence++;
                        }
                            break;
                        case 5:{
                            groundtruth.pose.orientation.y = (double)atof(string_gt.c_str());
                            sequence++;
                        }
                            break;
                        case 6:{
                            groundtruth.pose.orientation.z = (double)atof(string_gt.c_str());
                            sequence++;
                        }
                            break;
                        case 7:{
                            groundtruth.pose.orientation.w = (double)atof(string_gt.c_str());
                            sequence++;
                        }
                            break;
                        default:
                            break;
                    }
                }
            }
            else if(groundtruth_format_ == "euroc_format"){
                while(getline(temp_one_row_gt, string_gt, ',')){
                    switch(sequence){
                        case 0:{
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
                            //the case 4,5,6,7 is quternion w, x, y, z for euroc
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
            }
            else{
                ROS_ERROR("format not support. Must be tum or euroc format");
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
    std::cout<<"among "<<msg_count_<<" keyframes, we found "<<match_count_<<" keyframes that has correspondence in groundtruth data"<<std::endl;
}

void FST::CheckTime(){
    if(msg_count_==1){
        last_timestamp_msg_received_ = ros::Time::now().toSec();
    }
    else if(msg_count_>=1 && msg_count_ == msg_count_before_){
        double time_diff = ros::Time::now().toSec() - last_timestamp_msg_received_;
        last_timestamp_msg_received_ = ros::Time::now().toSec();
        time_no_new_msg += time_diff;
        if(time_no_new_msg>max_wait_time_){
            ROS_INFO("no message reveive in %f seconds, we think the slam has stopped so we'll do registration now", max_wait_time_);
            keep_spin_ = false;
            sub_slam_.shutdown();
        }
    }
    if(msg_count_ != msg_count_before_)
        time_no_new_msg = 0.0;
    msg_count_before_ = msg_count_;
}
