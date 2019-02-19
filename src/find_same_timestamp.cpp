#include "find_same_timestamp.h"

using namespace std;

fst::fst(ros::NodeHandle& nh){
    sub_slam = nh.subscribe("keyframe_pose", 100, &fst::slam_pose_callback, this);

    if(nh.getParam("do_registration_threshold", do_registration_threshold)){
        ROS_INFO("when synchronized message number is more than %d, we will do registration. Modify this number in launch file", do_registration_threshold);
    }
    else{
        do_registration_threshold = 500;
        ROS_INFO("don't know when to do registration, set default as %d", do_registration_threshold);
    }

    if(nh.getParam("groundtruth_csv_file_address_", groundtruth_csv_file_address_)){
        ROS_INFO("get the file address");
    }
    else{
        ROS_INFO("no file address input");
    }
};

void fst::slam_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msgs){
    all_slam.push_back(*msgs);
    
    msg_count++;
    ROS_INFO("get msgs %d", msg_count);
    
    if(msg_count >= do_registration_threshold){
        ROS_INFO("start findding the same timestamp of two message");
        keep_spin = false;
        sub_slam.shutdown();//do not subscribe message anymore
    }
};

void fst::read_gt_pose_from_csv(){
    bool skip_first_row  = true;
    int  sequence         = 0;
    ifstream groundtruth_file(groundtruth_csv_file_address_.c_str());
    if(!groundtruth_file){
        ROS_FATAL("cannot find the file");
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
            all_gt.push_back(groundtruth);
        }
    }    
};

void fst::find(){
    std::vector<geometry_msgs::PoseStamped>::iterator it_gt, it_slam;

    geometry_msgs::PoseStamped one_gt, one_slam;

    int from_last = 0;
    int count     = 0;
    std::cout<<"all slam size "<<all_slam.size()<<", gt size "<<all_gt.size()<<std::endl;
    for(it_slam = all_slam.begin(); it_slam != all_slam.end(); it_slam++){
        one_slam = *it_slam;
        double slam_pose_time = one_slam.header.stamp.sec + 1e-9 * one_slam.header.stamp.nsec;
        for(it_gt = all_gt.begin() + from_last; it_gt != all_gt.end(); it_gt++){
            count++;
            one_gt = *it_gt;
            double gt_pose_time = one_gt.header.stamp.sec + 1e-9 * one_gt.header.stamp.nsec;
            //if the timestamp of slam and ground truth is smaller than a certain value, we think they have the same timestamp
            if(std::abs(slam_pose_time - gt_pose_time)<same_timestamp_threshold){
                //std::cout<<"find same time"<<std::endl;
                same_gt.push_back(one_gt);
                same_slam.push_back(one_slam);
                from_last = count;
                break;
            }
        }
        if(count >= all_gt.size())
            count = 0; //sometimes the groundtruth doesn't have data when the keyframe starts, whcih means some keyframes cannot find the corresponding groundtruth that shares the same timestamp so the count should go back to 0
        if(same_gt.size() == do_registration_threshold)
            break;
    }
}
