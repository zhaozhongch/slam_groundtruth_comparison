#include "registration.h"

registration::registration(ros::NodeHandle& nh_){
    nh = nh_;

    pub_MA = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",10);

    if(nh.getParam("do_registration_threshold", do_registration_threshold)){
        ROS_INFO("got registration threshold %d. Modify this number in launch file", do_registration_threshold);
    }
    else{
        ROS_FATAL("don't know when to do registration");
    }

    
    nh.getParam("visualization_choice_", visualization_choice_);
    //if(visualizationChoice.c_str() == "visualize") this seems not work
    // std::cout<<"visualization choice "<<visualizationChoice.c_str()<<std::endl;
    if(visualization_choice_.compare(std::string("visualize")) == 0){
         ROS_INFO("if you want to visualize the result, please run rviz first. The world frame name now is called my_frame and markerArray with ground truth and slam pose will be published");
    }
}

//do registration algorithm is according to http://graphics.stanford.edu/courses/cs348a-17-winter/Papers/quaternion.pdf
void registration::calculate_transformation_registration(){
    int countflag = 0;

    geometry_msgs::Point point_gt, point_slam;
    geometry_msgs::Point ave_point_gt, ave_point_slam;
    geometry_msgs::Point bias_point_gt, bias_point_slam;
    std::vector<geometry_msgs::PoseStamped>::iterator it_gt, it_slam;

    Eigen::Matrix4d m_bias_gt;
    Eigen::Matrix4d m_bias_slam;
    Eigen::Matrix4d M = Eigen::Matrix4d::Zero();

    ave_point_gt    = getAverage3Dpoint(poses_gt);
    ave_point_slam  = getAverage3Dpoint(poses_slam);

    for(it_gt=poses_gt.begin(),it_slam = poses_slam.begin() ; it_gt!=poses_gt.end() && it_slam!=poses_slam.end() ; it_gt++, it_slam++){
        countflag++;
        point_gt   = (*it_gt).pose.position;
        point_slam = (*it_slam).pose.position;

        // if(countflag < 5){
        //     std::cout<<std::setprecision(15)<< (*it_gt).header.stamp.sec + 1e-9 * (*it_gt).header.stamp.nsec<<std::endl; 
        //     std::cout<<std::setprecision(15)<< (*it_slam).header.stamp.sec + 1e-9 * (*it_slam).header.stamp.nsec<<std::endl; 
        // }
        
        bias_point_gt.x   = point_gt.x   - ave_point_gt.x;
        bias_point_gt.y   = point_gt.y   - ave_point_gt.y;
        bias_point_gt.z   = point_gt.z   - ave_point_gt.z;
        bias_point_slam.x = point_slam.x - ave_point_slam.x;
        bias_point_slam.y = point_slam.y - ave_point_slam.y; 
        bias_point_slam.z = point_slam.z - ave_point_slam.z;

        m_bias_gt   = point3D2MatrixForm_Q(bias_point_gt);
        m_bias_slam = point3D2MatrixForm_P(bias_point_slam);
        //std::cout<<"m_bias_gt \n"<<m_bias_gt<<std::endl;
        //std::cout<<"m_bias_slam \n "<<m_bias_slam<<std::endl;
        M += m_bias_slam.transpose() * m_bias_gt;
        //std::cout<<"M \n"<<M<<std::endl;
    }

    Eigen::EigenSolver<Eigen::Matrix4d> es(M);
    Eigen::Vector4d  e_value;//eigen value
    e_value = es.eigenvalues().real();//es.eigenvalues()返回复数对的Ｖector4cd， 用.real()提取实数部分

    //Vector4d Etest = es.eigenvalues().imag();//image should be 0

    //pick the eigen vector that corresponding to the maximum eigen value, then the vector in quaternion form is the optimal rotation
    Eigen::Vector4d::Index Imax;
    double maxEvalue = e_value.maxCoeff(&Imax);
    Eigen::Vector4cd max_e_vector = es.eigenvectors().col(Imax); //eigenvector, two column four rows, complex form
    Eigen::Vector4d max_e_vector_real = max_e_vector.real();
    //get the optimal rotation
    optimal_transformation.orientation.w = max_e_vector_real[0];
    optimal_transformation.orientation.x = max_e_vector_real[1];
    optimal_transformation.orientation.y = max_e_vector_real[2];
    optimal_transformation.orientation.z = max_e_vector_real[3];

    //get the optimal translation after we get the rotation. Need change quaternion to rotation matrix to do matrix multiplication
    Eigen::Quaterniond q;
    q.w() = max_e_vector_real[0];
    q.x() = max_e_vector_real[1];
    q.y() = max_e_vector_real[2];
    q.z() = max_e_vector_real[3];
    
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    std::cout<<"rotation matrix \n "<<R<<std::endl;
    
    Eigen::Vector3d ave_point_slam_eigen, ave_point_gt_eigen;
    ave_point_slam_eigen<<ave_point_slam.x,ave_point_slam.y,ave_point_slam.z;
    ave_point_gt_eigen<<ave_point_gt.x,ave_point_gt.y,ave_point_gt.z;
    //cout<<"R is "<<"\n"<<R<<endl;
    Eigen::Vector3d b = ave_point_gt_eigen - R*ave_point_slam_eigen;//formula １３
    std::cout<<"translation vector \n "<<b<<std::endl;
    optimal_transformation.position.x = b(0);
    optimal_transformation.position.y = b(1);
    optimal_transformation.position.z = b(2);
}

void registration::apply_registration(){
    std::vector<geometry_msgs::PoseStamped>::iterator it;
    //transformation to tf form
    tf::Transform tf_optimal_transformation;
    tf::poseMsgToTF(optimal_transformation, tf_optimal_transformation);
    //geometry point to tf point so that we can apply transformation(matrix) multiplication
    geometry_msgs::Point msg_position_slam, msg_position_slam_after_registration;
    geometry_msgs::PoseStamped  msg_pose_slam_after_registration;
    tf::Vector3 tf_position_slam;
    for(it = poses_slam.begin(); it != poses_slam.end(); it++){
        msg_position_slam = (*it).pose.position;
        tf::pointMsgToTF(msg_position_slam,tf_position_slam);
        tf::Vector3 tf_position_slam_after_registration = tf_optimal_transformation * tf_position_slam;
        tf::pointTFToMsg(tf_position_slam_after_registration, msg_position_slam_after_registration);
        
        //the orientation should not change because we treat all points as rigid body. We didn't do anything on orientation
        msg_pose_slam_after_registration.pose.orientation = (*it).pose.orientation;
        msg_pose_slam_after_registration.pose.position    = msg_position_slam_after_registration;
        msg_pose_slam_after_registration.header           = (*it).header;
        poses_slam_after_registration.push_back(msg_pose_slam_after_registration);
    }

    regi_done = true;
}

void registration::pub_marker(){

    visualization_msgs::MarkerArray markerArr;
    
    std::vector<geometry_msgs::PoseStamped>::iterator it_gt, it_slam;

    set_marker_basics(marker1);//set frameid, marker type, timestamp, lifetime, scale and shape
    set_marker_basics(marker2);

    set_marker_namespace(marker1, "slam_poses");
    set_marker_namespace(marker2, "groundtruth_poses");

    set_marker_color(marker1, std::string("red"));//can choose red, blue, green, white and black
    set_marker_color(marker2, std::string("blue"));

    int count = 0;
    for(it_gt=poses_gt.begin(),it_slam = poses_slam_after_registration.begin() ; it_gt!=poses_gt.end() && it_slam!=poses_slam_after_registration.end() ; it_gt++, it_slam++){
        set_marker_id(marker1, count);
        set_marker_id(marker2, count);

        set_marker_pose(marker1, *(it_slam));
        set_marker_pose(marker2, *(it_gt));

        markerArr.markers.push_back(marker1);
        markerArr.markers.push_back(marker2);
        count++;
    }

    usleep(500000);
    pub_MA.publish(markerArr);
}

double registration::show_error(){
    std::vector<geometry_msgs::PoseStamped>::iterator it_gt, it_slam;
    for(it_gt=poses_gt.begin(),it_slam = poses_slam_after_registration.begin() ; it_gt != poses_gt.end() && it_slam!=poses_slam_after_registration.end() ; it_gt++, it_slam++){
        geometry_msgs::PoseStamped slam, gt;
        slam = *it_slam;
        gt   = *it_gt;
        error += sqrt((slam.pose.position.x - gt.pose.position.x) * (slam.pose.position.x - gt.pose.position.x) 
                 + (slam.pose.position.y - gt.pose.position.y) * (slam.pose.position.y - gt.pose.position.y)
                 + (slam.pose.position.z - gt.pose.position.z) * (slam.pose.position.z - gt.pose.position.z));
    }
    ROS_INFO("the translation RMSE error between slam and ground truth is...... %f", error);
}

void registration::set_slam_pose(const std::vector<geometry_msgs::PoseStamped>& slam_pose){
    poses_slam = slam_pose;
};

void registration::set_gt_pose(const std::vector<geometry_msgs::PoseStamped>& gt_pose){
    poses_gt   = gt_pose;
};



//the following is private memeber function
geometry_msgs::Point registration::getAverage3Dpoint(const std::vector<geometry_msgs::PoseStamped>& poses){
    geometry_msgs::Point ave;
    int count = 0;
    ave.x     = 0.0;
    ave.y     = 0.0;
    ave.z     = 0.0;
    std::vector<geometry_msgs::PoseStamped>::const_iterator it;
    for(it = poses.begin(); it != poses.end(); it++){
        count++;
        ave.x += (*it).pose.position.x;
        ave.y += (*it).pose.position.y;
        ave.z += (*it).pose.position.z;
    }
    ave.x = ave.x/count;
    ave.y = ave.y/count;
    ave.z = ave.z/count;
    return ave;
};

Eigen::Matrix4d registration::point3D2MatrixForm_Q(geometry_msgs::Point point){
    Eigen::Matrix4d m = Eigen::Matrix4d::Zero();

    m(1,0) =  point.x;
    m(2,0) =  point.y;
    m(3,0) =  point.z;

    m(0,1) = -point.x;
    m(2,1) =  point.z;
    m(3,1) = -point.y;
    
    m(0,2) = -point.y;
    m(1,2) = -point.z;
    m(3,2) =  point.x;

    m(0,3) = -point.z;
    m(1,3) =  point.y;
    m(2,3) = -point.x;

    return m;    
};

Eigen::Matrix4d registration::point3D2MatrixForm_P(geometry_msgs::Point point)
{
    Eigen::Matrix4d m = Eigen::Matrix4d::Zero();
    m(1,0) =  point.x;
    m(2,0) =  point.y;
    m(3,0) =  point.z;

    m(0,1) = -point.x;
    m(2,1) = -point.z;
    m(3,1) =  point.y;
    
    m(0,2) = -point.y;
    m(1,2) =  point.z;
    m(3,2) = -point.x;

    m(0,3) = -point.z;
    m(1,3) = -point.y;
    m(2,3) =  point.x;

    return m;
}

void registration::set_marker_basics(visualization_msgs::Marker& marker){

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the marker type.
    uint32_t shape = visualization_msgs::Marker::ARROW;
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the scale(length height...) of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.lifetime = ros::Duration();
};

// Set the namespace and id for this marker.  This serves to create a unique ID
// Any marker sent with the same namespace and id will overwrite the old one
// markers with the different namespace or different id won't be overwrited 
void registration::set_marker_namespace(visualization_msgs::Marker& marker, std::string name){
    marker.ns = name.c_str();
};

void registration::set_marker_id(visualization_msgs::Marker& marker, int id){
    marker.id = id;
}

enum colorChoice{
    red, //0
    green, //1
    blue, //2
    white, //3 
    black //4 
};
void registration::set_marker_color(visualization_msgs::Marker& marker, std::string name){
    //use map so that I can use switch case for string. switch case can not be used for string
    std::map<std::string, colorChoice> choice;

    choice.insert(std::pair<std::string,colorChoice>("red", red));
    choice.insert(std::pair<std::string,colorChoice>("green", green));
    choice.insert(std::pair<std::string,colorChoice>("blue", blue));
    choice.insert(std::pair<std::string,colorChoice>("white", white));
    choice.insert(std::pair<std::string,colorChoice>("black", black));
    // Set the color -- be sure to set alpha to something non-zero!
    switch(choice.find(name.c_str())->second){
        case red:
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
        break;
        case green:
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
        break;
        case blue:
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0;
        break;
        case white:
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0;
        break;
        case black:
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
        break;
        default: //deafult white
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0;
        break;            
    }
}

// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
void registration::set_marker_pose(visualization_msgs::Marker& marker, geometry_msgs::PoseStamped pose){
    marker.pose.position.x = pose.pose.position.x;
    marker.pose.position.y = pose.pose.position.y;
    marker.pose.position.z = pose.pose.position.z;
    marker.pose.orientation.x = pose.pose.orientation.x;
    marker.pose.orientation.y = pose.pose.orientation.y;
    marker.pose.orientation.z = pose.pose.orientation.z;
    marker.pose.orientation.w = pose.pose.orientation.w;
}