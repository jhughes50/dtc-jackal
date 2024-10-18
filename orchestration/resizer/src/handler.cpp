/*!
* Jason Hughes
* August 2024
* 
* Node to pass information to localizer
* and to publish the grid map
*
* DTC PRONTO 2024
*
*/

#include "amm/handler.hpp"

MapperNode::MapperNode(ros::NodeHandle* nh) : extrinsics_(3,4)
{
    ros::Duration(2.0).sleep();

    nh->getParam("alt_threshold", alt_threshold_);
    nh->getParam("subsample", sub_sample_);
    loadIntrinsics(nh);
    
    cv::Size d(width_/4, height_/4);
    down_res_ = d;

    img_sub_ = nh->subscribe("/image", 1, &MapperNode::imageCallback, this);
    alt_sub_ = nh->subscribe("/mavros/altitude", 1, &MapperNode::altitudeCallback, this);
    local_pose_sub_ = nh->subscribe("/mavros/global_position/local", 1, &MapperNode::localPositionCallback, this);

    map_pub_ = nh->advertise<grid_map_msgs::GridMap>("map", 1); 

    grid_map::GridMap map({"elevation"});
    map.setBasicLayers({"elevation"});
    map.setFrameId("map");
    map_ = map;

    count_ = 0;
    alt_ = 0.0; 

    ROS_INFO("[AMM] Node Initialized");
}

void MapperNode::imageCallback(const sensor_msgs::Image& msg)
{
    if (alt_ > alt_threshold_ && count_ == 0)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat cvimg = cv_ptr->image;

        cv::resize(cvimg, cvimg, down_res_);
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        cv_bridge::CvImage cv_image(header, "bgr8", cvimg);
    
        sensor_msgs::Image new_msg;
        cv_image.toImageMsg(new_msg);

        ROS_INFO("[AMM] Initializing Grid Map");
        grid_map::GridMapRosConverter::initializeFromImage(new_msg, resolution_, map_);
        ROS_INFO("[AMM] Initialized"); 
        grid_map::GridMapRosConverter::addLayerFromImage(new_msg, "elevation", map_, 0.0, 1.0);
        grid_map::GridMapRosConverter::addColorLayerFromImage(new_msg, "color", map_);
        ROS_INFO("[AMM] Layers added");
        grid_map::GridMapRosConverter::toMessage(map_, gmsg_);
        gmsg_.info.header.frame_id="map";
        map_pub_.publish(gmsg_);
        count_ = 0;
        count_++;
    }
    else if (count_ > 0 && count_ < 10)
    {
        ROS_INFO("[AMM] Publishing Grid Map");
        map_pub_.publish(gmsg_);
        count_++;
    }
}


void MapperNode::localPositionCallback(const nav_msgs::Odometry& msg)
{
    Eigen::Vector3d position(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    translation_ = position;
    Eigen::Quaterniond quaternion(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);

    rotation_matrix_ = quaternion.toRotationMatrix();

    extrinsics_.block<3,3>(0,0) = rotation_matrix_;
    extrinsics_.block<3,1>(0,3) = position;
    pose_ = msg.pose.pose;
}

void MapperNode::altitudeCallback(const mavros_msgs::Altitude& msg)
{
    alt_ = msg.relative;
	
    ROS_INFO_STREAM("[AMM] Current Altiude: " << alt_);

    int width = width_ / 4;

    double img_width = 2.0 * (alt_ *(double) tan(hfov_/2));
    resolution_ = img_width / width;
}

void MapperNode::loadIntrinsics(ros::NodeHandle* nh)
{
    ROS_INFO("[AMM] Initializing Params");
    intrinsics_ = Eigen::Matrix3d::Identity();
    std::vector<double> coeffs;
    nh->getParam("sync/cam0/intrinsics", coeffs);
    intrinsics_(0,0) = coeffs[0];
    intrinsics_(1,1) = coeffs[1];
    intrinsics_(0,2) = coeffs[2];
    intrinsics_(1,2) = coeffs[3];
    ROS_INFO("[AMM] intrinsics initialized"); 
    std::vector<int> res;
    nh->getParam("sync/cam0/resolution", res);
    width_ = res[0];
    height_ = res[1];
    nh->getParam("sync/cam0/hfov", hfov_);
    nh->getParam("sync/cam0/vfov", vfov_);
    ROS_INFO("[AMM] Params Initialized");
}

