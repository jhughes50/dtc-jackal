/*!
* Jason Hughes
* August 2024
*
* Handle the ros input and pass it to 
* the localizer
*
* DTC PRONTO 2024
*/

#ifndef HANDLER_HPP
#define HANDLER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <mavros_msgs/Altitude.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <Eigen/Geometry>
#include <math.h>

#include <amm/pixel_localizer.hpp>
#include <amm/matrix.hpp>


class MapperNode
{
    public:
        MapperNode(ros::NodeHandle* nh);

    private:

        void loadIntrinsics(ros::NodeHandle* nh);

        void imageCallback(const sensor_msgs::Image& msg);
        void altitudeCallback(const mavros_msgs::Altitude& msg);
        void localPositionCallback(const nav_msgs::Odometry& msg);

        ros::Subscriber img_sub_;
        ros::Subscriber alt_sub_;
        ros::Subscriber local_pose_sub_;

        ros::Publisher map_pub_;

        Eigen::MatrixXd extrinsics_;
        Eigen::Matrix3d intrinsics_;
        Eigen::Matrix3d rotation_matrix_;
        Eigen::Vector3d translation_;

        int hfov_, vfov_;
        int height_, width_;
        int sub_sample_;
        double resolution_;
    	geometry_msgs::Pose pose_;
        
        cv::Size down_res_;
        float alt_;
	    float alt_threshold_;
        int count_ = 0;

        grid_map::GridMap map_;
	    grid_map_msgs::GridMap gmsg_;
};
#endif
