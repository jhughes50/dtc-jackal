/*!
* Jason Hughes
* September 2024
*
* Test the grid map production 
* of amm
*
* DTC PRONTO 2024
*/


//39.941254, -75.198416
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
    // initialize ROS
    ROS_INFO("[TEST] Initializing test node");
    ros::init(argc, argv, "amm_test_node");
    ros::NodeHandle nh;

    ROS_INFO("[TEST] Creating publisher");
    ros::Publisher pub = nh.advertise<grid_map_msgs::GridMap>("map", 2); 

    // load test image
    ROS_INFO("[TEST] Loading Test Image");
    cv::Mat img = cv::imread("/home/jason/ws/src/amm/test/test.png", cv::IMREAD_COLOR);
    cv::Size down_res_size(img.cols/4,img.rows/4);
    cv::resize(img, img, down_res_size);

    ROS_INFO_STREAM("[TEST] Img Size " << img.rows << ", " << img.cols);
    // initialize grid map
    grid_map::GridMap map({"elevation"});
    map.setBasicLayers({"elevation"});
    map.setFrameId("map");

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    cv_bridge::CvImage cv_image(header, "rgb8", img);

    // Convert CvImage to ROS image message
    sensor_msgs::Image msg;
    cv_image.toImageMsg(msg);
    
    ROS_INFO("[TEST] Initializing grid map from Image");
    grid_map::GridMapRosConverter::initializeFromImage(msg, 0.1, map);

    ROS_INFO("[TEST] Adding Elevation layer");
    float min = 0.0;
    float max = 1.0;
    grid_map::GridMapRosConverter::addLayerFromImage(msg, "elevation", map, min, max);
    ROS_INFO("[TEST] Adding Color layer");
  	grid_map::GridMapRosConverter::addColorLayerFromImage(msg, "color", map);

    ROS_INFO("[TEST] Converting to ros msg");
    grid_map_msgs::GridMap gmsg;
	grid_map::GridMapRosConverter::toMessage(map, gmsg);
    gmsg.info.header.frame_id="map";
    
    ROS_INFO("[TEST] Publishing 10 msgs at 10Hz");
    
    ros::Rate rate(10);
    int count = 0;

    while(count < 10)
    {
        pub.publish(gmsg);
        rate.sleep();
        count++;
    }
    return 0;
}
