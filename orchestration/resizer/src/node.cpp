/*!
* Jason Hughes
* August 2024
*
* Start up for grid map maker
*
* DTC PRONTO 2024
*/

#include "amm/handler.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "amm_img2map");
    ros::NodeHandle nh;

    MapperNode map(&nh);

    ros::spin();

    return 0;
}
