#include "removert/Removerter.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "removert");
    ROS_INFO("\033[1;32m----> Removert Main Started.\033[0m");

    Removerter RMV;
    if(RMV.use_batch_removal) {
        RMV.batchRemoval();
    }
    else {
            RMV.run();
    }
    ROS_INFO("\033[1;32m----> Removert Main ended, please quit\033[0m");
    ros::spin();

    return 0;
}