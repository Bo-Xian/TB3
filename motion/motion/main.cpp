#include "nodehandle.h"
void SigintHandler(int sig)
{
    ROS_INFO("shutting down!");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_node");
    ros::NodeHandle h_node;
    //signal(SIGINT, SigintHandler);
    ros::Rate loop_rate(60); //program speed limit

	NodeHandle node;
    std::cout<<"motion_node start\n";
    fflush(stdout); //更新文字緩衝區

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Node exit");
    printf("Process exit\n");
    return 0;
}
