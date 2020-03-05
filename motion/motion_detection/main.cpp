#include "nodehandle.h"
void SigintHandler(int sig)
{
    ROS_INFO("shutting down!");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_detection");
    ros::NodeHandle h_node;
    //signal(SIGINT, SigintHandler);
    ros::Rate loop_rate(1); //program speed limit

	NodeHandle node;
    std::cout<<"motion_detection start\n";
    fflush(stdout); //更新文字緩衝區

    while (ros::ok())
    {
        if(node.is_moving==false){
            node.stop_robot();        
        }
        node.is_moving = false;
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Node exit");
    printf("Process exit\n");
    return 0;
}
