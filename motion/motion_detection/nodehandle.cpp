#include "nodehandle.h"

NodeHandle::NodeHandle()
{
    is_moving = false;
    move_sub = nh.subscribe("tb3/strategy/moving", 1, &NodeHandle::movecall, this);
    
    cmdvel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}
NodeHandle::~NodeHandle()
{
}
void NodeHandle::movecall(const std_msgs::Bool msg)
{
    if(msg.data==true){
        std::cout<<"moving\n";
        is_moving = true;
    }
}

void NodeHandle::stop_robot()
{
    std::cout<<"stop\n";
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
	twist.angular.z = 0;
    cmdvel_pub.publish(twist);
}
