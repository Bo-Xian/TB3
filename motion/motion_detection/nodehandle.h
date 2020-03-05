#include <cstdio>
#include <highgui.h>
#include <math.h>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>

using namespace std;

class NodeHandle
{
  public:
    NodeHandle();
    ~NodeHandle();
    void stop_robot();
    bool is_moving;
  private:
    ros::NodeHandle nh;
    //==============subscriber====================
    ros::Subscriber move_sub;

    void movecall(const std_msgs::Bool msg);

    ros::Publisher cmdvel_pub;
};
