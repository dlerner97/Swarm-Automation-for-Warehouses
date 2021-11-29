#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "circle");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise <geometry_msgs::Twist> ("/cmd_vel", 1000);
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = 1.0;
    move_cmd.angular.z = 1.0;
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        pub.publish(move_cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
        
