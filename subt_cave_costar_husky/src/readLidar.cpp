#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <math.h>

void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "readLidar");

    ros::NodeHandle n;
    ros::Subscriber lidar_sub = n.subscribe("/COSTAR_HUSKY/points", 1000, lidar_callback);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("COSTAR_HUSKY/cmd_vel", 10);

    while (ros::ok())
    {
        ros::spinOnce();
    }
}
