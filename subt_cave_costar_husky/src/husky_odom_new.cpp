#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
// #include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_odom_publisher");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("COSTAR_HUSKY/laser_odom", 10);

    tf::TransformListener pose_listener;

    ros::Rate loop_rate(10.0);
    while (n.ok())
    {
        ros::spinOnce(); // check for incoming messages

        tf::StampedTransform transform;
        nav_msgs::Odometry laser_odom;

        try
        {
            pose_listener.lookupTransform("/map", "/camera", ros::Time(0), transform);
            // ROS_INFO("publishing data to /COSTAR_HUSKY/laser_odom");
            // cout << "x: " << transform.getOrigin().x() << endl;
            // cout << "y: " << transform.getOrigin().y() << endl;
            // cout << "z: " << transform.getOrigin().z() << endl;
            // cout << "qw: " << transform.getRotation().w() << endl;
            // cout << "qx: " << transform.getRotation().x() << endl;
            // cout << "qy: " << transform.getRotation().y() << endl;
            // cout << "qz: " << transform.getRotation().z() << endl;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s \n no data published to /COSTAR_HUSKY/laser_odom", ex.what());
            ros::Duration(1.0).sleep();
        }

        // set header
        laser_odom.header.stamp = ros::Time::now();
        laser_odom.header.frame_id = "map";
        //set position
        laser_odom.pose.pose.position.x = transform.getOrigin().x();
        laser_odom.pose.pose.position.y = transform.getOrigin().y();
        laser_odom.pose.pose.position.z = transform.getOrigin().z();
        //set orientation
        laser_odom.pose.pose.orientation.w = transform.getRotation().w();
        laser_odom.pose.pose.orientation.x = transform.getRotation().x();
        laser_odom.pose.pose.orientation.y = transform.getRotation().y();
        laser_odom.pose.pose.orientation.z = transform.getRotation().z();

        odom_pub.publish(laser_odom);

        loop_rate.sleep();
    }

    return 0;
}