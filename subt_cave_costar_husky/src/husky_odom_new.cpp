#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_odom_publisher");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("COSTAR_HUSKY/laser_odom", 10);

    // tf::TransformListener pose_listener;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate loop_rate(10.0);
    while (n.ok())
    {
        ros::spinOnce(); // check for incoming messages

        // tf::StampedTransform transform;
        geometry_msgs::TransformStamped transformStampted;
        nav_msgs::Odometry laser_odom;

        try
        {
            // pose_listener.lookupTransform("/map", "/camera", ros::Time(0), transform);
            transformStampted = tfBuffer.lookupTransform("map", "camera", ros::Time(0));
            // auto tf_out = tfBuffer.transform<geometry_msgs::PoseStamped>('camera', 'COSTAR_HUSKY', ros::Duration(3.0));

            // ROS_INFO("publishing data to /COSTAR_HUSKY/laser_odom");
            // cout << "x: " << transform.getOrigin().x() << endl;
            // cout << "y: " << transform.getOrigin().y() << endl;
            // cout << "z: " << transform.getOrigin().z() << endl;
            // cout << "qw: " << transform.getRotation().w() << endl;
            // cout << "qx: " << transform.getRotation().x() << endl;
            // cout << "qy: " << transform.getRotation().y() << endl;
            // cout << "qz: " << transform.getRotation().z() << endl;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s \n no data published to /COSTAR_HUSKY/laser_odom", ex.what());
            ros::Duration(1.0).sleep();
        }

        // set header
        laser_odom.header.stamp = ros::Time::now();
        laser_odom.header.frame_id = "map";
        //set position
        // laser_odom.pose.pose.position.x = transform.getOrigin().x();
        // laser_odom.pose.pose.position.y = transform.getOrigin().y();
        // laser_odom.pose.pose.position.z = transform.getOrigin().z();
        laser_odom.pose.pose.position.x = transformStampted.transform.translation.x;
        laser_odom.pose.pose.position.y = transformStampted.transform.translation.y;
        laser_odom.pose.pose.position.z = transformStampted.transform.translation.z;
        //set orientation
        // laser_odom.pose.pose.orientation.w = transform.getRotation().w();
        // laser_odom.pose.pose.orientation.x = transform.getRotation().x();
        // laser_odom.pose.pose.orientation.y = transform.getRotation().y();
        // laser_odom.pose.pose.orientation.z = transform.getRotation().z();
        laser_odom.pose.pose.orientation.w = transformStampted.transform.rotation.w;
        laser_odom.pose.pose.orientation.x = transformStampted.transform.rotation.x;
        laser_odom.pose.pose.orientation.y = transformStampted.transform.rotation.y;
        laser_odom.pose.pose.orientation.z = transformStampted.transform.rotation.z;

        odom_pub.publish(laser_odom);

        loop_rate.sleep();
    }

    return 0;
}