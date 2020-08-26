#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;

// robot pose
double *x = new double;
double *y = new double;
double *z = new double;
double *qw = new double;
double *qx = new double;
double *qy = new double;
double *qz = new double;

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    *x = msg->pose.pose.position.x;
    *y = msg->pose.pose.position.y;
    *z = msg->pose.pose.position.z;
    *qw = msg->pose.pose.orientation.w;
    *qx = msg->pose.pose.orientation.x;
    *qy = msg->pose.pose.orientation.y;
    *qz = msg->pose.pose.orientation.z;

    // cout << "Position: " << *x << " " << *y << " " << *z << endl;
    // cout << "Orientation: " << *qw << " " << *qx << " " << *qy << " " << *qz << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;
    ros::Subscriber odom_sub = n.subscribe("/COSTAR_HUSKY/odom", 10, odom_callback);

    // ros::Subscriber imu_sub = n.subscribe("/X1/imu/data", 1000, imu_callback);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time;
    ROS_INFO("start broadcasting to frame: odom");

    ros::Rate loop_rate(60.0);
    while (n.ok())
    {
        ros::spinOnce(); // check for incoming messages
        current_time = ros::Time::now();

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "COSTAR_HUSKY/base_link/front_laser";

        odom_trans.transform.translation.x = *x;
        odom_trans.transform.translation.y = *y;
        odom_trans.transform.translation.z = *z;
        odom_trans.transform.rotation.w = *qw;
        odom_trans.transform.rotation.x = *qx;
        odom_trans.transform.rotation.y = *qy;
        odom_trans.transform.rotation.z = *qz;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        loop_rate.sleep();
    }

    delete x, y, z, qw, qx, qy, qz;
    return 0;
}