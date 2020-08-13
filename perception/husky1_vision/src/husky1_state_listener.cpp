#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "husky1_state_listener");
    ros::NodeHandle node;
     // Artifact position publisher
    ros::Publisher pub = node.advertise<std_msgs::String>("husky1_position", 100);
     // Create tf Listener
    tf::TransformListener listener;
    ros::Rate loop_rate(10);
    // Listening to transform from artifact_origin -> COSTAR_HUSKY
    while(node.ok()){
    tf::StampedTransform transform;
    std_msgs::String position;
        try{
            listener.lookupTransform("/artifact_origin", "/COSTAR_HUSKY", ros::Time(0), transform);
            position.data = std::to_string(transform.getOrigin().x()) + ' ' + std::to_string(transform.getOrigin().y()) + ' ' + std::to_string(transform.getOrigin().z());
            pub.publish(position);
            //ROS_INFO_STREAM("Husky1 position x: " << transform.getOrigin().x() << " y: " << transform.getOrigin().y() <<  " z: " << transform.getOrigin().z());
        }catch(tf::TransformException &ex){
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            //ROS_INFO_STREAM("Hello");
        }
    loop_rate.sleep();
    }
    
    
return 0;
}

