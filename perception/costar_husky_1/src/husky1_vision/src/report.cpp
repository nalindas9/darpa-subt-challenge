/*
Reporting Artifact back to Base Station

Reference:
1. https://github.com/osrf/subt_seed/blob/master/src/subt_seed_node.cc
2. https://github.com/osrf/subt_hello_world/blob/master/subt_solution_launch/src/artifact_reporter.cpp

Authors:
Nalin Das (nalindas9@gmail.com)
Graduate Student pursuing Masters in Robotics,
University of Maryland, College Park
*/

#include<iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <subt_msgs/PoseFromArtifact.h>
#include <subt_communication_broker/subt_communication_client.h>
#include <subt_ign/CommonTypes.hh>
#include <subt_ign/protobuf/artifact.pb.h>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <ignition/msgs.hh>

void artifactCallback(const std_msgs::String::ConstPtr&);
void BaseStationCallback(
    const std::string&,
    const std::string&,
    const uint32_t,
    const std::string&
);
void ReportArtifacts(
    const ros::TimerEvent &, 
    subt::CommsClient&
);

int main(int argc, char **argv){
    ros::init(argc, argv, "report");
    ros::NodeHandle report;
    ros::Subscriber sub = report.subscribe("artifact", 100, artifactCallback);
    ROS_INFO("Reporting Artifact ...");
    // Set up communications with the base station for artifact report
    subt::CommsClient commsClient("COSTAR_HUSKY");
    commsClient.Bind(&BaseStationCallback, "COSTAR_HUSKY");
    // found artifacts will be attempted to be sent periodically through a timer
    ros::Timer timer = report.createTimer(ros::Duration(1.0), boost::bind(&ReportArtifacts, _1, boost::ref(commsClient)));
    ros::spin();
    return 0;
}

// Artifact topic subscriber callback
void artifactCallback(const std_msgs::String::ConstPtr& msg){
    //ROS_INFO("Artifact detected: %s", msg->data.c_str());
}

// Base Station Callback
void BaseStationCallback(const std::string& srcAddress, const std::string& dstAddress, const uint32_t dstPort, const std::string& data){
    subt::msgs::ArtifactScore res;
    if(!res.ParseFromString(data)){
        ROS_ERROR("ArtifactReporter::BaseStationCallback(): error deserializing message.");
    }
    geometry_msgs::Point location;
    location.x = res.artifact().pose().position().x();
    location.y = res.artifact().pose().position().y();
    location.z = res.artifact().pose().position().z();
    
    ROS_INFO_STREAM("Artifact " << res.artifact().type() << " at location " << location.x << ", " << location.y << ", " << location.z << " was recieved by the base station\n");
    ROS_INFO("Message from [%s] to [%s] on port [%u]:\n [%s]", srcAddress.c_str(),
      dstAddress.c_str(), dstPort, res.DebugString().c_str());

}

// Report Artifact
void ReportArtifacts(const ros::TimerEvent&, subt::CommsClient& commsClient){
    subt::msgs::Artifact artifact;
    // Set Artifact type and pose
    ignition::msgs::Pose pose;
    pose.mutable_position()->set_x(70.661);
    pose.mutable_position()->set_y(-4.233);
    pose.mutable_position()->set_z(0.168);
    artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_BACKPACK)); 
    artifact.mutable_pose()->CopyFrom(pose);
    // Serialize the Artifact
    std::string serializedData;
    if(!artifact.SerializeToString(&serializedData)){
        ROS_ERROR_STREAM("ArtifactReporter::ReportArtifact(): Error serializing message\n" << artifact.DebugString());
    }
    //ROS_INFO_STREAM("The serialized data is " << serializedData);
    // Report the Artifact
    commsClient.SendTo(serializedData, subt::kBaseStationName);
}












