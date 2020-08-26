function [odom_pos_x, odom_pos_y, odom_theta] = odometry(pose)

odom_pos_x = pose.Pose.Pose.Position.X;
odom_pos_y = pose.Pose.Pose.Position.Y;

quat = [pose.Pose.Pose.Orientation.W, ...
        pose.Pose.Pose.Orientation.X, ...
        pose.Pose.Pose.Orientation.Y, ...
        pose.Pose.Pose.Orientation.Z];
odom_theta = quat2eul(quat);

odom_theta = odom_theta(1);
