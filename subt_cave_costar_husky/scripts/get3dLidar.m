clc;
clear;
close all;
rosshutdown;

rosinit('localhost')

X1 = rospublisher('/COSTAR_HUSKY/cmd_vel');
velmsg = rosmessage(X1);
odom = rossubscriber('/COSTAR_HUSKY/odom');
laser = rossubscriber('/COSTAR_HUSKY/points');

msg = receive(laser);
points = msg.Data;

xyz = readXYZ(msg);

n = 5;
w = msg.Width;
single_layer = xyz((n-1)*w+1:n*w,:);
% plot3(xyz_3(:,1), xyz_3(:,2), xyz_3(:,3), '.')
% scatter3(xyz(:,1), xyz(:,2), xyz(:,3), '.')
plot(single_layer(:,1), single_layer(:,2), '.')