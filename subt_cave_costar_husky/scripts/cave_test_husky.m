clc;
clear;
close all;
rosshutdown;

rosinit('localhost')

% camera = rossubscriber('/X1/front/image_raw');
X1 = rospublisher('/COSTAR_HUSKY/cmd_vel');
velmsg = rosmessage(X1);
odom = rossubscriber('/COSTAR_HUSKY/odom');
laser = rossubscriber('/COSTAR_HUSKY/front_cliff_scan');
gmapping = rossubscriber('/map');

pose = receive(odom);
[x0,y0,theta0] = odometry(pose);
global pose0
pose0 = [x0,y0,theta0];

% global x_length y_length resolution map
% x_length = 100;    % meter
% y_length = 100;
% resolution = 5;   % grid/meter
% map = robotics.OccupancyGrid(x_length,y_length,resolution);
% map.GridLocationInWorld=[-x_length/2,-y_length/2];
% map.OccupiedThreshold=0.75;

vfh = robotics.VectorFieldHistogram;
vfh.NumAngularSectors = 180;
vfh.DistanceLimits = [0 5];
vfh.RobotRadius = 0.6;
vfh.MinTurningRadius = 0.5;
vfh.SafetyDistance = 0.2;
vfh.HistogramThresholds = [1 1];


%%
% mapping
rate = robotics.Rate(20);
reset(rate)
% figure;
while rate.TotalElapsedTime < 40
    
    pose = receive(odom);
    laserScan = receive(laser);
    camera_latest = receive(camera);
    
%     occval=mapping2(laserScan,pose);
    
%     img = getCamera(camera_latest);
    
%     subplot(2,1,1)
%     imshow(img)
    
    % Get laser scan data
    ranges = double(laserScan.Ranges);
    angles = double(laserScan.readScanAngles);
    
    % Create a lidarScan object from the ranges and angles
    scan = lidarScan(ranges,angles);
    
    % Call VFH object to computer steering direction
    targetDir = 0;
    steerDir = vfh(ranges, angles, targetDir);
    
    % Calculate velocities
    if ~isnan(steerDir) % If steering direction is valid
        desiredV = 0.5;
        w = exampleHelperComputeAngularVelocity(steerDir, 1);
    else % Stop and search for valid direction
        desiredV = 0.0;
        w = 0.5;
    end
    
    % Assign and send velocity commands
    velmsg.Linear.X = desiredV;
    velmsg.Angular.Z = w;
    
    send(X1,velmsg);
    
%     subplot(2,1,2)
%     show(map)
    waitfor(rate);
end

velmsg.Linear.X = 0;
velmsg.Angular.Z = 0;
send(X1,velmsg);

map_data = receive(gmapping);

% occval=fliplr(occval');
% index=occval==0;
% occval(occval==1)=0;
% occval(index)=1;
% [row,col] = find(occval == 0);
% inflate(map,0.2);


