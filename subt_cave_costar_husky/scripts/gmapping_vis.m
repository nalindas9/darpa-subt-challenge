clc;
clear;
close all;
rosshutdown;

rosinit('localhost')

% load test_map.mat;

gmapping = rossubscriber('/map');
map_data = receive(gmapping);

width = map_data.Info.Width;
height = map_data.Info.Height;

raw_data = reshape(map_data.Data,[width, height])';

imshow(raw_data)


