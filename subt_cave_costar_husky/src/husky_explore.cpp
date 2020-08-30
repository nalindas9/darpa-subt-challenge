#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <math.h>

geometry_msgs::Twist vel_command;

float cliff_range = 0.5;
std::vector<float> lidar_range = {0.0};
std::vector<float> lidar_angle = {0.0};

//*****************  VFH params *****************
int num_sector = 180;
float c = 1.0;
float a = 5.0;
float b = 2.0; // larger b allows closer obstacles
float bin_threshold = 4.5;
int len_threshold = 5;
//***********************************************

int RECOVERY = 0;

std::vector<float> calc_magnitude()
{
    std::vector<float> magnitude;
    for (int i = 0; i < lidar_range.size(); i++)
    {
        magnitude.push_back(c * c * (a - b * (lidar_range[i])));
    }
    // hole detection
    if (cliff_range > 1.0)
    {
        std::cout << "hole detected" << std::endl;
        RECOVERY = 1;
        for (int i = 0; i < len_threshold / 2; i++)
        {
            magnitude[round(num_sector / 2) + i] = a;
            magnitude[round(num_sector / 2) - i] = a;
        }
    }
    else if (cliff_range < 0.4)
    {
        std::cout << "rock detected" << std::endl;
        RECOVERY = 1;
        for (int i = 0; i < len_threshold / 2; i++)
        {
            magnitude[round(num_sector / 2) + i] = a;
            magnitude[round(num_sector / 2) - i] = a;
        }
    }
    return magnitude;
}

std::vector<int> calc_histogram(std::vector<float> mag)
{
    std::vector<float> histogram;
    std::vector<int> hist_bin;
    int sector_interval = round(mag.size() / num_sector);
    for (int i = 0; i < num_sector; i++)
    {
        float hist = 0;
        for (int j = 0; j < sector_interval; j++)
        {
            hist += mag[i * sector_interval + j];
        }
        histogram.push_back(hist);
    }
    //binarization
    for (int i = 0; i < histogram.size(); i++)
    {
        if (histogram[i] > bin_threshold)
            hist_bin.push_back(1);
        else
            hist_bin.push_back(0);
    }

    return hist_bin;
}

int select_sector(std::vector<int> hist_bin)
{
    int curr_len = 0;
    bool isValley = false;
    std::vector<int> candidates;
    int mid = round(hist_bin.size() / 2);
    int valley_idx = mid;
    int sum = 0; // count number of 0 sectors
    // find & store all admissible valleys
    for (int i = 0; i < hist_bin.size(); i++)
    {
        if (hist_bin[i] == 0 && !isValley)
        {
            isValley = true;
        }
        if ((hist_bin[i] == 1 && isValley) || (i == hist_bin.size() - 1))
        {
            isValley = false;
            if (curr_len >= len_threshold)
            {
                candidates.push_back(i - round(curr_len / 2));
            }
            curr_len = 0;
        }
        if (isValley)
        {
            curr_len++;
        }
        sum += hist_bin[i];
    }
    if ((num_sector - sum) >= len_threshold)
    {
        // find the valley needs minimum turning
        int dist = 1000;
        for (int i = 0; i < candidates.size(); i++)
        {
            if (abs(candidates[i] - mid) < dist)
            {
                valley_idx = candidates[i];
                dist = abs(candidates[i] - mid);
            }
        }
    }
    else
    {
        // no valley detected
        valley_idx = -1;
        RECOVERY = 1;
    }
    return valley_idx;
}

void vel_control(int valley_idx)
{
    float kp = 0.022;
    float error = valley_idx - num_sector / 2;
    float ang_vel = kp * error;
    // min angular velocity
    if (ang_vel > 0 && ang_vel <= 0.1)
        ang_vel = 0.1;
    if (ang_vel < 0 && ang_vel >= -0.1)
        ang_vel = -0.1;
    vel_command.linear.x = 0.3;
    vel_command.angular.z = ang_vel;
}

void vfh()
{
    std::vector<float> mag = calc_magnitude();
    std::vector<int> hist_bin = calc_histogram(mag);
    int valley_idx = select_sector(hist_bin);
    vel_control(valley_idx);

    // for debugging
    std::cout << "cliff scan: " << cliff_range << std::endl;
    std::cout << "histogram:\n";
    for (int i = 0; i < hist_bin.size(); i++)
        std::cout << hist_bin[i];
    std::cout << "\nval_idx:" << valley_idx << " ang_vel:" << vel_command.angular.z << std::endl;
}

void view_lidar()
{
    std::cout << "START ANGLES" << std::endl;
    for (int i = 0; i < lidar_angle.size(); i++)
    {
        std::cout << lidar_angle[i] << " ";
    }
    std::cout << "END ANGLES" << std::endl;

    std::cout << "START RANGES" << std::endl;
    for (int i = 0; i < lidar_range.size(); i++)
    {
        std::cout << lidar_range[i] << " ";
    }
    std::cout << "END RANGES" << std::endl;

    std::cout << "total lasers:" << lidar_range.size() << std::endl;
}

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    lidar_range = msg->ranges;
    float beams = msg->angle_min;
    while (beams <= msg->angle_max)
    {
        lidar_angle.push_back(beams);
        beams += msg->angle_increment;
    }
    // view_lidar();
}

void cliff_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    cliff_range = msg->ranges.front();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "husky_explore");

    ros::NodeHandle n;
    ros::Subscriber lidar_sub = n.subscribe("COSTAR_HUSKY/front_scan", 1, lidar_callback);
    ros::Subscriber cliff_sub = n.subscribe("COSTAR_HUSKY/front_cliff_scan", 1, cliff_callback);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("COSTAR_HUSKY/cmd_vel", 10);

    ROS_INFO("start exploration");

    ros::Rate loop_rate(40);
    int backward_counter = 0;
    int spin_counter = 0;
    std::cout << RECOVERY << std::endl;
    while (n.ok())
    {
        ros::spinOnce();
        if (RECOVERY == 0)
        {
            vfh();
        }
        else
        {
            // recovery behavior
            if (backward_counter < 50)
            {
                vel_command.linear.x = -0.2;
                vel_command.angular.z = 0.0;
                backward_counter++;
            }
            else
            {
                if (spin_counter < 100)
                {
                    // use lidar distance to determine spin left or right
                    float left_sum = 0;
                    float right_sum = 0;
                    for (auto beam = lidar_range.end(); beam > lidar_range.end() - 80; beam--)
                        left_sum += *beam;
                    for (auto beam = lidar_range.begin(); beam < lidar_range.begin() + 80; beam++)
                        right_sum += *beam;
                    // if (lidar_range.back() > lidar_range.front())
                    if (left_sum > right_sum)
                        vel_command.angular.z = 0.3;
                    else
                        vel_command.angular.z = -0.3;
                    vel_command.linear.x = 0.0;
                    spin_counter++;
                }
                else
                {
                    backward_counter = 0;
                    spin_counter = 0;
                    RECOVERY = 0;
                }
            }
        }
        vel_pub.publish(vel_command);
        loop_rate.sleep();
    }

    // stop robot
    for (int i = 0; i < 10; i++)
    {
        vel_command.linear.x = 0.0;
        vel_command.angular.z = 0.0;
        vel_pub.publish(vel_command);
    }

    return 0;
}