#include "ros/ros.h"
#include "iostream"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <fstream>
#include <math.h>

using namespace std;
bool init_flag = true;

class Scan
{
public:
    double scan_velocity;
    double a_side_length;
    double b_side_length;
    double corner_radius;

public:
    Scan()
    {
        // the velocity commands for these three joints
        x_joint_pub_ = n_.advertise<std_msgs::Float64>("/machine_tool/x_joint_velocity_controller/command", 1000);
        y_joint_pub_ = n_.advertise<std_msgs::Float64>("/machine_tool/y_joint_velocity_controller/command", 1000);
        r_joint_pub_ = n_.advertise<std_msgs::Float64>("/machine_tool/r_joint_velocity_controller/command", 1000);
        // Topic you want to subscribe
        // sub_ = n_.subscribe("/desired_velocity", 1, &Scan::callback, this);
    }

    // void callback(const geometry_msgs::Twist &desried_vel)
    // {
    //     ROS_INFO("Linear Components: %f, %f", desried_vel.linear.x, desried_vel.linear.y);
    //     ROS_INFO("Angular Components: %f", desried_vel.angular.z);
    //     // do something with the input and generate the output
    // }
    void run()
    {
        ofstream something_great_in;
        something_great_in.open("v.txt", std::ios::trunc);
        std_msgs::Float64 x_joint_velocity_msgs;
        std_msgs::Float64 y_joint_velocity_msgs;
        std_msgs::Float64 r_joint_velocity_msgs;
        x_joint_velocity_msgs.data = 0.0;
        y_joint_velocity_msgs.data = 0.0;
        r_joint_velocity_msgs.data = 0.0;
        double theta = 0.0;
        double bias;
        double current_time;
        double timing;
        double last_timing;

        double corner_length = M_PI / 2 * corner_radius;

        double a_side_time = a_side_length / scan_velocity;
        double b_side_time = b_side_length / scan_velocity;
        double corner_time = corner_length / scan_velocity;
        double corner_angular_vel = M_PI / 2 / corner_time;

        double corner2center_tan = std::atan2(b_side_length, a_side_length);
        double corner2center_dis = std::sqrt(a_side_length * a_side_length / 4 + b_side_length * b_side_length / 4);
        double total_length = 2 * (2 * corner_length + a_side_length + b_side_length);
        double total_time = total_length / scan_velocity;
        double phase_timing[] = {a_side_time / 2,
                                 a_side_time / 2 + corner_time,
                                 a_side_time / 2 + corner_time + b_side_time,
                                 a_side_time / 2 + 2 * corner_time + b_side_time,
                                 3 * a_side_time / 2 + 2 * corner_time + b_side_time,
                                 3 * a_side_time / 2 + 3 * corner_time + b_side_time,
                                 3 * a_side_time / 2 + 3 * corner_time + 2 * b_side_time,
                                 3 * a_side_time / 2 + 4 * corner_time + 2 * b_side_time,
                                 total_time};

        while (timing <= total_time && n_.ok())
        {
            if (init_flag || bias < 0.001)
            {
                bias = ros::Time::now().toSec();
                init_flag = false;
            }
            current_time = ros::Time::now().toSec();
            timing = current_time - bias;
            if (0 < timing && timing <= phase_timing[0])
            {
                y_joint_velocity_msgs.data = -scan_velocity;
            }
            if (phase_timing[0] < timing && timing <= phase_timing[1])
            {
                theta = (timing - phase_timing[0]) * corner_angular_vel;
                x_joint_velocity_msgs.data = corner2center_dis * cos(theta + corner2center_tan) * corner_angular_vel;
                y_joint_velocity_msgs.data = corner2center_dis * sin(theta + corner2center_tan) * corner_angular_vel;
                r_joint_velocity_msgs.data = corner_angular_vel;
            }
            if (phase_timing[1] < timing && timing <= phase_timing[2])
            {
                x_joint_velocity_msgs.data = 0.0;
                y_joint_velocity_msgs.data = -scan_velocity;
                r_joint_velocity_msgs.data = 0.0;
            }
            if (phase_timing[2] < timing && timing <= phase_timing[3])
            {
                theta = (timing - phase_timing[2]) * corner_angular_vel + M_PI / 2;
                x_joint_velocity_msgs.data = corner2center_dis * cos(theta - corner2center_tan) * corner_angular_vel;
                y_joint_velocity_msgs.data = corner2center_dis * sin(theta - corner2center_tan) * corner_angular_vel;
                r_joint_velocity_msgs.data = corner_angular_vel;
            }
            if (phase_timing[3] < timing && timing <= phase_timing[4])
            {
                x_joint_velocity_msgs.data = 0.0;
                y_joint_velocity_msgs.data = -scan_velocity;
                r_joint_velocity_msgs.data = 0.0;
            }
            if (phase_timing[4] < timing && timing <= phase_timing[5])
            {
                theta = (timing - phase_timing[4]) * corner_angular_vel;
                x_joint_velocity_msgs.data = corner2center_dis * cos(theta + corner2center_tan) * corner_angular_vel;
                y_joint_velocity_msgs.data = corner2center_dis * sin(theta + corner2center_tan) * corner_angular_vel;
                r_joint_velocity_msgs.data = corner_angular_vel;
            }
            if (phase_timing[5] < timing && timing <= phase_timing[6])
            {
                x_joint_velocity_msgs.data = 0.0;
                y_joint_velocity_msgs.data = -scan_velocity;
                r_joint_velocity_msgs.data = 0.0;
            }
            if (phase_timing[6] < timing && timing <= phase_timing[7])
            {
                theta = (timing - phase_timing[6]) * corner_angular_vel + M_PI / 2;
                x_joint_velocity_msgs.data = corner2center_dis * cos(theta - corner2center_tan) * corner_angular_vel;
                y_joint_velocity_msgs.data = corner2center_dis * sin(theta - corner2center_tan) * corner_angular_vel;
                r_joint_velocity_msgs.data = corner_angular_vel;
            }
            if (phase_timing[7] < timing && timing <= total_time)
            {
                x_joint_velocity_msgs.data = 0.0;
                y_joint_velocity_msgs.data = -scan_velocity;
                r_joint_velocity_msgs.data = 0.0;
            }
            // cout << "bias: " << bias << ", current_time: " << current_time << ", timing: " << timing << endl;
            // cout << "x_joint_velocity_msgs: " << x_joint_velocity_msgs.data << "\n"
            //      << "y_joint_velocity_msgs: " << y_joint_velocity_msgs.data << "\n"
            //      << "r_joint_velocity_msgs: " << r_joint_velocity_msgs.data << "\n"
            //      << endl;
            if (last_timing != timing)
            {
                something_great_in << "timing: " << timing << "\t"
                                   << "x: " << x_joint_velocity_msgs.data << "\t"
                                   << "y: " << y_joint_velocity_msgs.data << "\t"
                                   << "r: " << r_joint_velocity_msgs.data << "\t"
                                   << "\n";
                x_joint_pub_.publish(x_joint_velocity_msgs);
                y_joint_pub_.publish(y_joint_velocity_msgs);
                r_joint_pub_.publish(r_joint_velocity_msgs);
                last_timing = timing;
            }
            sleep(0.001);
        }

        x_joint_velocity_msgs.data = 0.0;
        y_joint_velocity_msgs.data = 0.0;
        r_joint_velocity_msgs.data = 0.0;
        x_joint_pub_.publish(x_joint_velocity_msgs);
        y_joint_pub_.publish(y_joint_velocity_msgs);
        r_joint_pub_.publish(r_joint_velocity_msgs);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher x_joint_pub_;
    ros::Publisher y_joint_pub_;
    ros::Publisher r_joint_pub_;
    // ros::Subscriber sub_;

}; // End of class SubscribeAndPublish

int main(int argc, char **argv)
{

    // initialization
    ros::init(argc, argv, "machine_tool_scan_process");
    Scan scan;

    ros::AsyncSpinner spinner(7);
    spinner.start();
    scan.a_side_length = 0.118;
    scan.b_side_length = 0.042;
    scan.corner_radius = 0.015;
    scan.scan_velocity = 0.02;
    scan.run();

    ros::waitForShutdown();
    return 0;
}
