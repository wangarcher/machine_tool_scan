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
using namespace std;

double set_velocity = -0.02; // 20mm/s
double set_time = 20.712;    // with the total length of 414.24mm
bool init_flag = true;

class Scan
{
public:
    Scan()
    {
        // the velocity commands for these three joints
        x_joint_pub_ = n_.advertise<std_msgs::Float64>("/x_joint_velocity_controller/command", 1000);
        y_joint_pub_ = n_.advertise<std_msgs::Float64>("/y_joint_velocity_controller/command", 1000);
        z_joint_pub_ = n_.advertise<std_msgs::Float64>("/z_joint_velocity_controller/command", 1000);
        r_joint_pub_ = n_.advertise<std_msgs::Float64>("/r_joint_velocity_controller/command", 1000);
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
        std_msgs::Float64 x_joint_velocity_msgs;
        std_msgs::Float64 y_joint_velocity_msgs;
        std_msgs::Float64 z_joint_velocity_msgs;
        std_msgs::Float64 r_joint_velocity_msgs;
        x_joint_velocity_msgs.data = 0.0;
        y_joint_velocity_msgs.data = 0.0;
        z_joint_velocity_msgs.data = 0.0;
        r_joint_velocity_msgs.data = 0.0;
        double bias;
        double current_time;
        double timing;
        double last_timing;

        while (n_.ok())
        {
            if (init_flag || bias < 0.001)
            {
                bias = ros::Time::now().toSec();
                init_flag = false;
            }
            current_time = ros::Time::now().toSec();
            timing = current_time - bias;

            x_joint_velocity_msgs.data = 0.1;
            y_joint_velocity_msgs.data = 0.1;
            r_joint_velocity_msgs.data = 1;
            z_joint_velocity_msgs.data = 10 * cos(30 * timing);

            cout << "bias: " << bias << ", current_time: " << current_time << ", timing: " << timing << endl;
            cout << "x_joint_velocity_msgs: " << x_joint_velocity_msgs.data << "\n"
                 << "y_joint_velocity_msgs: " << y_joint_velocity_msgs.data << "\n"
                 << "z_joint_velocity_msgs: " << z_joint_velocity_msgs.data << "\n"
                 << "r_joint_velocity_msgs: " << r_joint_velocity_msgs.data << "\n"
                 << endl;
            if (last_timing != timing)
            {
                // something_great_in << "timing: " << timing << "\t"
                //                    << "x: " << x_joint_velocity_msgs.data << "\t"
                //                    << "y: " << y_joint_velocity_msgs.data << "\t"
                //                    << "r: " << r_joint_velocity_msgs.data << "\t"
                //                    << "\n";
                x_joint_pub_.publish(x_joint_velocity_msgs);
                y_joint_pub_.publish(y_joint_velocity_msgs);
                z_joint_pub_.publish(z_joint_velocity_msgs);
                r_joint_pub_.publish(r_joint_velocity_msgs);
                last_timing = timing;
            }
            sleep(0.001);
        }

        x_joint_velocity_msgs.data = 0.0;
        y_joint_velocity_msgs.data = 0.0;
        z_joint_velocity_msgs.data = 0.0;
        r_joint_velocity_msgs.data = 0.0;
        x_joint_pub_.publish(x_joint_velocity_msgs);
        y_joint_pub_.publish(y_joint_velocity_msgs);
        z_joint_pub_.publish(z_joint_velocity_msgs);
        r_joint_pub_.publish(r_joint_velocity_msgs);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher x_joint_pub_;
    ros::Publisher y_joint_pub_;
    ros::Publisher z_joint_pub_;
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

    scan.run();

    ros::waitForShutdown();
    return 0;
}
