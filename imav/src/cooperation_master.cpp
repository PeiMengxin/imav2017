#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <sstream>
#include <stdlib.h>
#include <cmath>
#include <stdio.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/CommandHome.h>
#include <ros/param.h>
#include <iostream>
using namespace std;

std_msgs::String current_distance;
void get_uwb_disrance_cb(const std_msgs::String::ConstPtr &msg) //????
{
    current_distance = *msg;
}
int cooperation_control(int actual_distance) //???????
{
    //?????1800

    return 0;
}
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void get_local_pose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
    //????????
    // ROS_INFO("ori:%f %f %f %f", current_pose.pose.orientation.w, current_pose.pose.orientation.x,
    //         current_pose.pose.orientation.y, current_pose.pose.orientation.z);
    //???????
    // ROS_INFO("pos: %f %f %f", current_pose.pose.position.x, current_pose.pose.position.y,
    //         current_pose.pose.position.z);
}
//????
//filter

int med_filter_tmp_0[11];
int med_fil_cnt;
int Moving_Median(int width_num, int in)
{
    int tmp[11];
    int i, j;
    int t;

    if (width_num >= 10)
        return 0;
    else
    {
        if (++med_fil_cnt >= width_num)
            med_fil_cnt = 0;
        med_filter_tmp_0[med_fil_cnt] = in;
        for (i = 0; i < width_num; i++)
            tmp[i] = med_filter_tmp_0[i];
        // ROS_INFO_STREAM("tmp[0]= "<<tmp[0]);
        // ROS_INFO_STREAM("tmp[1]= "<<tmp[1]);
        // ROS_INFO_STREAM("tmp[2]= "<<tmp[2]);
        // ROS_INFO_STREAM("tmp[3]= "<<tmp[3]);
        //??????
        for (i = 0; i < width_num - 1; i++)
        {
            for (j = 0; j < (width_num - 1 - i); j++)
            {
                if (tmp[j] > tmp[j + 1])
                {
                    t = tmp[j];
                    tmp[j] = tmp[j + 1];
                    tmp[j + 1] = t;
                }
            }
        }
        return (tmp[(int)width_num / 2]); //??tmp[]???
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cooperation_master");
    ros::NodeHandle nh;
    ros::Subscriber get_uwb_distance_sub = nh.subscribe("uwb_distance_string", 1, get_uwb_disrance_cb);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher attitude_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_attitude/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient set_home_client = nh.serviceClient<mavros_msgs::CommandHome>("mavros/cmd/set_home");

    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, get_local_pose);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    for (int i = 20; ros::ok() && i > 0; --i)
    {
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();

    double exp_high = 3; //????
    ros::param::get("~exp_high", exp_high);
    double cooperation_master_speed;
    ros::param::get("~cooperation_master_speed",cooperation_master_speed);

    float high_err = 0;
    float kp_high = 0.2;

    float pid_out_high = 0;
    float q0, q1, q2, q3; //???
    float t12, t22, t31, t32, t33;
    float pitch_radian, roll_radian, yaw_radian;
    float yaw;

    while (ros::ok())
    {
        //????
        q0 = current_pose.pose.orientation.w;
        q1 = current_pose.pose.orientation.x;
        q2 = current_pose.pose.orientation.y;
        q3 = current_pose.pose.orientation.z;

        t32 = 2 * (q2 * q3 + q0 * q1);
        t31 = 2 * (q1 * q3 - q0 * q2);
        t33 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        t12 = 2 * (q1 * q2 - q0 * q3);
        t22 = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;

        pitch_radian = asin(t32);
        roll_radian = atan2(-t31, t33);
        yaw_radian = atan2(t12, t22); //????????-90????????-180?180?

        yaw = yaw_radian / 0.017453;
        //ROS_INFO("yaw:%f", yaw);
    
        high_err = exp_high - current_pose.pose.position.z;

        pid_out_high = kp_high * high_err;
            
        if (pid_out_high > 0.3 || pid_out_high < -0.3)
        {
            pid_out_high = 0.3 * pid_out_high / abs(pid_out_high);
        }

        // dis_err_last = dis_err;

        geometry_msgs::TwistStamped velocity_tw;
        velocity_tw.twist.linear.z = pid_out_high;
        velocity_tw.twist.linear.x = cos(yaw_radian) * cooperation_master_speed;
        velocity_tw.twist.linear.y = sin(-yaw_radian) * cooperation_master_speed;

        ROS_INFO("master velocity_tw:%f %f", velocity_tw.twist.linear.x, velocity_tw.twist.linear.y);

        velocity_pub.publish(velocity_tw); //??????

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}