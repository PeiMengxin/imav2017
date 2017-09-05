#include <ros/ros.h>
#include <ros/param.h>
#include <serial/serial.h>
#include <std_msgs/Byte.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string>
#include <sstream>
#include <iostream>
using namespace std;


serial::Serial uwb; //声明串口对象

void read_callback(const std_msgs::Byte::ConstPtr& msg)
{
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"get_uwb_distance_node");//初始化节点
    ros::NodeHandle nh;//声明节点句柄
    ros::Subscriber read_pub = nh.subscribe("read",1,read_callback);//订阅主题，并配置回调函数
    ros::Publisher pub_string = nh.advertise<std_msgs::String>("uwb_distance_string", 10);
    
    std::string uwb_port_name;
    ros::param::get("~uwb_port_name",uwb_port_name);
    
    try
    {
        uwb.setPort(uwb_port_name);/* code for Try */
        uwb.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        uwb.setTimeout(to);
        uwb.open();
    }
    catch (serial::IOException& e)
    {
       ROS_ERROR_STREAM("Unable to open port "); 
       return -1;  /* code for Catch */
    }
    //指定循环的频率 
    ros::Rate loop_rate(20); 
    //std::stringstream ss;
    while(ros::ok()) 
    { 
        //std_msgs::UInt8 distance;
        //distance.data = 0;
        //ss.clear();
        if(uwb.available())
        { 
            ROS_INFO_STREAM("Reading from serial port"); 
            std_msgs::String result; 
            result.data = uwb.read(uwb.available()); 
            ROS_INFO_STREAM("Read: " << result.data);
	    //ROS_INFO("Read: %d" << int(result.data[0]));
            //ss<<result.data;
            //ss>>distance.data; 
            //ROS_INFO_STREAM("pub: " << distance.data);
            //pub_uint8.publish(distance);
            pub_string.publish(result);
        } 

        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
  
    } 
    
}