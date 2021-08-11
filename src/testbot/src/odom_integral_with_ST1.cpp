//版本：整合底盤、通訊、導航用

#include  <ros/ros.h>
//tf2必要
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/LinearMath/Transform.h"
//不確定用處
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
//tf2和odom用到的訊息格式
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int32MultiArray.h>
//額外的函式庫
#include <vector>
#include <typeinfo>
#include <stdio.h>

using namespace std;
double current_speed[3];

//msgs格式  v , omega , x , y , theta
void ST1_callback(const std_msgs::Int32MultiArray::ConstPtr& msg){ 
    current_speed[0] = float(msg->data[0])/1000;
    current_speed[1] = float(msg->data[1])/1000;
};


// void tf2_listener(){
//     tf2_ros::Buffer tfBuffer;
//     tf2_ros::TransformListener tfListener(tfBuffer);
//     geometry_msgs::TransformStamped transformStamped_listen;
//     try{
//         ROS_INFO("listener success");
//         transformStamped_listen= tfBuffer.lookupTransform("base_footprint", "odom",ros::Time(0));
//     }
//     catch (tf2::TransformException &ex) {
//         ROS_WARN("%s",ex.what());
//         ROS_INFO("in");
//         ros::Duration(1.0).sleep();  // sleep for one second
//         //continue;
//     }
// };

int main(int argc, char** argv){
    ros::init(argc, argv, "odom_integral");
    ros::NodeHandle n;
    ros::Subscriber ST1_sub = n.subscribe<std_msgs::Int32MultiArray>("rxST1",20, &ST1_callback) ;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    static tf2_ros::TransformBroadcaster br;
    //default
    double x = 0.8;                 // x
    double y = 0.3;                 // y
    double th = 0.0;              // theta
    double vx = 0.0;         
    double vy = 0.0;
    double vth = 0.0;          // 角速度
    double dt = 0.0;            // 積分時間

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate rate(200);
    while( n.ok() ){
        current_time = ros::Time::now();
        vx = current_speed[0];
        vy = current_speed[1];
        vth = current_speed[2];

        dt = (current_time - last_time).toSec();
        //ROS_INFO("time : %f", dt);
        //ROS_INFO("velocity : %f", vx);
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = current_time;
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_footprint";
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion odom_quat;
        odom_quat.setRPY(0, 0, th);
        transformStamped.transform.rotation.x = odom_quat.x();
        transformStamped.transform.rotation.y = odom_quat.y();
        transformStamped.transform.rotation.z = odom_quat.z();
        transformStamped.transform.rotation.w = odom_quat.w();
        br.sendTransform(transformStamped);

        //publish the odometry message 
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        tf2::Quaternion Temporary;
        geometry_msgs::Quaternion odom_quat_pub;
        Temporary.setRPY(0, 0, th);
        tf2::convert( Temporary , odom_quat_pub );
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat_pub;

        //set the velocity
        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);
        //tf2_listener();
        ros::spinOnce();
        last_time = ros::Time::now();
        rate.sleep();
    }
    return 0;
}