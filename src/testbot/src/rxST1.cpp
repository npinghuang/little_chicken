//版本：整合底盤、通訊、導航用
#include <ros/ros.h>
#include <stdio.h>
#include <std_msgs/Int32MultiArray.h>


int main(int argc, char **argv){
    ros::init(argc, argv, "rxST1");
    ros::NodeHandle n;
    ros::Publisher rxST1 = n.advertise<std_msgs::Int32MultiArray>("rxST1", 1000);
    std_msgs::Int32MultiArray rx_;
    rx_.data = {1234,1000,2000,800,200,3000};
    while(ros::ok()){
        //ROS_INFO("%d",rx_.data[0]);
        rxST1.publish(rx_);  
    }
    ros::spinOnce();
    return 0;
}