//版本：整合底盤、通訊、導航用
//用處：把cmd_vel轉換成txST1的topic
#include <ros/ros.h>
#include <typeinfo>
#include <stdio.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

using namespace std;
float current_speed[2] = {0.0,0.0};
float ST1_x = -100000.0;
float ST1_y = -100000.0;
float ST1_theta = -100000.0;


void cmd_callback(const geometry_msgs::Twist &twist_aux){
    //ROS_INFO("in to call back");
    current_speed[0]= twist_aux.linear.x;
    current_speed[1]= twist_aux.angular.z;
};
void ST1_callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    ST1_x = msg->data[2]/1000000.;
    ST1_y = msg->data[3]/1000000.;
    ST1_theta = msg->data[4]/1000000.;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "planner_to_ST");
    ros::NodeHandle n;
    ros::NodeHandle nh_lcoal("~");
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel",1, &cmd_callback) ;
    ros::Subscriber ST1_sub = n.subscribe("rxST1",1, &ST1_callback) ;
    ros::Publisher txST1 = n.advertise<std_msgs::Int32MultiArray>("txST1", 1);
    std_msgs::Int32MultiArray toST_info;


    bool initial_state = true;
    float p_init_x;
    float p_init_y;
    float p_init_yaw;

    nh_lcoal.param<float>("init_x", p_init_x, 1.0);
    nh_lcoal.param<float>("init_y", p_init_y, 1.5);
    nh_lcoal.param<float>("init_yaw", p_init_yaw, 0.0);
    float l = 0.17145;// 小雞軸距 : m

    //default
    float last_vel = 0.0;
    float last_avel = 0.0;
    ros::Time last,now;
                
    ros::Rate rate(200);
    while(ros::ok()){
      ros::spinOnce();
      toST_info.data.clear();
      if(initial_state){
          // ROS_INFO_STREAM("QQQQQQQQ : "<< ST1_x);
          // ROS_INFO_STREAM("QQQQQQQQ : "<< ST1_y);
          // ROS_INFO_STREAM("QQQQQQQQ : "<< ST1_theta);
          if ( fabs(p_init_x-ST1_x) <= 0.005 && fabs(p_init_y-ST1_y)<=0.005 && fabs(p_init_yaw-ST1_theta)<=0.01){
              ROS_INFO("initial state over");
              initial_state = false;
              p_init_x = -2000;
              p_init_y = -3000;
          }
          else{
            last = ros::Time::now();
          }   
      }
      for(int i = 0;i<2;i++){
          toST_info.data.push_back(int(current_speed[i]*1000));
      }
      toST_info.data.push_back(int(p_init_x *1000));
      toST_info.data.push_back(int(p_init_y *1000));
      toST_info.data.push_back(int(p_init_yaw *1000));
      /* check wheel velocity */
      // toST_info.data.push_back(int((current_speed[0]+l*current_speed[1])*1000) );
      // toST_info.data.push_back(int((current_speed[0]-l*current_speed[1])*1000));

      /* check acceleration */
      now = ros::Time::now();
      int acc_over = 0;
      int aacc_over = 0;
      float duration = (now-last).toSec();
      if( (current_speed[0]-last_vel)/duration >= 0.3){
        acc_over = 1;
        ROS_INFO_STREAM("acc_over : "<<(current_speed[0]-last_vel)/duration);
      }
      if( (current_speed[1]-last_avel)/duration >= 0.5){
        aacc_over = 1;
        ROS_INFO_STREAM("angular_acc_over : "<<(current_speed[1]-last_avel)/duration);
      }
      // toST_info.data.push_back(acc_over);
      // toST_info.data.push_back(aacc_over);
      
      last = now;
      last_vel = current_speed[0];
      last_avel = current_speed[1];
      /*----------------------*/


      txST1.publish(toST_info);
      rate.sleep(); 
    }
    return 0;
}