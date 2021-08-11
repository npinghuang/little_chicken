#ifndef POSE_TRACKER_
#define POSE_TRACKER_

#include <vector>
#include <string>
#include <typeinfo>
#include <stdio.h>
#include <cmath>

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Pose2D.h>

enum class ActionMode {AIM, MOVING, ROTATE, STOP};
// enum class Strategy {angular, linear}; //for velocity_profile

class pose_tracker{
  public:
    pose_tracker();
    ~pose_tracker();
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr &goal);
    void get_now_pose();
    float quaternion_to_theta(float x, float y, float z,float w);
    bool reach_goal_xy();
    bool reach_goal_theta();
    void rotate(float g_theta, float n_theta);
    void linear();
    void set_line_func();//點到點求直線方程式
    float compensate_angular_vel();
    void give_velocity(float linear_vel, float rotate_vel);
    float linear_velocity_profile();
    float angular_velocity_profile(float difference_angle);

  private:
    ros::Publisher vel_pub_;
    ros::Subscriber goal_sub_;

    //param
    int control_freq=30;
    float xy_tolerance=0.05;
    float theta_tolerance=0.05;
    // float aim_tolerance=0.05;
    float Kd_=0.1;
    float Kt_=0.5;

    // float rotate_v=0.1;
    // float straight_v=0.1;
    float aim_theta;

    //velocity profile
    float max_vel=0.8;
    float temp_max_vel;
    float acc=0.5;
    float calculate_vel;
    float vel_threshold=0.01;
    int linear_state; // 0:triangle_curve   1:t_curve
    bool calculate_linear_distance;
    // bool calculate_angular_distance;
    float min_dist,desire_dist;//for follow line
    float desire_time,duration_time;

    //angular_velocity_profile
    float w_now = 0.0;
    float w_max = 2;
    float w_threshold = 0.01;
    float w_acc = 2;


    ros::Time start_calculate_time;

    float line_[3]; //a,b,c 

    geometry_msgs::Pose2D goal_pose_,now_pose_;

    ActionMode state;

};
#endif

