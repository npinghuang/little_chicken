#ifndef POSE_TRACKER_2_
#define POSE_TRACKER_2_

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
#include <nav_msgs/Odometry.h>

// enum class Strategy {angular, linear}; 
enum class ActionMode {AIM, MOVING, ROTATE, STOP, EMERGENCY};

class Tolerance{
  public:
    int number;
    float xy_tolerance;
    float theta_tolerance; 
};

class pose_tracker{
  public:
    pose_tracker();
    ~pose_tracker();
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr &goal);
    void get_now_pose(const nav_msgs::Odometry::ConstPtr &now_pose);
    float quaternion_to_theta(float x, float y, float z,float w);
    float theta_converter(float theta);//convert to 0 ~ 2pi
    float rotate_direct_discriminate(float g_theta, float n_theta);

    bool reach_goal_xy(float g_x, float g_y, float n_x, float n_y);
    bool reach_goal_theta(float g_theta, float n_theta);

    void rotate(float g_theta, float n_theta);
    void linear(float g_x, float g_y, float n_x, float n_y);
    void emergency_dec();

    void set_line_func();//點到點求直線方程式
    float compensate_angular_vel();
    void give_velocity(float linear_vel, float rotate_vel);
    float linear_velocity_profile(float difference_distance);
    float angular_velocity_profile(float difference_angle);

  private:
    ros::Publisher vel_pub_;
    ros::Subscriber goal_sub_,odom_sub_;
    ros::Publisher to_main,check_point;

    // bool fuck_you_do_not_wait = false;

    //param
    bool re_plan;
    float control_freq;
    float xy_tolerance;
    float theta_tolerance;
    float xy_tolerance_default;
    float theta_tolerance_default;
    // float aim_tolerance=0.05;

    //for plan state pub
    bool ifpub_state;
    int plan_number;

    //for backward_strategy
    bool backward;
    float backward_tolerance;
    float theta_;//if backward -> now_pose_.theta-2pi
    bool redecide_line;

    //for rotate_state
    int rotate_state; // 0->正轉 1->反轉 
    int last_rotate_state;// 2->待機
    //for linear state
    float desire_dist;
    float initial_pose[2];
    int overshoot_state;

    //for straight line compensate
    float deviate_distance;
    bool set_line;
    float line_[3]; //a,b,c 
    float Kd_;
    float Kt_;
    float deviate_angle;

    float aim_theta;

    //for both profile
    bool decelerate_mode;
    ros::Time last_time, current_time;

    //for special strategy
    bool latch_xy;
    bool linear_compensate;
    bool overshoot_compensate;

    //velocity
    bool reset_v;
    float v_now;
    float v_max;
    float v_threshold;
    float v_acc;
    float v_dacc;//減速
    float v_dacc_e;//emergency
    float v_recovery;//過衝用
    float v_dacc_tolerance;

    //angular_velocity
    bool reset_w;
    float w_now;
    float w_max;
    float w_threshold;
    float w_acc;
    float w_dacc;//減速
    float w_dacc_e;//emergency
    float w_recovery;//過衝用

    geometry_msgs::Pose2D goal_pose_,now_pose_;

    //to main
    std_msgs::Int32MultiArray to_main_msgs;
    int msg_count;
    bool reach_goal_;

    std::string base_footprint_name;

    //for dynamic tolerance ------ input : point_number, xy_tolerance, theta_tolerance
    XmlRpc::XmlRpcValue tolerance_input;
    std::vector<Tolerance> tolerance_list; 

    ActionMode state,last_state;

};
#endif

