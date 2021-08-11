#include <ros/ros.h>
#include <simple_version_navigation/pose_tracker_2.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/LinearMath/Transform.h"
#include <geometry_msgs/Pose2D.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

using namespace std;

pose_tracker::pose_tracker(){
  ros::NodeHandle n;
  vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  goal_sub_ =  n.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal",1,&pose_tracker::goalCB,this);
  odom_sub_ =  n.subscribe<nav_msgs::Odometry>("odom",1,&pose_tracker::get_now_pose,this);
  to_main = n.advertise<std_msgs::Int32MultiArray>("plan_state",50);
  check_point = n.advertise<geometry_msgs::Pose2D>("check_point",1);
  
  //initial param_value
  n.param<float>("control_freq", control_freq, 30.0);
  n.param<float>("xy_tolerance", xy_tolerance, 0.01);
  n.param<float>("theta_tolerance", theta_tolerance, 0.01);
  n.param<float>("backward_tolerance", backward_tolerance, 0.0);

  n.param<float>("deviate_distance", deviate_distance, 0.15);
  n.param<float>("deviate_angle", deviate_angle, 0.2);
  n.param<float>("Kd", Kd_, 0.1);
  n.param<float>("Kt", Kt_, 0.5);

  n.param<bool>("latch_xy", latch_xy, true);//make sure rotate will not consider xy
  n.param<bool>("linear_compensate", linear_compensate, true);
  n.param<bool>("overshoot_compensate", overshoot_compensate, true);

  n.param<float>("max_linear_velocity", v_max, 0.5);
  n.param<float>("v_threshold", v_threshold, 0.05);
  n.param<float>("velocity_acc", v_acc, 0.1);
  n.param<float>("velocity_deceleration", v_dacc, 0.1);
  n.param<float>("velocity_deceleration_emergency", v_dacc_e, 0.5);
  n.param<float>("v_recovery", v_recovery, 0.05);

  n.param<float>("max_angular_velocity", w_max, 1.2);
  n.param<float>("w_threshold", w_threshold, 0.1);
  n.param<float>("angular_velocity_acc", w_acc, 0.8);
  n.param<float>("angular_velocity_deceleration", w_dacc, 0.8);
  n.param<float>("angular_deceleration_emergency", w_dacc_e, 1.2);
  n.param<float>("w_recovery", w_recovery, 0.1);

  n.param<string>("base_footprint_name", base_footprint_name, "base_foorprint");

  ROS_INFO_STREAM("max_linear_velocity :"<<v_max);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ROS_INFO("nononon");
  while ( !(tfBuffer.canTransform("map",base_footprint_name,ros::Time(0),ros::Duration(0.5))) ){
    ROS_INFO("wait for TF");//等tf被發布出來
  }


  //initial value
  state = ActionMode::STOP;
  re_plan = false;
  v_now = 0.0;
  w_now = 0.0;
  msg_count = 0;
  reach_goal_ = false;
  ifpub_state = false;

  ros::Rate rate(control_freq);
  while (ros::ok() ){
  
    ros::spinOnce();

    //set backward_strategy    
    if(re_plan){
      aim_theta = atan2(goal_pose_.y - now_pose_.y, goal_pose_.x - now_pose_.x);
      aim_theta = theta_converter(aim_theta);
      float forward_stra_angle =  rotate_direct_discriminate(aim_theta, now_pose_.theta)
                                + rotate_direct_discriminate(goal_pose_.theta, aim_theta);

      float backward_stra_angle =  rotate_direct_discriminate(aim_theta, theta_converter(now_pose_.theta-M_PI) )
                              + rotate_direct_discriminate(goal_pose_.theta, theta_converter(aim_theta-M_PI) );                    

      if ( backward_stra_angle+backward_tolerance <= forward_stra_angle ){
        backward = true;
      }
      else{
        backward = false;
      }
    }
      
    if (backward){
      theta_ = theta_converter(now_pose_.theta-M_PI);
    }
    else{
      theta_ = now_pose_.theta;
    }
    
    if ( re_plan ){
      if ( reach_goal_xy(goal_pose_.x, goal_pose_.y, now_pose_.x, now_pose_.y) ){
        if ( reach_goal_theta(goal_pose_.theta, now_pose_.theta) ){
          ROS_INFO("reach goal");
          reach_goal_ = true;
          state = ActionMode::STOP;
          check_point.publish(now_pose_);
          // give_velocity(0.0, 0.0);
        }
        else{
          state = ActionMode::ROTATE;
        }
      }
      else{       
        if ( reach_goal_theta(aim_theta, theta_) ){
          initial_pose[0] = now_pose_.x;
          initial_pose[1] = now_pose_.y;
          desire_dist = sqrt( pow((goal_pose_.x-now_pose_.x),2) + pow((goal_pose_.y-now_pose_.y),2) );
          state = ActionMode::MOVING;
        }
        else{
          state = ActionMode::AIM;
        }
      }
      re_plan = false;
    }

    //切換狀態時 重設部份變數
    if (last_state != state){
      last_time = ros::Time::now();
      set_line = true;
      reset_w = true;
      reset_v = true;
      decelerate_mode = false;
      last_rotate_state = 2;
    }

    if( ifpub_state){
      msg_count += 1;
      if(reach_goal_){
        to_main_msgs.data.clear();
        to_main_msgs.data.push_back(1);
        to_main_msgs.data.push_back(0);
        to_main_msgs.data.push_back(msg_count);
        to_main.publish(to_main_msgs);
        ifpub_state = false;
      }
      else{
        to_main_msgs.data.clear();
        to_main_msgs.data.push_back(0);
        to_main_msgs.data.push_back(0);
        to_main_msgs.data.push_back(msg_count);
        to_main.publish(to_main_msgs);
      }
    }

    switch (state)
    {
      case ActionMode::AIM:
        // ROS_INFO(" aim ");
        if( reset_w ){
          w_now = 0.0;
          reset_w = false;
        }
        rotate(aim_theta, theta_);
        break;
      case ActionMode::MOVING:
        ROS_INFO(" moving ");
        if( set_line ){
          set_line_func();
          set_line = false;
        }
        if( reset_v ){
          v_now = 0.0;
          reset_v = false;
        }
        linear(goal_pose_.x, goal_pose_.y, now_pose_.x, now_pose_.y);
        break;
      case ActionMode::ROTATE:
        // ROS_INFO(" rotate ");
        if( reset_w ){
          w_now = 0.0;
          reset_w = false;
        }
        if( latch_xy ){
          goal_pose_.x = now_pose_.x;
          goal_pose_.y = now_pose_.y;
        }
        rotate(goal_pose_.theta, now_pose_.theta);
        break;
      case ActionMode::EMERGENCY:
        ROS_INFO(" emergency decceleration ");
        emergency_dec();
        break;
      case ActionMode::STOP:
        ROS_INFO(" stop ");
        give_velocity(0.0, 0.0);
        break;
    }

    last_state = state;
    ROS_INFO("--------------------------");
    rate.sleep();
  }
}

void pose_tracker::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goal){
  // goal_pose_ = *goal;
  goal_pose_.x =  goal->pose.position.x;
  goal_pose_.y =  goal->pose.position.y;
  goal_pose_.theta = quaternion_to_theta(goal->pose.orientation.x, goal->pose.orientation.y,
                                         goal->pose.orientation.z, goal->pose.orientation.w);
  goal_pose_.theta = theta_converter(goal_pose_.theta);
  re_plan = true;
  reach_goal_ = false;
  ifpub_state = true;
  ROS_INFO("get new goal");
}

void pose_tracker::get_now_pose(const nav_msgs::Odometry::ConstPtr &now_pose){
  now_pose_.x = now_pose->pose.pose.position.x;
  now_pose_.y = now_pose->pose.pose.position.y;
  now_pose_.theta = quaternion_to_theta(now_pose->pose.pose.orientation.x, now_pose->pose.pose.orientation.y,
                                        now_pose->pose.pose.orientation.z, now_pose->pose.pose.orientation.w);
  now_pose_.theta = theta_converter(now_pose_.theta);
}

float pose_tracker::quaternion_to_theta(float x, float y, float z,float w){
  tf2::Quaternion q(x,y,z,w);
  tf2::Matrix3x3 m(q);
  double roll,pitch,yaw;
  m.getRPY(roll,pitch,yaw);
  return yaw;
}

float pose_tracker::theta_converter(float theta){
  if(theta < 0)
    return  2*M_PI + theta;
  else if(theta >= 2*M_PI){
    return theta-2*M_PI;
  }
  else
    return theta;
}

float pose_tracker::rotate_direct_discriminate(float g_theta, float n_theta){
  float temp_angle;
  temp_angle = g_theta-n_theta;

  temp_angle = theta_converter(temp_angle);
    
  if( temp_angle >= 0.0 && temp_angle <= M_PI ){
    rotate_state = 0;
    return temp_angle;//正轉
  }
  else if(temp_angle > M_PI && temp_angle <= 2*M_PI){
    rotate_state = 1;
    return 2*M_PI-temp_angle;//逆轉
  }
}

bool pose_tracker::reach_goal_xy(float g_x, float g_y, float n_x, float n_y){
  if( (abs(g_x - n_x) <= xy_tolerance) && (abs(g_y - n_y) <= xy_tolerance) ){
    return true;
  }
  else{
    return false;
  }
}

bool pose_tracker::reach_goal_theta(float g_theta, float n_theta){
  if ( abs(g_theta - n_theta) <= theta_tolerance ){
    return true;
  }
  else{
    return false;
  }
}

void pose_tracker::rotate(float g_theta, float n_theta){
  float out_put_theta = rotate_direct_discriminate(g_theta, n_theta);//順便更新rotate_state
  
  if( reach_goal_theta(g_theta, n_theta) ){
    re_plan = true;
  }
  else{
    if ( (last_rotate_state!=2) && last_rotate_state!=rotate_state){
      if( rotate_state == 0 )
        give_velocity(0.0, w_recovery);
      else if(rotate_state == 1)
        give_velocity(0.0, -w_recovery);
    }
    else{
      if(rotate_state == 0){
        give_velocity(0.0, angular_velocity_profile(out_put_theta) );//正轉
      }
      else if(rotate_state == 1){
        give_velocity(0.0, -angular_velocity_profile(out_put_theta) );//逆轉
      }
      last_rotate_state = rotate_state;
    }
  }
}

void pose_tracker::linear(float g_x, float g_y, float n_x, float n_y){//過頭退會來需要角度修正？
  float angle_v;
  float now_dist = sqrt( pow((g_x-n_x),2) + pow((g_y-n_y),2) );
  float pass_dist = sqrt( pow((initial_pose[0]-n_x),2) + pow((initial_pose[1]-n_y),2) );
  if( reach_goal_xy(g_x, g_y, n_x, n_y) ){
    ROS_INFO("linear over");
    re_plan = true;
    return;
  }
  else{ 
    if(pass_dist > desire_dist){
      overshoot_state = -1;
      if(redecide_line){
        set_line_func();
        theta_ = theta_converter(now_pose_.theta-M_PI);
        redecide_line = false;
      }
      angle_v = compensate_angular_vel();
      if(state == ActionMode::EMERGENCY)
        return;
      if(overshoot_compensate){
        ROS_INFO_STREAM("backward or not :"<<backward);
        if(backward)
          give_velocity( v_recovery, angle_v);
        else
          give_velocity( -v_recovery, angle_v);
      }
      else{
        if(backward)
          give_velocity( linear_velocity_profile(now_dist), angle_v);
        else
          give_velocity( -linear_velocity_profile(now_dist), angle_v);
      }
    }
    else{
      overshoot_state=1;
      redecide_line = true;
      angle_v = compensate_angular_vel();
      if(state == ActionMode::EMERGENCY)
        return;
      if(backward)
        give_velocity( -linear_velocity_profile(now_dist), angle_v);
      else
        give_velocity( linear_velocity_profile(now_dist), angle_v);
    }
    return;
  }
}

void pose_tracker::emergency_dec(){
  current_time = ros::Time::now();
  if(v_now>=v_threshold && w_now>=w_threshold){
    if(w_now>=0)
      w_now -= w_dacc_e*(current_time - last_time).toSec();
    else
      w_now += w_dacc_e*(current_time - last_time).toSec();
    v_now -= v_dacc_e*(current_time - last_time).toSec();
    if(backward)
        give_velocity( -overshoot_state*v_now, w_now);
    else
      give_velocity( overshoot_state*v_now, w_now);
  }
  else{
    give_velocity(0.0,0.0);
    state = ActionMode::STOP;
    re_plan=true;
  }
  last_time = ros::Time::now();
}

void pose_tracker::set_line_func(){ //note theta is atan(-a,b)
  float delta_x = goal_pose_.x - now_pose_.x;
  float delta_y = goal_pose_.y - now_pose_.y;
  line_[0] = -delta_y;
  line_[1] = delta_x;
  line_[2] = -( line_[0]*goal_pose_.x + line_[1]*goal_pose_.y);
}

float pose_tracker::compensate_angular_vel(){//要重新整理
  if( linear_compensate ){
    float pose_relate_line, divisor, distance_, line_theta_, temp_angle;
    float n_theta_ = theta_;
    //get distance
    pose_relate_line = line_[0]*now_pose_.x + line_[1]*now_pose_.y + line_[2];
    divisor = sqrt( pow(line_[0],2)+pow(line_[1],2) );
    distance_ = -(pose_relate_line/divisor);

    //angle
    line_theta_ = atan2(-line_[0], line_[1]);
    if(line_theta_ < 0){
      line_theta_ =  2*M_PI + line_theta_;
    }
    if(n_theta_ < 0){
      n_theta_ = 2*M_PI + n_theta_;
    }

    temp_angle = line_theta_ - n_theta_;
    if(temp_angle<0){
      temp_angle += 2*M_PI;
    }
    if( temp_angle >= 0.0 && temp_angle <= M_PI ){
      //正轉
      temp_angle = temp_angle;
    }
    else if(temp_angle > M_PI && temp_angle <= (2*M_PI+0.0001)){
      //逆轉
      temp_angle = temp_angle - 2*M_PI;
    }

    w_now = Kd_*distance_ + Kt_*temp_angle;
    //確保不要爆衝
    if(abs(temp_angle)>=deviate_angle){
      ROS_INFO("cause deviated angle to large, need to replan");
      state = ActionMode::EMERGENCY;
    }
    
    if(abs(distance_) >= deviate_distance){
      ROS_INFO("cause deviated distance to large, need to replan");
      state = ActionMode::EMERGENCY;
    }

    ROS_INFO_STREAM("now_angle :"<<n_theta_);
    ROS_INFO_STREAM("line theta :"<<line_theta_);
    ROS_INFO_STREAM("dis :"<<distance_);
    ROS_INFO_STREAM("Kd_*distance_ :"<<Kd_*distance_);
    ROS_INFO_STREAM("angle :"<<temp_angle);
    ROS_INFO_STREAM("Kt_*temp_angle :"<<Kt_*temp_angle);
    ROS_INFO_STREAM("ang_vel :"<<w_now);

    return w_now;
  }
  else{
    return 0.0;
  }
}

void pose_tracker::give_velocity(float linear_vel, float rotate_vel){
  geometry_msgs::Twist vel;
  vel.linear.x = linear_vel;
  vel.linear.y = 0.0 ;
  vel.linear.z = 0.0 ;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = rotate_vel;
  vel_pub_.publish(vel);
}

float pose_tracker::linear_velocity_profile(float difference_distance){
  current_time = ros::Time::now();
  float decelerate_distance = (v_now + v_threshold)*v_now/v_dacc;
  if ( difference_distance<=decelerate_distance ){
    decelerate_mode = true;
  }
  else{
    v_now += v_acc*(current_time - last_time).toSec();
    if(v_now > v_max){
      v_now = v_max;
    }
  }

  if (decelerate_mode){
    float new_v_dacc = 0.5*( pow(v_now,2)-pow(v_threshold,2) )/difference_distance;
    if( fabs(new_v_dacc)>= fabs(v_dacc)){
      new_v_dacc = v_dacc;
    }
    v_now -= new_v_dacc*(current_time - last_time).toSec();
    if(v_now <= v_threshold){
      v_now = v_threshold;
    }
  }
  ROS_INFO_STREAM("v_now : "<< v_now);
  last_time = ros::Time::now();
  return v_now;
}


float pose_tracker::angular_velocity_profile(float difference_angle){
  current_time = ros::Time::now();
  float decelerate_angle =  (w_now + w_threshold)*w_now/w_dacc;
  ROS_INFO_STREAM("w_now 1 : "<< w_now);
  ROS_INFO_STREAM("decelerate_angle : "<< decelerate_angle);
  ROS_INFO_STREAM("difference_angle : "<< difference_angle);
  // ROS_INFO_STREAM("decelerate_angle : "<< decelerate_angle);
  // ROS_INFO_STREAM("difference_angle : "<< difference_angle);

  if ( difference_angle<=decelerate_angle ){
    decelerate_mode = true;
  }
  else if( difference_angle > decelerate_angle ){
    w_now += w_acc*(current_time - last_time).toSec();
    if(w_now >= w_max){
      w_now = w_max;
    }
    ROS_INFO_STREAM("w_now 2 : "<< w_now);
  }

  // re_calculate_acc
  if (decelerate_mode){
    ROS_INFO_STREAM("deceleration");
    float new_w_dacc = 0.5*( pow(w_now,2)-pow(w_threshold,2) )/difference_angle;
    if( fabs(new_w_dacc)>= fabs(w_dacc)){//要注意阿～～
      new_w_dacc = w_dacc;
    }
    w_now -= new_w_dacc*(current_time - last_time).toSec();
    ROS_INFO_STREAM("time : "<<(current_time - last_time).toSec());
    if(w_now <= w_threshold){
      w_now = w_threshold;
    }
    ROS_INFO_STREAM("w_now 3 : "<< w_now);
  }

  ROS_INFO_STREAM("w_now : "<< w_now);
  last_time = ros::Time::now();
  return w_now;
}

pose_tracker::~pose_tracker(){}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_tracker");  
  pose_tracker pose_tracker;

  return 0;
}
