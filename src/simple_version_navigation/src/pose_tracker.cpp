#include <ros/ros.h>
#include <simple_version_navigation/pose_tracker.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/LinearMath/Transform.h"
#include <geometry_msgs/Pose2D.h>

#include <geometry_msgs/Twist.h>
#include <cmath>

using namespace std;

pose_tracker::pose_tracker(){
  ros::NodeHandle n;
  vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  goal_sub_ =  n.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal",1,&pose_tracker::goalCB,this);
  state = ActionMode::STOP;
  ros::Rate rate(control_freq);
  while (ros::ok() ){
    ros::spinOnce();
    get_now_pose();
    if( reach_goal_xy() && reach_goal_theta() ){
      ROS_INFO("reach goal");
      state = ActionMode::STOP;
    }
    switch (state)
    {
      case ActionMode::AIM: //case 裡面不應該宣告東西，因為他不是一個scope
        aim_theta = atan2(goal_pose_.y - now_pose_.y, goal_pose_.x - now_pose_.x);
        rotate(aim_theta, now_pose_.theta);
        ROS_INFO(" aim ");
        break;
      case ActionMode::MOVING:
        ROS_INFO(" moving ");
        linear();
        break;
      case ActionMode::ROTATE:
        ROS_INFO(" rotate ");
        rotate(goal_pose_.theta, now_pose_.theta);
        break;
      case ActionMode::STOP:
        ROS_INFO(" stop ");
        give_velocity(0.0, 0.0);
        break;
    }
    rate.sleep();
  }
}

void pose_tracker::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goal){
  // goal_pose_ = *goal;
  goal_pose_.x =  goal->pose.position.x;
  goal_pose_.y =  goal->pose.position.y;
  goal_pose_.theta = quaternion_to_theta(goal->pose.orientation.x, goal->pose.orientation.y,
                                         goal->pose.orientation.z, goal->pose.orientation.w);
  w_now = 0.0;
  state = ActionMode::AIM;
}

void pose_tracker::get_now_pose(){
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("map","base_footprint",ros::Time(0),ros::Duration(0.1));
  }
  catch (tf2::TransformException &ex){
    ROS_WARN("%s",ex.what());
  }
  now_pose_.x = transformStamped.transform.translation.x;
  now_pose_.y = transformStamped.transform.translation.y;
  now_pose_.theta = quaternion_to_theta(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y,
                                    transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
}

float pose_tracker::quaternion_to_theta(float x, float y, float z,float w){
  tf2::Quaternion q(x,y,z,w);
  tf2::Matrix3x3 m(q);
  double roll,pitch,yaw;
  m.getRPY(roll,pitch,yaw);
  return yaw;
}

bool pose_tracker::reach_goal_xy(){
  if( (abs(goal_pose_.x - now_pose_.x) <= xy_tolerance) && (abs(goal_pose_.y - now_pose_.y) <= xy_tolerance) ){
    return true;
  }
  else{
    return false;
  }
}

bool pose_tracker::reach_goal_theta(){
  if ( abs(goal_pose_.theta - now_pose_.theta) <= theta_tolerance ){
    return true;
  }
  else{
    return false;
  }
}

void pose_tracker::rotate(float g_theta, float n_theta){
  float temp_angle;
  if(g_theta < 0){
    g_theta =  2*M_PI + g_theta;
  }
  if(n_theta < 0){
    n_theta = 2*M_PI + n_theta;
  }

  temp_angle = g_theta-n_theta;
  if ( reach_goal_xy() && (abs(temp_angle) < theta_tolerance) ){
    give_velocity(0.0 ,0.0);//stop
    ROS_INFO("nonononononon");
    state = ActionMode::STOP;
    return;
  }
  else if( abs(temp_angle) < theta_tolerance ){
    set_line_func();//點到點求直線方程式
    calculate_linear_distance = true;
    state = ActionMode::MOVING;
    return;
  }
  else{
    if(temp_angle<0){
      temp_angle += 2*M_PI;
    }
    
    if( temp_angle >= 0.0 && temp_angle <= M_PI ){
      ROS_INFO(" positive rotate ");
      give_velocity(0.0, angular_velocity_profile(temp_angle) );//正轉
    }
    else if(temp_angle > M_PI && temp_angle <= 2*M_PI){
      ROS_INFO(" negative rotate ");
      give_velocity(0.0, -(angular_velocity_profile(2*M_PI-temp_angle)) );//逆轉
    }
  }
}

void pose_tracker::linear(){
  if( reach_goal_xy() ){
    ROS_INFO("linear over");
    w_now = 0.0;
    state = ActionMode::ROTATE;
    return;
  }
  else{
    float angle_v = compensate_angular_vel();
    float straight_v = linear_velocity_profile();
    give_velocity(straight_v, angle_v);
    return;
  }
}

void pose_tracker::set_line_func(){ //note theta is atan(-a,b)
  float delta_x = goal_pose_.x - now_pose_.x;
  float delta_y = goal_pose_.y - now_pose_.y;
  line_[0] = -delta_y;
  line_[1] = delta_x;
  line_[2] = -( line_[0]*goal_pose_.x + line_[1]*goal_pose_.y);
  ROS_INFO_STREAM("delta x :"<<delta_x);
  ROS_INFO_STREAM("delta y :"<<delta_y);
  ROS_INFO_STREAM("a : "<<line_[0]);
  ROS_INFO_STREAM("b : "<<line_[1]);
}

float pose_tracker::compensate_angular_vel(){
  float pose_relate_line, divisor, distance_, line_theta_, temp_angle;
  float n_theta_ = now_pose_.theta;
  float angular_vel = 0.0;
  //get distance
  pose_relate_line = line_[0]*now_pose_.x + line_[1]*now_pose_.y + line_[2];
  divisor = sqrt( pow(line_[0],2)+pow(line_[1],2) );
  //when distance is negative -> 正轉, vice versa.
  distance_ = -(pose_relate_line/divisor);

  //angle
  line_theta_ = atan2(-line_[0], line_[1]);
  // ROS_INFO_STREAM("theta_ : "<< line_theta_);
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
  else if(temp_angle > M_PI && temp_angle <= 2*M_PI){
    //逆轉
    temp_angle = temp_angle - 2*M_PI;
  }

  angular_vel = Kd_*distance_ + Kt_*temp_angle;
  // ROS_INFO_STREAM("temp_angle : "<< temp_angle);
  // ROS_INFO_STREAM("distance_ : "<< distance_);
  // ROS_INFO_STREAM("angular_vel : "<< angular_vel);
  
  return angular_vel;
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

float pose_tracker::linear_velocity_profile(){
  if ( calculate_linear_distance ){
    start_calculate_time = ros::Time::now();
    ROS_INFO_STREAM(" time --- "<< start_calculate_time.toSec());
    desire_dist = sqrt( pow((goal_pose_.x-now_pose_.x),2) + pow((goal_pose_.y-now_pose_.y),2) );
    min_dist = pow(max_vel,2)/acc;
    if (desire_dist <= min_dist){
      temp_max_vel = sqrt(desire_dist);
      desire_time = temp_max_vel/acc;
      linear_state = 0;
    }
    else{
      desire_time = max_vel/acc;
      duration_time = (desire_dist-min_dist)/max_vel;
      linear_state = 1;
    }
    calculate_linear_distance = false;
  }

  float t_now = (ros::Time::now() - start_calculate_time).toSec();
  switch (linear_state)
  {
    case 0:
      if ( t_now < desire_time){
        calculate_vel = acc*t_now; 
      }
      else{
        calculate_vel = temp_max_vel-acc*(t_now-desire_time);
        if(calculate_vel <= vel_threshold){
          calculate_vel = vel_threshold;
        }
      }
    case 1:
      if ( t_now <= desire_time){
        calculate_vel = acc*t_now; 
      }
      else if( t_now < desire_time+duration_time){
        calculate_vel = max_vel;
      }
      else{
        calculate_vel = max_vel-acc*(t_now-desire_time-duration_time);
        if(calculate_vel <= 0.0){
          calculate_vel = vel_threshold;
        }
      }
  }
  return calculate_vel;
}

float pose_tracker::angular_velocity_profile(float difference_angle){
  float decelerate_angle =  (w_now + w_threshold)*w_now/w_acc;
  ROS_INFO_STREAM("decelerate_angle : "<< decelerate_angle);
  ROS_INFO_STREAM("difference_angle : "<< difference_angle);
  if ( difference_angle<=decelerate_angle ){
    w_now -= w_acc/control_freq;
    if(w_now <= w_threshold){
      w_now = w_threshold;
    }
  }
  else{
    w_now += w_acc/control_freq;
    if(w_now >= w_max){
      w_now = w_max;
    }
  }
  ROS_INFO_STREAM("w_now : "<< w_now);
  return w_now;
}

pose_tracker::~pose_tracker(){}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_tracker");
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  while ( !(tfBuffer.canTransform("map","base_footprint",ros::Time(0),ros::Duration(0.2))) ){
    ROS_INFO("wait for TF");//等tf被發布出來
  }

  pose_tracker pose_tracker;
  return 0;
}
