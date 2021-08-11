#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/master.h>


int main(int argc, char **argv){
    ros::init(argc, argv, "goal_pose");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    ros::Publisher goal_pose_set = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    ros::Rate loop_rate(1);
    geometry_msgs::PoseStamped goal;

    float g_x;
    float g_y;
    float g_theta;
    
    nh.param<float>("goal_x", g_x, 0.33);
    nh.param<float>("goal_y", g_y, 0.575);
    nh.param<float>("goal_theta", g_theta, 0);
    ROS_INFO("hahahaha");
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.pose.position.x = g_x;
    goal.pose.position.y = g_y;
    goal.pose.position.z = 0.0;
    tf2::Quaternion goal_;
    goal_.setRPY(0, 0, g_theta);
    goal.pose.orientation = tf2::toMsg(goal_);
    
    int time = 0;
    
    while( ros::ok() ){
      if(goal_pose_set.getNumSubscribers()<1){ 
        ROS_INFO("setting...");
        time = 0 ;         
      }
      else if(time<1){
        ROS_INFO_STREAM("time : "<<time);
        time+=1;
        goal_pose_set.publish(goal);
      }
      loop_rate.sleep(); 
    }
    return 0;
}