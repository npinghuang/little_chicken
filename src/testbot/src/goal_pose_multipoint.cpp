#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Int32MultiArray.h>
#include <ros/master.h>
#include <vector>

using namespace std;

int plan_state = 1; //initial
int plan_count = -1; //initial

void plan_callback(const std_msgs::Int32MultiArray::ConstPtr& plan_state_){
  if(plan_count != plan_state_->data[2]){
    plan_state = plan_state_->data[0];
    plan_count = plan_state_->data[2];
    ROS_INFO_STREAM("state : "<<plan_state_->data[0]);
  }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "goal_pose");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    ros::Subscriber planner_sub= n.subscribe<std_msgs::Int32MultiArray>("plan_state",1,&plan_callback);
    ros::Rate loop_rate(10);

    XmlRpc::XmlRpcValue goal_input;
    vector<geometry_msgs::PoseStamped> goal_list;
    bool goal_loop; //是否一直循環goal_list
    
    nh.param("goal_list", goal_input, goal_input);
    nh.param("goal_loop", goal_loop, true);

    ROS_INFO_STREAM("type : "<< goal_input.getType());
    for(int i=0; i<goal_input.size(); i++){
      XmlRpc::XmlRpcValue point = goal_input[i];
      geometry_msgs::PoseStamped goal;
      
      goal.header.stamp = ros::Time::now();
      goal.header.frame_id = "map";
      goal.pose.position.x = point[0];
      goal.pose.position.y = point[1];
      goal.pose.position.z = 0.0;
      tf2::Quaternion goal_;
      goal_.setRPY(0, 0, point[2]);
      goal.pose.orientation = tf2::toMsg(goal_);

      goal_list.push_back(goal);
    }
    
    while( ros::ok() ){
      ros::spinOnce();
      while(goal_pub.getNumSubscribers()<1 && planner_sub.getNumPublishers()<1 ){ 
        ROS_INFO("setting...");
        loop_rate.sleep();         
      }
      geometry_msgs::PoseStamped goal;
      // ROS_INFO_STREAM("state : "<<plan_state);
      switch (plan_state){
        case 1:
          ROS_INFO_STREAM("case 1 ");
          if( !goal_list.empty() ){
            goal_list[0].header.stamp = ros::Time::now();
            goal = goal_list[0];
            goal_pub.publish(goal);

            if(plan_count!=-1){
              goal_list.erase(goal_list.begin(),goal_list.begin()+1);
              if(goal_loop)
                goal_list.push_back(goal);
            }
          }
          break;
        case 0:
          ROS_INFO_STREAM("case 0 ");
          break;
      }

      loop_rate.sleep(); 
    }
    return 0;
}