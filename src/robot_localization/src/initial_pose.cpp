#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "initial_pose");

  ros::NodeHandle n;
  ros::NodeHandle nh_local("~");

  ros::Publisher initial_pose_set = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("set_pose", 10);

  double p_x;
  double p_y;
  double p_theta;
  double p_delay;
  double p_cov;
  nh_local.param<double>("x", p_x, 0.98);
  nh_local.param<double>("y", p_y, 2.805);
  nh_local.param<double>("theta", p_theta, -180);
  nh_local.param<double>("delay", p_delay, 1);
  nh_local.param<double>("p_cov", p_cov, 1e-9);

  tf2::Quaternion q;
  q.setRPY(0., 0., p_theta);

  geometry_msgs::PoseWithCovarianceStamped initial_pose;
  initial_pose.header.seq = 0;
  initial_pose.header.stamp.sec = 0;
  initial_pose.header.stamp.nsec = 0;
  initial_pose.header.frame_id = "map";
  initial_pose.pose.pose.position.x = p_x;
  initial_pose.pose.pose.position.y = p_y;
  initial_pose.pose.pose.position.z = 0;
  initial_pose.pose.pose.orientation = tf2::toMsg(q);

  // clang-format off
  initial_pose.pose.covariance = {p_cov, 0.0,   0.0, 0.0, 0.0, 0.0,
                                  0.0,   p_cov, 0.0, 0.0, 0.0, 0.0,
                                  0.0,   0.0,   0.0, 0.0, 0.0, 0.0,
                                  0.0,   0.0,   0.0, 1e3, 0.0, 0.0,
                                  0.0,   0.0,   0.0, 0.0, 1e3, 0.0,
                                  0.0,   0.0,   0.0, 0.0, 0.0, p_cov};
  // clang-format on

  ros::Duration(p_delay).sleep();

  initial_pose_set.publish(initial_pose);

  ros::spin();
  return 0;
}
