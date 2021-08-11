#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>

#include <termios.h>   
#include <math.h>  
#include <stdio.h>  
#include <stdlib.h>  
// #include <ncurses.h>

//https://bytetool.web.app/en/ascii/
#define KEYCODE_W 0x77  
#define KEYCODE_A 0x61  
#define KEYCODE_S 0x73  
#define KEYCODE_D 0x64
#define KEYCODE_c 99 

using namespace std;
enum class mode {script,stop};
mode state;

bool next_point = true;
int last_count=10000000;

float point[][3]= { {0.8,2.7,3.14},{0.3,2.7,3.14},{1.5,2.7,3.14} };
int point_count=0;


// char getch(){
//   char c;
//   static struct termios oldt, newt;
//   tcgetattr( STDIN_FILENO, &oldt);           // save old settings
//   newt = oldt;
//   newt.c_lflag &= ~(ICANON);                 // disable buffering      
//   tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
  
//   c = getchar();  // read character 

//   ROS_INFO("66666");
//   tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings

//   return c;
// }

char getch()
{
	fd_set set;//long型別的陣列，是一個數組的巨集定義
	struct timeval timeout;
	int rv;
	char buff = 0;
	int len = 1;
	int filedesc = 0;
	FD_ZERO(&set);//清空set
	FD_SET(filedesc, &set);//將filedesc加入set中
	
	timeout.tv_sec = 0; //second
	timeout.tv_usec = 1000; //microsecond

	rv = select(filedesc + 1, &set, NULL, NULL, &timeout); //檔案內可讀的fd總數

	struct termios old = {0};//初始值
	if (tcgetattr(filedesc, &old) < 0){ //-1為失敗
		ROS_ERROR("tcgetattr fail");
  }
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(filedesc, TCSANOW, &old) < 0){
		ROS_ERROR("tcsetattr ICANON");
  }

	if(rv == -1){
		// ROS_ERROR("select");
  }
	else if(rv == 0){
		// ROS_INFO("no_key_pressed");
  }
	else
		read(filedesc, &buff, len );

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(filedesc, TCSADRAIN, &old) < 0){
		ROS_ERROR ("tcsetattr ~ICANON");
  }
	return (buff);
}

void state_CB(const std_msgs::Int32MultiArray::ConstPtr& plan_state){
  ROS_INFO("check if change point");
  if (plan_state->data[0]==1 && plan_state->data[2]!=last_count){
    next_point = true;
    ROS_INFO("I need to change now~~");
  }
  last_count = plan_state->data[2];
}

void script(){
  if(next_point){
    ROS_INFO("point change now~~");
    point_count +=1;
    if(point_count == sizeof(point)/sizeof(point[0]) ){
      point_count -= point_count;
    }
  }
  else
    return;
}

void pub_goal(float x,float y,float theta,ros::Publisher set_goal_){
  ROS_INFO("pub_now~~~");
  geometry_msgs::PoseStamped goal;
  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "map";
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0;
  tf2::Quaternion goal_quat;
  goal_quat.setRPY(0, 0, theta);
  goal.pose.orientation.x = goal_quat.x();
  goal.pose.orientation.y = goal_quat.y();
  goal.pose.orientation.z = goal_quat.z();
  goal.pose.orientation.w = goal_quat.w();

  set_goal_.publish(goal);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "goal_pose",ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);//後面必須留，免得原本的指令被取代掉
  ros::NodeHandle n;
  ros::Publisher set_goal = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
  // boost::thread t = boost::thread(boost::bind(&keyboardLoop, &goal_pose_set)); 
  ros::Subscriber fake_main = n.subscribe<std_msgs::Int32MultiArray>("plan_state",10,&state_CB);
  
  ros::Rate rate(1);
  state = mode::stop;
  while (ros::ok()){
    ros::spinOnce();
    ROS_INFO("hihihihi");
    int c = getch();   // call your non-blocking input function
    ROS_INFO_STREAM("now is : "<<c);
    switch(c)
    {
      case KEYCODE_c:
        ROS_INFO("allaal");
        state = mode::script;
        break;
    }

    switch (state)
    {
    case mode::script:
      ROS_INFO("hihihihihihihi");
      script();
      if(next_point){
        pub_goal( point[point_count][0], point[point_count][1], point[point_count][2],set_goal);
        next_point = false;
      }
      break;
    }
    rate.sleep();
  }
  return 0;
}