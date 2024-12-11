#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <math.h>
//#include <conio.h>
#define PI 3.14159
#include <thread>
#include <chrono>

double pos_x=0;
double pos_y=0;
double b_pos_x=0;
double b_pos_y=0;
double c_pos_x=0;
double c_pos_y=0;
double d_pos_x=0;
double d_pos_y=0;

float orien_x=0;
float orien_y=0;
float orien_z=0;
float orien_w=0;

float b_orien_x=0;
float b_orien_y=0;
float b_orien_z=0;
float b_orien_w=0;

float c_orien_x=0;
float c_orien_y=0;
float c_orien_z=0;
float c_orien_w=0;

move_base_msgs::MoveBaseGoal goal;
move_base_msgs::MoveBaseGoal goal_b;
move_base_msgs::MoveBaseGoal goal_c;
move_base_msgs::MoveBaseGoal goal_d;

int j=0;
float xant,yant;
float x1_coord,y1_coord,oz1_coord;
float x2_coord,y2_coord,oz2_coord,oz3_coord;
float x3,y3,x4,y4;
float ox1=0,oy1=0,ox2=0,oy2=0;
int conta_aux=5;
int conta_aux_2=5;
float checkpoint_x;
float checkpoint_y;
float origen_x=0;
float origen_y=0;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
tf2::Quaternion myQuaternion;


double distance_a_b (double pos_x,double pos_y,double b_pos_x,double b_pos_y)
{

if(sqrt( ((pos_x - b_pos_x)*(pos_x - b_pos_x)) + ((pos_y - b_pos_y)*(pos_y - b_pos_y)))> 6.0 ) return 2;
else return 0;
}


double distance_b_c (double b_pos_x,double b_pos_y,double c_pos_x,double c_pos_y)
{

double dist= (sqrt( ((c_pos_x - b_pos_x)*(c_pos_x - b_pos_x)) + ((c_pos_y - b_pos_y)*(c_pos_y - b_pos_y))));
return dist;
}

double distance_c_d (double c_pos_x,double c_pos_y,double d_pos_x,double d_pos_y)
{

double dist=(sqrt( ((c_pos_x - d_pos_x)*(c_pos_x - d_pos_x)) + ((c_pos_y - d_pos_y)*(c_pos_y - d_pos_y))));
return (sqrt( ((c_pos_x - d_pos_x)*(c_pos_x - d_pos_x)) + ((c_pos_y - d_pos_y)*(c_pos_y - d_pos_y))));
}


void goals_b(MoveBaseClient &ac_b) 
{

if(distance_b_c( b_pos_x, b_pos_y, c_pos_x, c_pos_y)==0)
{
ROS_INFO("Sending goal to robot_b ");
ac_b.sendGoal(goal_b);
}

}

void goals_c(MoveBaseClient &ac_c) 
{
if(distance_c_d ( c_pos_x, c_pos_y, d_pos_x, d_pos_y)==0)
{
ROS_INFO("Sending goal to robot_c");
ac_c.sendGoal(goal_c);
}

}

void goals_d(MoveBaseClient &ac_d) 
{

ROS_INFO("Sending goal robot_d");
ac_d.sendGoal(goal_d);


}


void chatterCallback_a(const nav_msgs::Odometry::ConstPtr& msg)
{
pos_x=msg->pose.pose.position.x;
pos_y=msg->pose.pose.position.y;

orien_x=msg->pose.pose.orientation.x;
orien_y=msg->pose.pose.orientation.y;
orien_z=msg->pose.pose.orientation.z;
orien_w=msg->pose.pose.orientation.w;


goal_b.target_pose.header.frame_id = "robot_b_map";
goal_b.target_pose.header.stamp = ros::Time::now();

goal_b.target_pose.pose.position.x =pos_x+x1_coord;
goal_b.target_pose.pose.position.y =pos_y-y1_coord;

myQuaternion.setRPY( 0, 0, atan2((goal.target_pose.pose.position.y-goal_b.target_pose.pose.position.y),(goal.target_pose.pose.position.x -goal_b.target_pose.pose.position.x )) );

goal_b.target_pose.pose.orientation.x=  myQuaternion.getX();
goal_b.target_pose.pose.orientation.y=  myQuaternion.getY();
goal_b.target_pose.pose.orientation.z=  myQuaternion.getZ(); 
goal_b.target_pose.pose.orientation.w = myQuaternion.getW();


}

void chatterCallback_b(const nav_msgs::Odometry::ConstPtr& msg)
{
b_pos_x=msg->pose.pose.position.x;
b_pos_y=msg->pose.pose.position.y;

b_orien_x=msg->pose.pose.orientation.x;
b_orien_y=msg->pose.pose.orientation.y;
b_orien_z=msg->pose.pose.orientation.z;
b_orien_w=msg->pose.pose.orientation.w;

goal_c.target_pose.header.frame_id = "robot_c_map";
goal_c.target_pose.header.stamp = ros::Time::now();


goal_c.target_pose.pose.position.x =b_pos_x+x1_coord;
goal_c.target_pose.pose.position.y =b_pos_y-y1_coord;

myQuaternion.setRPY( 0, 0, atan2((goal_b.target_pose.pose.position.y-goal_c.target_pose.pose.position.y),(goal_b.target_pose.pose.position.x -goal_c.target_pose.pose.position.x )) );

goal_c.target_pose.pose.orientation.x=  myQuaternion.getX();
goal_c.target_pose.pose.orientation.y=  myQuaternion.getY();
goal_c.target_pose.pose.orientation.z=  myQuaternion.getZ(); 
goal_c.target_pose.pose.orientation.w = myQuaternion.getW(); 


}

void chatterCallback_c(const nav_msgs::Odometry::ConstPtr& msg)
{
c_pos_x=msg->pose.pose.position.x;
c_pos_y=msg->pose.pose.position.y;

c_orien_x=msg->pose.pose.orientation.x;
c_orien_y=msg->pose.pose.orientation.y;
c_orien_z=msg->pose.pose.orientation.z;
c_orien_w=msg->pose.pose.orientation.w;

goal_d.target_pose.header.frame_id = "robot_d_map";
goal_d.target_pose.header.stamp = ros::Time::now();

goal_d.target_pose.pose.position.x =c_pos_x+x1_coord;
goal_d.target_pose.pose.position.y =c_pos_y-y1_coord;

myQuaternion.setRPY( 0, 0, atan2((goal_c.target_pose.pose.position.y-goal_d.target_pose.pose.position.y),(goal_c.target_pose.pose.position.x -goal_d.target_pose.pose.position.x )) );

goal_d.target_pose.pose.orientation.x=  myQuaternion.getX();
goal_d.target_pose.pose.orientation.y=  myQuaternion.getY();
goal_d.target_pose.pose.orientation.z=  myQuaternion.getZ(); 
goal_d.target_pose.pose.orientation.w = myQuaternion.getW();


}

void chatterCallback_d(const nav_msgs::Odometry::ConstPtr& msg)
{
d_pos_x=msg->pose.pose.position.x;
d_pos_y=msg->pose.pose.position.y;



}



int main(int argc, char** argv)
{


float x1,y1,x2,y2;
int index;

ros::init(argc, argv, "simple_navigation_goals");
ros::init(argc, argv, "map_publisher");
ros::init(argc, argv, "odom_listener");

ros::NodeHandle n;
ros::NodeHandle nh;

ros::Subscriber sub = n.subscribe("/robot/robotnik_base_control/odom", 1000, chatterCallback_a);
ros::Subscriber sub1 = n.subscribe("/robot_b/robotnik_base_control/odom", 1000, chatterCallback_b);
ros::Subscriber sub2 = n.subscribe("/robot_c/robotnik_base_control/odom", 1000, chatterCallback_c);
ros::Subscriber sub3 = n.subscribe("/robot_d/robotnik_base_control/odom", 1000, chatterCallback_d);
	
	
//tell the action client that we want to spin a thread by default
MoveBaseClient ac("robot/move_base", true);
MoveBaseClient ac_b("robot_b/move_base", true);
MoveBaseClient ac_c("robot_c/move_base", true);
MoveBaseClient ac_d("robot_d/move_base", true);

//wait for the action server to come up
while(!ac.waitForServer(ros::Duration(5.0))){
ROS_INFO("Waiting for the move_base action server to come up");
}

FILE *fp;
FILE *fp2;


fp = fopen("/home/robcib/catkin_ws/src/path_planning_sims/path_planning_intro/unit4_pp/maps/map_path.txt","r");
fscanf(fp, "%*[^\n]\n");
fscanf(fp, "%*[^\n]\n");


fp2 = fopen("/home/robcib/catkin_ws/src/path_planning_sims/path_planning_intro/unit4_pp/maps/map_coords.txt","r");
fscanf(fp2,"%f,%f,%f,",&x1_coord,&y1_coord,&oz1_coord);
fscanf(fp2,"%f,%f,%f,%f",&x2_coord,&y2_coord,&oz2_coord,&oz3_coord);
fclose(fp2);

x1_coord=785-2*x1_coord;
y1_coord=785-2*y1_coord;


ros::Time start_time = ros::Time::now();
//ros::Rate rate(0.5);

while((conta_aux==5 && conta_aux_2==5)&&ros::ok())
{



// Spin once to process callbacks
ros::spinOnce();
        
if(j==0) 
{
conta_aux=fscanf(fp,"%d\t%f,%f\t%f,%f",&index,&x1,&y1,&x2,&y2);
ox1=x1;
oy1=y1;
ox2=x2;
oy2=y2;
conta_aux_2=fscanf(fp,"%d\t%f,%f\t%f,%f",&index,&x3,&y3,&x4,&y4);
}
else
if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && (distance_a_b( pos_x, pos_y, b_pos_x, b_pos_y)==0)  ) //(distance_a_b( pos_x, pos_y, b_pos_x, b_pos_y)==0)
{
xant=x1; yant=y1;
x1=x3; y1=y3; x2=x4; y2=y4;
conta_aux_2=fscanf(fp,"%d\t%f,%f\t%f,%f",&index,&x3,&y3,&x4,&y4);
}

//////////////////////////////////////////////////////////////////////
//robot 

goal.target_pose.header.frame_id = "robot_map";
goal.target_pose.header.stamp = ros::Time::now();

if(j==0)
{
goal.target_pose.pose.position.x =(x1-ox1);
goal.target_pose.pose.position.y =(y1-oy1);
}
else
{
//float ans= sqrt((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1));
goal.target_pose.pose.position.x =(x1-ox1) ;
goal.target_pose.pose.position.y =(y1-oy1) ;
}
myQuaternion.setRPY( 0, 0, atan2((y3-y1),(x3-x1)) );

goal.target_pose.pose.orientation.x=  myQuaternion.getX();
goal.target_pose.pose.orientation.y=  myQuaternion.getY();
goal.target_pose.pose.orientation.z=  myQuaternion.getZ(); 
goal.target_pose.pose.orientation.w = myQuaternion.getW();



if(j==0)
{
ROS_INFO("Sending goal: %f,%f,%f,%f %f . ALso: %d",x1,y1,x3,y3, 360*(atan((y3-y1+oy1)/(x3-x1+ox1)))/(2*PI) ,conta_aux);

//checkpoint_x=787-2*goal.target_pose.pose.position.x;
//checkpoint_y=787-2*goal.target_pose.pose.position.y;

std::string myString0 = "gnome-terminal -- rosrun gazebo_ros spawn_model -file /home/robcib/catkin_ws/src/simple_navigation_goals/src/prueba.urdf -urdf -x" + std::to_string(+x1-ox1-x1_coord) + " -y " + std::to_string(-y1+oy1+y1_coord)
+ " -z 1.0 -model my_object_" + std::to_string(j);

std::cout << myString0 << std::endl;

const char* checkpoint0 = myString0.c_str();
std::system(checkpoint0);

}

if(j==0)
{
ac.sendGoal(goal);
ac.waitForResult();	
}
else
{
ac.sendGoal(goal);
//rate.sleep();
//ac_b.sendGoal(goal_b);
//ac_c.sendGoal(goal_c);
//ac_d.sendGoal(goal_d);

ros::Time current_time = ros::Time::now();
ros::Duration elapsed_time = current_time - start_time;

if(elapsed_time.toSec() >= 3)
{
goals_b(ac_b);
goals_c(ac_c);
goals_d(ac_d);
start_time = ros::Time::now();
}
}
//ac.waitForResult();	
//ac_b.waitForResult();
//ac_c.waitForResult();
//ac_d.waitForResult();


j++;


if((j!=0) && (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) && (distance_a_b==0)  ) //&& (distance_a_b==0)
{
ROS_INFO("Sending goal: %f,%f,%f,%f %f . ALso: %d",x1,y1,x3,y3, 360*(atan((y3-y1+oy1)/(x3-x1+ox1)))/(2*PI) ,conta_aux);

std::string myString0 = "gnome-terminal -- rosrun gazebo_ros spawn_model -file /home/robcib/catkin_ws/src/simple_navigation_goals/src/prueba.urdf -urdf -x" + std::to_string(+x3-ox1-x1_coord)
+ " -y " + std::to_string(y3-oy1+y1_coord)
+ " -z 1.0 -model my_object_"
+std::to_string(j);
std::cout << myString0 << std::endl;

const char* checkpoint0 = myString0.c_str();
std::system(checkpoint0);
}


}


fclose(fp);

return 0;
}



