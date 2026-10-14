#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <vector>
#include <map>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include <ecl/threads.hpp>

using namespace std;
using Client =  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ;
ros::Publisher inpuPub;
ros::Publisher *cmdVelPubPtr;
Client *actionClient;
geometry_msgs::Point current_point;
geometry_msgs::Pose2D base_p_;
geometry_msgs::Pose2D real_pose;
ecl::Thread thread_;
ecl::Thread thread_1;
int dt = 0;
int weizhi = 0;

geometry_msgs::Point setPoint(double _x, double _y, double _z);
geometry_msgs::Quaternion setQuaternion(double _angleRan);
vector<geometry_msgs::Pose> init_goalList();
visualization_msgs::Marker init_markers(const vector<geometry_msgs::Pose>  &wayPoints);
void activeCb();
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
double computeDistance(geometry_msgs::Point& m_current_point, geometry_msgs::Point& m_goal);
void shutdown(int sig);
void baseposeCallback(geometry_msgs::Pose2D bp);
vector<geometry_msgs::Pose> wayPoints;
int f = 0;


geometry_msgs::Point setPoint(double _x, double _y, double _z)
{
	geometry_msgs::Point m_point;
	m_point.x = _x;
	m_point.y = _y;
	m_point.z = _z;
	return m_point;
}

geometry_msgs::Quaternion setQuaternion(double _angleRan)
{
	geometry_msgs::Quaternion m_quaternion;
	m_quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, _angleRan);
	return m_quaternion;
}

vector<geometry_msgs::Pose> init_goalList()
{
	//Create a list to hold the target quaternions (orientations)
	vector<geometry_msgs::Pose> wayPoints;
	geometry_msgs::Pose pose ;

	int points_size = 0;
	float x = 0.0;
	float y = 0.0;
	float th = 0.0;
	string ParamName;
	ros::param::get("/scenario/points_size",points_size);
	for(int i = 1; i < points_size+1; i++){
		ParamName = "/scenario/point_" + to_string(i)  + "/x";
		ros::param::get(ParamName,x);
		//ROS_INFO("point_x: %f",x);
		ParamName = "/scenario/point_" + to_string(i)  + "/y";
		ros::param::get(ParamName,y);
		//ROS_INFO("point_y: %f",x);
		ParamName = "/scenario/point_" + to_string(i)  + "/th";
		ros::param::get(ParamName,th);
		ROS_INFO("point_%d: %f  %f  %f",i,x,y,th);
		pose.position = setPoint(x,y, th);
		pose.orientation = setQuaternion(th);
		wayPoints.push_back(pose);
	}
	return wayPoints;
}


visualization_msgs::Marker init_markers(const vector<geometry_msgs::Pose>  &wayPoints)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();

	marker.ns = "waypoints";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration(); //0 is forever
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.r = 1.0;
	marker.color.g = 0.7;
	marker.color.b = 1.0;
	marker.color.a = 1.0;

	for (const auto &point : wayPoints){
		marker.points.push_back(point.position);
	}
	return marker;
}

// Called once when the goal becomes active
void activeCb()
{
	ROS_INFO("New goal Received");
}

// Called every time feedback is received for the goal
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	//ROS_INFO("Got base_position of Feedback");
	current_point.x = feedback->base_position.pose.position.x;
	current_point.y = feedback->base_position.pose.position.y;
	current_point.z = feedback->base_position.pose.position.z;
	//ROS_INFO("x y: %f  %f  %f ",current_point.x,current_point.y,current_point.z);
}

void baseposeCallback(geometry_msgs::Pose2D bp)
{
	if(f == 0){
		if(sqrt(pow((bp.x - wayPoints[0].position.x),2)+pow((bp.y-wayPoints[0].position.y),2))<0.2 && fabs(bp.theta - wayPoints[0].position.z) < 0.15){
			ROS_INFO("diyigedian...");
			f = 1;
			dt = 1;
			actionClient->cancelAllGoals();
			ros::Duration(1).sleep(); 

			cmdVelPubPtr->publish(geometry_msgs::Twist());
			ros::Duration(1).sleep();
				base_p_.x = bp.x;
			base_p_.y = bp.y;
			base_p_.theta = bp.theta; 
			ros::param::set("/usr_task",4);
			ros::param::set("/usr_task1",4);
		}
	}else{
		if(sqrt(pow((bp.x - base_p_.x),2)+pow((bp.y-base_p_.y),2))>0.3)
		{
			ROS_INFO("diERgedian...");
			actionClient->cancelAllGoals();
	        //ros::Duration(1).sleep(); 
            ros::param::set("/usr_task",5);
	    ros::param::set("/usr_task1",5);
	        cmdVelPubPtr->publish(geometry_msgs::Twist());
	        ros::Duration(1).sleep();
                base_p_.x = bp.x;
	        base_p_.y = bp.y;
	        base_p_.theta = bp.theta;
			std_msgs::Int8 a;
			a.data = 1;
			inpuPub.publish(a);
		}
	}

	real_pose.x = bp.x;
	real_pose.y = bp.y;
	real_pose.theta = bp.theta;
	
	
}

void thread_1func()
{
	// while(1){
		// if(sqrt(pow((real_pose.x - wayPoints[0].position.x),2)+pow((real_pose.y-wayPoints[0].position.y),2))<0.1)
		// {

		// }
		// if(sqrt(pow((real_pose.x - wayPoints[1].position.x),2)+pow((real_pose.y-wayPoints[1].position.y),2))<0.1)
		// {

		// }
		// ros::Duration(0.5).sleep();
	// }
	// int task=0;
	// do {
	// 	ros::param::get("/usr_task",task);
	// 	if(task == 6) 
	// 	{
	// 		ros::param::set("/usr_task",5);
    //                     actionClient->cancelAllGoals();
	//         	cmdVelPubPtr->publish(geometry_msgs::Twist());
	// 		ros::Duration(1).sleep(); 
	// 		std_msgs::Int8 a;
	// 		a.data = 1;
	// 		inpuPub.publish(a);
			
	// 	}
	// 	ros::Duration(0.1).sleep(); 
	// }while ( 1 );

}

void shutdown(int sig)
{
     actionClient->cancelAllGoals();
   	ros::Duration(2).sleep(); 

	cmdVelPubPtr->publish(geometry_msgs::Twist());
	ros::Duration(1).sleep(); 
	ROS_INFO("Stopped the robot!");
	ros::shutdown();
}

void thread_func()
{
	ros::spin();
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "scenario_deploy");
	ros::NodeHandle node;
	ros::Rate loop_rate(10);
	signal(SIGINT, shutdown);
	ROS_INFO("scenario Deploy Node Start...");
	wayPoints = init_goalList();
	ros::Subscriber sub_basepos = node.subscribe("basepose",1000,baseposeCallback);
	inpuPub = node.advertise<std_msgs::Int8>("/vision/input", 5);
	thread_.start(ecl::generateFunctionObject(thread_func));
	thread_1.start(ecl::generateFunctionObject(thread_1func));
	ROS_INFO("Waiting for move_base action server...");
	actionClient = new Client("move_base", true);
	if (!actionClient->waitForServer(ros::Duration(60)))
	{
		ROS_WARN("Can't connected to move base server");
		return 1;
	}
	ROS_INFO("Connected to move base server");
	ROS_INFO("Starting path planning and tracking...");
	visualization_msgs::Marker  marker = init_markers(wayPoints);
	ros::Publisher cmdVelPub = node.advertise<geometry_msgs::Twist>("/raw_cmd_vel", 5);
	ros::Publisher rawcmdVelPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
	cmdVelPubPtr = &cmdVelPub;
	ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("waypoint_markers", 10);

	move_base_msgs::MoveBaseGoal goal;
    geometry_msgs::Twist pub_msg_vel;
	//Cycle through the four waypoints
	string taskParamName = "/usr_task";
	int task=0;
	

	while (1)
	{
		/* code */
		
		//ROS_INFO("Waiting for usr_task 2 ...");
		for(int pointIndex=0; pointIndex < wayPoints.size(); ){
			if(dt == 1) {
			    dt = 0;    
			    ++pointIndex;
			}

			ROS_INFO("Waiting for usr_task 4 ...");
			do {
				ros::param::get(taskParamName,task);
				if(task == 4) 
				{
					ros::param::set("/usr_task",0);
					break;
				}/*else if(task == 3) 
				{
					ROS_INFO("while task == 3 ...");
					actionClient->cancelAllGoals();
					cmdVelPubPtr->publish(geometry_msgs::Twist());
				}*/
				ros::Duration(1).sleep(); 
			}while ( 1 );

			do {
                                ros::param::get("usr_task1",task);
                                if(task == 4)
                                {
                                        ros::param::set("/usr_task1",0);
                                        break;
                                }/*else if(task == 3)
                                {
                                        ROS_INFO("while task == 3 ...");
                                        actionClient->cancelAllGoals();
                                        cmdVelPubPtr->publish(geometry_msgs::Twist());
                                }*/
                                ros::Duration(1).sleep();
                        }while ( 1 );

			//ROS_INFO("start.............");
			marker_pub.publish(marker);
			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();
			goal.target_pose.pose = wayPoints[pointIndex];
			actionClient->sendGoal(goal, Client::SimpleDoneCallback(), &activeCb, &feedbackCb);

			if(actionClient->waitForResult(ros::Duration(6000))){
				if (actionClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
					ROS_INFO("the %d wayPoint arrived!", pointIndex);
					dt = 1;
					if(pointIndex == 1) //ros::param::set("/usr_task",0);
					{
						    ROS_INFO("start ...........!");
							pub_msg_vel.angular.z = 0.1;
							/*while(1){
								cmdVelPubPtr->publish(pub_msg_vel);
								ros::Duration(0.2).sleep();
								if(fabs(real_pose.theta - (wayPoints[1].position.z-3.14)) < 0.1) break;
							}*/
							ros::Duration(1).sleep();
							ros::param::set("/usr_task",4);
							ros::param::set("/usr_task1",4);
					}
					if(pointIndex == 2) //ros::param::set("/usr_task",0);
					{
						f = 0;
						    pointIndex = 0;
							ros::param::set("/usr_task",0);
							ros::param::set("/usr_task1",0);
					}
				}else ROS_INFO("the %d wayPoint not arrived!", pointIndex);
			}else{
				ROS_INFO("Timed out achieving goal!!!");
				actionClient->cancelAllGoals();
				
			}
			

		}
		ros::Duration(1).sleep();

	}

	ROS_INFO("Path planning and tracking  end...");
	return 0;
}
