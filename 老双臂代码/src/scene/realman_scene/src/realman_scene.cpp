#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <vector>
#include <map>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h> 
#include <geometry_msgs/Pose.h>
#include <realman_msgs/Move.h>
#include <cstdlib>

using namespace std;

realman_msgs::Move scen_mag;
std_msgs::UInt8 gripper_msg;
ros::Publisher *cmdVelPubPtr;

void shutdown(int sig)
{
	cmdVelPubPtr->publish(geometry_msgs::Twist());
	ros::Duration(1).sleep(); 
	ROS_INFO("Stopped the robot!");
	ros::shutdown();
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "scene_deploy");
	ros::NodeHandle node;
	ros::Rate loop_rate(10);
	signal(SIGINT, shutdown);
	ROS_INFO("Scene Deploy Node Start...");
	ros::service::waitForService("/robot_driver/move_joint");
	ROS_INFO("Scene wait...");
	ros::ServiceClient client_robot = node.serviceClient<realman_msgs::Move>("/robot_driver/move_joint");
	ros::Publisher gripperPub = node.advertise<std_msgs::UInt8>("/gripper", 5);
	ros::Publisher cmdVelPub = node.advertise<geometry_msgs::Twist>("/raw_cmd_vel", 5);
	cmdVelPubPtr = &cmdVelPub;
	geometry_msgs::Twist pub_msg_vel;
	string taskParamName = "/usr_task";
	int task = 0;
	int cout = 0;
	while(1){
		ROS_INFO("Waiting for usr_task...");
		do {
			ros::param::get(taskParamName,task);
			if(task == 2) break;
			ros::Duration(1).sleep(); 
		}while ( 1 );
		scen_mag.request.pose = {76.25875, 47.23124, 31.91873, -1.30149, 97.02478, -6.73137};
		if(!client_robot.call(scen_mag)) ROS_INFO("Scene1 faild...");
		scen_mag.request.pose = {75.57622, 45.09303, 43.85194, -1.33788, 86.76424, -6.73137};
		if(client_robot.call(scen_mag)){
			ROS_INFO("Scene2 sucessed...");
			ros::Duration(2).sleep();
			gripper_msg.data = 1;
			gripperPub.publish(gripper_msg);
			ros::Duration(3).sleep();
			scen_mag.request.pose = {76.25875, 47.23124, 31.91873, -1.30149, 97.02478, -6.73137};
			if(!client_robot.call(scen_mag)) ROS_INFO("Scene3 faild...");
			scen_mag.request.pose = {-0.43666, 2.27844, 62.56423, -2.41297, 93.93875, -0.02735};
			if(client_robot.call(scen_mag)){
				ROS_INFO("Scene3 sucessed...");
				
				ros::Duration(5).sleep();
				/*while(1){
					pub_msg_vel.linear.x = 0.15;
					cmdVelPubPtr->publish(pub_msg_vel);
					ros::Duration(0.1).sleep(); 
					cout++;

				}*/
				scen_mag.request.pose = {76.48054, 52.14557, 30.55093, -1.22733, 95.82727, -6.73137};
				if(client_robot.call(scen_mag)){
					ROS_INFO("Scene4 sucessed...");
					ros::Duration(2).sleep();
					gripper_msg.data = 0;
					gripperPub.publish(gripper_msg);
					ros::Duration(2).sleep();
					scen_mag.request.pose = {1.38692, -59.27571, 111.76805, -3.11149, 84.17284, -6.72519};
					if(client_robot.call(scen_mag)){
						
						ros::param::set("/usr_task",0);
					}
				}
				
			}else ROS_INFO("Scene3 failed...");
		}else{
			ROS_INFO("Scene2 failed...");
		}
	}
	
	
	/*
	actionClient = new Client("move_base", true);
	if (!actionClient->waitForServer(ros::Duration(60)))
	{
		ROS_WARN("Can't connected to move base server");
		return 1;
	}
	ROS_INFO("Connected to move base server");


	ROS_INFO("Starting path planning and tracking...");
	vector<geometry_msgs::Pose> wayPoints = init_goalList();
	visualization_msgs::Marker  marker = init_markers(wayPoints);
	ros::Publisher cmdVelPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
	cmdVelPubPtr = &cmdVelPub;
	ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("waypoint_markers", 10);
	move_base_msgs::MoveBaseGoal goal;

	std::map<int,int> waypointStationMap;
	////////////////////////////////////////////////////////////////
	waypointStationMap[0]=1;		//设置?号路点为1号station（1号station：抓取工位）
	waypointStationMap[6]=2;		//设置?号路点为2号station（2号station：放置工位）
	/////////////////////////////////////////////////////////////////

	//Cycle through the four waypoints
	for(int pointIndex=0; pointIndex<wayPoints.size(); ++pointIndex){
		if(pointIndex == 0){
			ros::param::set("/station_1/task",1);
		}

		marker_pub.publish(marker);
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose = wayPoints[pointIndex];
		actionClient->sendGoal(goal, Client::SimpleDoneCallback(), &activeCb, &feedbackCb);
		ros::param::set("/zeus_paths_multiple/docked",0);

		if(actionClient->waitForResult(ros::Duration(600))){
			if (actionClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				ROS_INFO("the %d wayPoint arrived!", pointIndex);
				if(waypointStationMap.find(pointIndex) != waypointStationMap.end()){
					ros::param::set("/zeus_paths_multiple/docked", waypointStationMap[pointIndex]);
					string taskParamName = "/station_" + to_string(waypointStationMap[pointIndex])  + "/task";
					int task=0;
					do {
						ros::param::get(taskParamName,task);
						ros::Duration(1).sleep(); 
					}while ( task );
				}
			}
		}else{
			actionClient->cancelAllGoals();
			ROS_WARN("Timed out achieving goal!!!");
			ros::param::set("/zeus_paths_multiple/docked",0);
		}
		ros::Duration(1).sleep();
	}
	*/

	ROS_INFO("Path planning and tracking  end...");
	return 0;
}
