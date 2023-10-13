#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
//#include "iusc_referee/MissionState.h"
//#include "iusc_referee/QuadPoseOther.h"
//#include "iusc_referee/StaticTargetOther.h"
#include "iusc_swarm_strategy/iusc_swarm.h"
#include <unistd.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <unordered_map>
#include <array>
//#include <prometheus_msgs/DroneState.h>
#include <unordered_map>
#include <iusc_maze/map2local.h>
// mission state

// int mission_state = 0;
int uav_number = 999;
std::array<float,3> drone_state = {0,0,0};
std::array<float,3> target_pose = {0,0,0};
std::unordered_map<int,bool> other_quad_pose_before_rect;

void quad_other_before_rect_sub_cb(const iusc_swarm_strategy::iusc_swarm &tmp){
    if( other_quad_pose_before_rect.count(tmp.uav_number-1)) return;
    other_quad_pose_before_rect[tmp.uav_number-1] = bool(tmp.before_rect_success) ;
}

void drone_state_cb(const geometry_msgs::PoseStamped &tmp){
    drone_state[0] = tmp.pose.position.x ;
    drone_state[1] = tmp.pose.position.y ;
    drone_state[2] = tmp.pose.position.z ;
}

float distance_3f(geometry_msgs::PoseStamped &self_pose,std::array<float,3> &other_pose){
    float distance = .0;
    distance = pow(self_pose.pose.position.x - other_pose[0] , 2) + pow(self_pose.pose.position.y - other_pose[1] , 2) + pow(self_pose.pose.position.z - other_pose[2] , 2);
    return sqrt(distance);
}

void cross_the_room(ros::NodeHandle& nh,ros::Publisher& goal_pose_pub){
    XmlRpc::XmlRpcValue leave_room_pose_all;
    XmlRpc::XmlRpcValue leave_room_pose;
    ros::Rate rate(20);
    ros::Rate rate_1(1.0);

    //ros::Subscriber drone_state_sub = nh.subscribe("/prometheus/drone_state",10,drone_state_cb);
    double time_out = .0;
    double time_wait = .0;
    switch (uav_number)
    {
    case 1:
    {       
        time_wait = 0.0;
        break;
    }
    case 2:
    {
        time_wait = 15.0;
        break;
    }
    case 3:
    {
        time_wait = 30.0;
        break;
    }
    case 4:
    {
        time_wait = 45.0;
        break;
    }
    case 5:
    {
        time_wait = 60.0;
        break;
    }
    case 6:
    {
        time_wait = 75.0;
        break;
    }
    }
    while (ros::ok() && (time_out < time_wait))
    {
        std::cout << "wait to cross"<< std::endl;
        rate_1.sleep();
        time_out ++;
        ros::spinOnce();
    }

    if(nh.getParam("/swarm_assemble/leave_room_target_pos",leave_room_pose_all)){
        } else ROS_ERROR("param set error");
    
    for(int i = 0 ;i < leave_room_pose_all.size() ; i++){

        leave_room_pose = leave_room_pose_all[i];
        nh.setParam("/target_pos_x" , leave_room_pose["x"]);
        nh.setParam("/target_pos_y" , leave_room_pose["y"]);
        nh.setParam("/target_pos_z" , leave_room_pose["z"]);
        // planning/goal
        geometry_msgs::PoseStamped first_planning_goal;
        first_planning_goal.header.stamp = ros::Time::now();
        first_planning_goal.pose.position.x = leave_room_pose["x"];
        first_planning_goal.pose.position.y = leave_room_pose["y"];
        first_planning_goal.pose.position.z = leave_room_pose["z"];
        goal_pose_pub.publish(first_planning_goal);

        std::cout << "set way point success  : " << i << std::endl;
        target_pose[0] = first_planning_goal.pose.position.x;
        target_pose[1] = first_planning_goal.pose.position.y;
        target_pose[2] = first_planning_goal.pose.position.z;
        
       // 坐标转换
        ros::ServiceClient map2local_client = nh.serviceClient<iusc_maze::map2local>("/map2local_server");// 请替换为实际的服务名称
        // 等待服务可用
        /*
        if (!map2local_client.waitForExistence()) {
            ROS_ERROR("Service not available.");
            return 1;
        }
        */
        while (ros::ok() && !map2local_client.waitForExistence()) {
            ROS_ERROR("Service not available.");
            rate.sleep();
            ros::spinOnce();
        }
        // 创建服务请求
        iusc_maze::map2local srv;
        srv.request.x_map = target_pose[0];
        srv.request.y_map = target_pose[1];
        //发送服务请求
        /*   if (map2local_client.call(srv)){
                if (srv.response.success) {
                    ROS_INFO("Service call succeeded. Local X: %f, Local Y: %f",srv.response.x_local,srv.response.y_local);
                // publish planning goal
                    ros::Publisher waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10, true);
                    geometry_msgs::PoseStamped dsr_pose;
                    dsr_pose.header.frame_id = "map";
                    dsr_pose.header.stamp = ros::Time::now();
                    dsr_pose.pose.position.x = srv.response.x_local;
                    dsr_pose.pose.position.y = srv.response.y_local;
                    dsr_pose.pose.position.z = target_pose[2];
                    waypoint_pub.publish(dsr_pose);
	                std::cout << "set way point after rect success!" << std::endl;
                } 
                else{
                    ROS_ERROR("Service call failed.");
                }
            }
             else {
                ROS_ERROR("Failed to call service.");
             }
        */
        while (ros::ok() && !(map2local_client.call(srv)) )
        {
            ROS_ERROR("Failed to call service.");
            rate.sleep();
            ros::spinOnce();
        }
        // publish planning goal
        ROS_INFO("Service call succeeded. Local X: %f, Local Y: %f",srv.response.x_local,srv.response.y_local);
        ros::Publisher waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10, true);
        geometry_msgs::PoseStamped dsr_pose;
        dsr_pose.header.frame_id = "map";
        dsr_pose.header.stamp = ros::Time::now();
        dsr_pose.pose.position.x = srv.response.x_local;
        dsr_pose.pose.position.y = srv.response.y_local;
        dsr_pose.pose.position.z = target_pose[2];
        waypoint_pub.publish(dsr_pose);

        //ros::Subscriber drone_state_sub = nh.subscribe("/prometheus/drone_state",10,drone_state_cb);
        ros::Subscriber drone_state_sub = nh.subscribe("/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
            float distance = distance_3f(dsr_pose,drone_state) ;
            std::cout << " --- distance to way point  : " << distance << std::endl;
            if ( distance_3f(dsr_pose,drone_state) < 0.3 ) break;
            rate_1.sleep();
            ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        std::cout << " reach target : " << i << std::endl;
    }

    if(nh.getParam("/swarm_assemble/out_of_room_target_pos",rect_pose_all)){
        if(uav_number >=1 && uav_number <=6){
            rect_pose = rect_pose_all[uav_number];
            nh.setParam("/target_pos_x" , rect_pose["x"]);
            nh.setParam("/target_pos_y" , rect_pose["y"]);
            nh.setParam("/target_pos_z" , rect_pose["z"]);
        // change target pose
            target_pose[0] = nh.param("/target_pos_x",.0);
            target_pose[1] = nh.param("/target_pos_y",.0);
            target_pose[2] = nh.param("/target_pos_z",.0);
       
       // 坐标转换
        ros::ServiceClient map2local_client = nh.serviceClient<iusc_maze::map2local>("/map2local_server");
        // 等待服务可用
        /*
        if (!map2local_client.waitForExistence()) {
            ROS_ERROR("Service not available.");
            return 1;
        }
        */
        while (ros::ok() && !map2local_client.waitForExistence()) {
            ROS_ERROR("Service not available.");
            rate.sleep();
            ros::spinOnce();
        }
        // 创建服务请求
        iusc_maze::map2local srv;
        srv.request.x_map = target_pose[0];
        srv.request.y_map = target_pose[1];
        //发送服务请求
        /*   if (map2local_client.call(srv)){
                if (srv.response.success) {
                    ROS_INFO("Service call succeeded. Local X: %f, Local Y: %f",srv.response.x_local,srv.response.y_local);
                // publish planning goal
                    ros::Publisher waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10, true);
                    geometry_msgs::PoseStamped dsr_pose;
                    dsr_pose.header.frame_id = "map";
                    dsr_pose.header.stamp = ros::Time::now();
                    dsr_pose.pose.position.x = srv.response.x_local;
                    dsr_pose.pose.position.y = srv.response.y_local;
                    dsr_pose.pose.position.z = target_pose[2];
                    waypoint_pub.publish(dsr_pose);
	                std::cout << "set way point after rect success!" << std::endl;
                } 
                else{
                    ROS_ERROR("Service call failed.");
                }
            }
             else {
                ROS_ERROR("Failed to call service.");
             }
        */
        while (ros::ok() && !(map2local_client.call(srv)) )
        {
            ROS_ERROR("Failed to call service.");
            rate.sleep();
            ros::spinOnce();
        }
        // publish planning goal
        ROS_INFO("Service call succeeded. Local X: %f, Local Y: %f",srv.response.x_local,srv.response.y_local);
        ros::Publisher waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10, true);
        geometry_msgs::PoseStamped dsr_pose;
        dsr_pose.header.frame_id = "map";
        dsr_pose.header.stamp = ros::Time::now();
        dsr_pose.pose.position.x = srv.response.x_local;
        dsr_pose.pose.position.y = srv.response.y_local;
        dsr_pose.pose.position.z = target_pose[2];
        waypoint_pub.publish(dsr_pose);
        std::cout << "set way point after rect success!" << std::endl;
            // wait for cross rect
        //ros::Subscriber drone_state_sub = nh.subscribe("/prometheus/drone_state",10,drone_state_cb);
        ros::Subscriber drone_state_sub = nh.subscribe("/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
            float distance = distance_3f(dsr_pose,drone_state) ;
            std::cout << " --- distance to way point  : " << distance << std::endl;
            if ( distance < 0.3 ) break;
            rate_1.sleep();
            ros::spinOnce();
            }
        drone_state_sub.shutdown(); // shutdown subscriber
        }
        else{
            ROS_ERROR("wrong uav unmber!");
            }
    } else ROS_ERROR("param set error before rect");

    std_msgs::String task2_status;
    task2_status = "task 2 done";
    ros::Publisher task2_status_msg = nh.advertise<std_msgs::String>("/Task2Done",10 ,true);
    for(int i = 0 ;i < 3 ; i++)
    {
        room_arrange_pub.publish(task2_status_msg);
        rate_1.sleep();
        ros::spinOnce();
    }
}

int main(int argc,char **argv){
    ros::init(argc,argv,"swarm");
    ros::NodeHandle nh("~");
    //ros::Subscriber mission_state_sub = nh.subscribe("/MissionState", 10, mission_state_cb);

    ros::Publisher goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/planning/goal",10,true);
    ros::Rate rate(20);
    ros::Rate rate_1(1);
    ros::Rate rate_2(0.1);
    
    XmlRpc::XmlRpcValue target_number_receive;
    while(ros::ok() && !(target_number>=0 && target_number<=5)){
        std::cout << " ------- wait target number --------" << std::endl;
        if(nh.getParam("target",target_number_receive))
        {
            target_number = target_number_receive;}
        else 
        {
            ROS_ERROR("target number not received");}
        rate_2.sleep();
        ros::spinOnce();
    }

    // 向其他无人机传输自己已到达窗户前集结点
    iusc_swarm_strategy::iusc_swarm before_rect_success_msg;
    before_rect_success_msg.target_number = target_number;
    before_rect_success_msg.before_rect_success = true;
    ros::Publisher before_rect_pub = nh.advertise<iusc_swarm_strategy::iusc_swarm>("/able_to_take_off",10 ,true);
    for(int i = 0 ;i < 3 ; i++)
    {
        before_rect_pub.publish(before_rect_success_msg);
        rate_1.sleep();
        ros::spinOnce();
    }

    // 等待所有无人机均到达窗户前集结点,60秒未等到则直接穿越

    ros::Subscriber quad_other_before_rect_sub = nh.subscribe("/able_to_take_off",10,quad_other_before_rect_sub_cb);
    double time_out = .0;
    
    while (ros::ok() && (time_out < 60))
    {
        if(other_quad_pose_before_rect.size() == 2)
        //if(other_quad_pose_before_rect.size() == 5)
        {
            int count = 0;
            for(auto tmp: other_quad_pose_before_rect)
                if(tmp.second) count ++;
            // reach!!!!
            if(count == 2) //三架无人机
            //if(count == 5) //六架无人机
                break;
        }
        rate_1.sleep();
        time_out ++;
        ros::spinOnce();
    }
    std::cout << " -------- all reach before rect , use time: "<< time_out << "-------------- " << std::endl;
    quad_other_before_rect_sub.shutdown();

    XmlRpc::XmlRpcValue rect_pose_all;
    XmlRpc::XmlRpcValue rect_pose;
    if(nh.getParam("/swarm_assemble/cross_rect_target_pos",rect_pose_all)){
        if(target_number >=0 && target_number <=5){
            rect_pose = rect_pose_all[target_number];
            nh.setParam("/target_pos_x" , rect_pose["x"]);
            nh.setParam("/target_pos_y" , rect_pose["y"]);
            nh.setParam("/target_pos_z" , rect_pose["z"]);
        // change target pose
            target_pose[0] = nh.param("/target_pos_x",.0);
            target_pose[1] = nh.param("/target_pos_y",.0);
            target_pose[2] = nh.param("/target_pos_z",.0);
       
       // 坐标转换
        ros::ServiceClient map2local_client = nh.serviceClient<iusc_maze::map2local>("/map2local_server");
        // 等待服务可用
        /*
        if (!map2local_client.waitForExistence()) {
            ROS_ERROR("Service not available.");
            return 1;
        }
        */
        while (ros::ok() && !map2local_client.waitForExistence()) {
            ROS_ERROR("Service not available.");
            rate.sleep();
            ros::spinOnce();
        }
        // 创建服务请求
        iusc_maze::map2local srv;
        srv.request.x_map = target_pose[0];
        srv.request.y_map = target_pose[1];
        //发送服务请求
        /*   if (map2local_client.call(srv)){
                if (srv.response.success) {
                    ROS_INFO("Service call succeeded. Local X: %f, Local Y: %f",srv.response.x_local,srv.response.y_local);
                // publish planning goal
                    ros::Publisher waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10, true);
                    geometry_msgs::PoseStamped dsr_pose;
                    dsr_pose.header.frame_id = "map";
                    dsr_pose.header.stamp = ros::Time::now();
                    dsr_pose.pose.position.x = srv.response.x_local;
                    dsr_pose.pose.position.y = srv.response.y_local;
                    dsr_pose.pose.position.z = target_pose[2];
                    waypoint_pub.publish(dsr_pose);
	                std::cout << "set way point after rect success!" << std::endl;
                } 
                else{
                    ROS_ERROR("Service call failed.");
                }
            }
             else {
                ROS_ERROR("Failed to call service.");
             }
        */
        while (ros::ok() && !(map2local_client.call(srv)) )
        {
            ROS_ERROR("Failed to call service.");
            rate.sleep();
            ros::spinOnce();
        }
        // publish planning goal
        ROS_INFO("Service call succeeded. Local X: %f, Local Y: %f",srv.response.x_local,srv.response.y_local);
        ros::Publisher waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10, true);
        geometry_msgs::PoseStamped dsr_pose;
        dsr_pose.header.frame_id = "map";
        dsr_pose.header.stamp = ros::Time::now();
        dsr_pose.pose.position.x = srv.response.x_local;
        dsr_pose.pose.position.y = srv.response.y_local;
        dsr_pose.pose.position.z = target_pose[2];
        waypoint_pub.publish(dsr_pose);
        std::cout << "set way point after rect success!" << std::endl;
            // wait for cross rect
        //ros::Subscriber drone_state_sub = nh.subscribe("/prometheus/drone_state",10,drone_state_cb);
        ros::Subscriber drone_state_sub = nh.subscribe("/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
            float distance = distance_3f(dsr_pose,drone_state) ;
            std::cout << " --- distance to way point  : " << distance << std::endl;
            if ( distance < 0.3 ) break;
            rate_1.sleep();
            ros::spinOnce();
            }
        drone_state_sub.shutdown(); // shutdown subscriber
        }
        else{
            ROS_ERROR("wrong uav unmber!");
            }
    } else ROS_ERROR("param set error before rect");
    
    //框后点协调点
    if(nh.getParam("/swarm_assemble/after_rect_target_pos",rect_pose_all)){
        if(target_number >=0 && target_number <=5){
            rect_pose = rect_pose_all[target_number];
            nh.setParam("/target_pos_x" , rect_pose["x"]);
            nh.setParam("/target_pos_y" , rect_pose["y"]);
            nh.setParam("/target_pos_z" , rect_pose["z"]);
        // change target pose
            target_pose[0] = nh.param("/target_pos_x",.0);
            target_pose[1] = nh.param("/target_pos_y",.0);
            target_pose[2] = nh.param("/target_pos_z",.0);
       
       // 坐标转换
        ros::ServiceClient map2local_client = nh.serviceClient<iusc_maze::map2local>("/map2local_server");
        // 等待服务可用
        /*
        if (!map2local_client.waitForExistence()) {
            ROS_ERROR("Service not available.");
            return 1;
        }
        */
        while (ros::ok() && !map2local_client.waitForExistence()) {
            ROS_ERROR("Service not available.");
            rate.sleep();
            ros::spinOnce();
        }
        // 创建服务请求
        iusc_maze::map2local srv;
        srv.request.x_map = target_pose[0];
        srv.request.y_map = target_pose[1];
        //发送服务请求
        /*   if (map2local_client.call(srv)){
                if (srv.response.success) {
                    ROS_INFO("Service call succeeded. Local X: %f, Local Y: %f",srv.response.x_local,srv.response.y_local);
                // publish planning goal
                    ros::Publisher waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10, true);
                    geometry_msgs::PoseStamped dsr_pose;
                    dsr_pose.header.frame_id = "map";
                    dsr_pose.header.stamp = ros::Time::now();
                    dsr_pose.pose.position.x = srv.response.x_local;
                    dsr_pose.pose.position.y = srv.response.y_local;
                    dsr_pose.pose.position.z = target_pose[2];
                    waypoint_pub.publish(dsr_pose);
	                std::cout << "set way point after rect success!" << std::endl;
                } 
                else{
                    ROS_ERROR("Service call failed.");
                }
            }
             else {
                ROS_ERROR("Failed to call service.");
             }
        */
        while (ros::ok() && !(map2local_client.call(srv)) )
        {
            ROS_ERROR("Failed to call service.");
            rate.sleep();
            ros::spinOnce();
        }
        // publish planning goal
        ROS_INFO("Service call succeeded. Local X: %f, Local Y: %f",srv.response.x_local,srv.response.y_local);
        ros::Publisher waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10, true);
        geometry_msgs::PoseStamped dsr_pose;
        dsr_pose.header.frame_id = "map";
        dsr_pose.header.stamp = ros::Time::now();
        dsr_pose.pose.position.x = srv.response.x_local;
        dsr_pose.pose.position.y = srv.response.y_local;
        dsr_pose.pose.position.z = target_pose[2];
        waypoint_pub.publish(dsr_pose);
        std::cout << "set way point after rect success!" << std::endl;
            // wait for cross rect
        //ros::Subscriber drone_state_sub = nh.subscribe("/prometheus/drone_state",10,drone_state_cb);
        ros::Subscriber drone_state_sub = nh.subscribe("/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
            float distance = distance_3f(dsr_pose,drone_state) ;
            std::cout << " --- distance to way point  : " << distance << std::endl;
            if ( distance < 0.3 ) break;
            rate_1.sleep();
            ros::spinOnce();
            }
        drone_state_sub.shutdown(); // shutdown subscriber
        }
        else{
            ROS_ERROR("wrong uav unmber!");
            }
    } else ROS_ERROR("param set error before rect");


    cross_the_room(nh,goal_pose_pub);
}