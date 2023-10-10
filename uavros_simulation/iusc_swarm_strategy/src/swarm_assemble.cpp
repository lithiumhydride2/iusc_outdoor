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
// mission state

// int mission_state = 0;
int uav_number = 999;
std::array<float,3> drone_state = {0,0,0};
std::array<float,3> target_pose = {0,0,0};
int target_number = 999;
enum {TOARM = 1, TOLAND = 2, MAYDAY = 3};
enum {NONE = 0, zone = 1, STATIC_RECTANGLE = 2, ROTATING_RECTANGE = 3, CIRCLE = 4, UGV = 5};

//  others_pose
std::unordered_map<int,bool> other_quad_pose_zone;
std::unordered_map<int,bool> other_quad_pose_before_rect;
std::unordered_map<int,bool> other_quad_pose_cross_rect;
std::unordered_map<int,bool> able_to_take_off_recv_map;
bool alle_to_take_off_sing = false;
bool able_to_move_in_room = false;
bool able_to_leave_room = false;

// void mission_state_cb(const iusc_referee::MissionState& tmp);
/*void target_number_sub_cb(std_msgs::Int32 &tmp){
    target_number = tmp;
}
*/

void quad_other_before_rect_sub_cb(const iusc_swarm_strategy::iusc_swarm &tmp){
    if( other_quad_pose_before_rect.count(tmp.uav_number-1)) return;
    other_quad_pose_before_rect[tmp.uav_number-1] = bool(tmp.before_rect_success) ;
}

void quad_other_cross_rect_sub_cb(const iusc_swarm_strategy::iusc_swarm &tmp){
    if( other_quad_pose_cross_rect.count(tmp.uav_number-1)) return;
    other_quad_pose_cross_rect[tmp.uav_number-1] = bool(tmp.cross_rect_success) ;
}

void quad_other_room_arrange_sub_cb(const iusc_swarm_strategy::iusc_swarm &tmp){
    switch(target_number)
    {
    case 3:{
        if (tmp.target_number == 2 && tmp.room_arrange_success == true){
            able_to_move_in_room = true;    
        }
        break;
    }
    case 1:{
        if (tmp.target_number == 3 && tmp.room_arrange_success == true){
            able_to_move_in_room = true;    
        }
        break;
    }
    case 4:{
        if (tmp.target_number == 1 && tmp.room_arrange_success == true){
            able_to_move_in_room = true;    
        }
        break;
    }
    case 0:{
        if (tmp.target_number == 4 && tmp.room_arrange_success == true){
            able_to_move_in_room = true;    
        }
        break;
    }
    case 5:{
        if (tmp.target_number == 0 && tmp.room_arrange_success == true){
            able_to_move_in_room = true;    
        }
        break;
    }
    }
}

void quad_other_room_arrived_sub_cb(const iusc_swarm_strategy::iusc_swarm &tmp){
    if( tmp.target_number == 5 ) 
    able_to_leave_room = tmp.room_arrange_success ;
}

/*
void drone_state_cb(const prometheus_msgs::DroneState &tmp){
    drone_state[0] = tmp.position[0] ;
    drone_state[1] = tmp.position[1] ;
    drone_state[2] = tmp.position[2] ;
}
*/

void drone_state_cb(const geometry_msgs::PoseStamped &tmp){
    drone_state[0] = tmp.pose.position.x ;
    drone_state[1] = tmp.pose.position.y ;
    drone_state[2] = tmp.pose.position.z ;
}

void impossible_mission(ros::NodeHandle &nh, ros::Publisher& goal_pose_pub);


float distance_3f(std::array<float,3> &self_pose,std::array<float,3> &other_pose){
    float distance = .0;
    for(int i = 0 ; i <self_pose.size() ; ++i){
        distance += pow(self_pose[i] - other_pose[i] , 2);
    }
    return sqrt(distance);
}

void impossible_mission(ros::NodeHandle& nh,ros::Publisher& goal_pose_pub){
    XmlRpc::XmlRpcValue one_pose_all;
    XmlRpc::XmlRpcValue one_pose;
    ros::Rate rate_1(1.0);

    //ros::Subscriber drone_state_sub = nh.subscribe("/prometheus/drone_state",10,drone_state_cb);
    
    switch (target_number)
    {
    case 2:
    {       
        if(nh.getParam("/swarm_assemble/one_target_pos_2",one_pose_all)){
        } else ROS_ERROR("param set error");
        for(int i = 0 ;i < one_pose_all.size() ; i++){

            one_pose = one_pose_all[i];
            nh.setParam("/target_pos_x" , one_pose["x"]);
            nh.setParam("/target_pos_y" , one_pose["y"]);
            nh.setParam("/target_pos_z" , one_pose["z"]);
            // planning/goal
            geometry_msgs::PoseStamped first_planning_goal;
            first_planning_goal.header.stamp = ros::Time::now();
            first_planning_goal.pose.position.x = one_pose["x"];
            first_planning_goal.pose.position.y = one_pose["y"];
            first_planning_goal.pose.position.z = one_pose["z"];
            goal_pose_pub.publish(first_planning_goal);

            std::cout << "set way point success  : " << i << std::endl;
            target_pose[0] = first_planning_goal.pose.position.x;
            target_pose[1] = first_planning_goal.pose.position.y;
            target_pose[2] = first_planning_goal.pose.position.z;

            /*ros::Subscriber drone_state_sub = nh.subscribe("/prometheus/drone_state",10,drone_state_cb);
    while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
    */
    switch (uav_number)
    {
    case 1:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav1/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 2:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav2/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 3:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav3/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 4:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav4/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 5:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav5/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 6:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav6/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }
    }
            std::cout << " reach target : " << i << std::endl;

        }
        iusc_swarm_strategy::iusc_swarm room_arrange_success_msg;
        room_arrange_success_msg.target_number = target_number;
        room_arrange_success_msg.room_arrange_success = true;
        ros::Publisher room_arrange_pub = nh.advertise<iusc_swarm_strategy::iusc_swarm>("/able_to_take_off",10 ,true);
        for(int i = 0 ;i < 3 ; i++)
        {
            room_arrange_pub.publish(room_arrange_success_msg);
            rate_1.sleep();
            ros::spinOnce();
        }
        break;
    }
    case 3:
    {
        ros::Subscriber quad_other_room_arrange_sub = nh.subscribe("/able_to_take_off",10,quad_other_room_arrange_sub_cb);
        double time_out = .0;
    
        while (ros::ok() && (time_out < 30) && able_to_move_in_room == false)
        {
            std::cout << "waiting for target 2 arrived"<< std::endl;
            rate_1.sleep();
            time_out ++;
            ros::spinOnce();
        }

        quad_other_room_arrange_sub.shutdown();

        if(nh.getParam("/swarm_assemble/one_target_pos_3",one_pose_all)){
        } else ROS_ERROR("param set error");
        for(int i = 0 ;i < one_pose_all.size() ; i++){

            one_pose = one_pose_all[i];
            nh.setParam("/target_pos_x" , one_pose["x"]);
            nh.setParam("/target_pos_y" , one_pose["y"]);
            nh.setParam("/target_pos_z" , one_pose["z"]);
            // planning/goal
            geometry_msgs::PoseStamped first_planning_goal;
            first_planning_goal.header.stamp = ros::Time::now();
            first_planning_goal.pose.position.x = one_pose["x"];
            first_planning_goal.pose.position.y = one_pose["y"];
            first_planning_goal.pose.position.z = one_pose["z"];
            goal_pose_pub.publish(first_planning_goal);

            std::cout << "set way point success  : " << i << std::endl;
            target_pose[0] = first_planning_goal.pose.position.x;
            target_pose[1] = first_planning_goal.pose.position.y;
            target_pose[2] = first_planning_goal.pose.position.z;

            /*ros::Subscriber drone_state_sub = nh.subscribe("/prometheus/drone_state",10,drone_state_cb);
    while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
    */
    switch (uav_number)
    {
    case 1:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav1/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 2:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav2/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 3:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav3/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 4:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav4/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 5:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav5/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 6:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav6/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }
    }
            std::cout << " reach target : " << i << std::endl;
        }
        iusc_swarm_strategy::iusc_swarm room_arrange_success_msg;
        room_arrange_success_msg.target_number = target_number;
        room_arrange_success_msg.room_arrange_success = true;
        ros::Publisher room_arrange_pub = nh.advertise<iusc_swarm_strategy::iusc_swarm>("/able_to_take_off",10 ,true);
        for(int i = 0 ;i < 3 ; i++)
        {
            room_arrange_pub.publish(room_arrange_success_msg);
            rate_1.sleep();
            ros::spinOnce();
        }
        break;
    }
    case 1:
    {
        ros::Subscriber quad_other_room_arrange_sub = nh.subscribe("/able_to_take_off",10,quad_other_room_arrange_sub_cb);
        double time_out = .0;
    
        while (ros::ok() && (time_out < 30*2) && able_to_move_in_room == false)
        {
            std::cout << "waiting for target 2 arrived"<< std::endl;
            rate_1.sleep();
            time_out ++;
            ros::spinOnce();
        }
        quad_other_room_arrange_sub.shutdown();

        if(nh.getParam("/swarm_assemble/one_target_pos_1",one_pose_all)){
        } else ROS_ERROR("param set error");
        for(int i = 0 ;i < one_pose_all.size() ; i++){

            one_pose = one_pose_all[i];
            nh.setParam("/target_pos_x" , one_pose["x"]);
            nh.setParam("/target_pos_y" , one_pose["y"]);
            nh.setParam("/target_pos_z" , one_pose["z"]);
            // planning/goal
            geometry_msgs::PoseStamped first_planning_goal;
            first_planning_goal.header.stamp = ros::Time::now();
            first_planning_goal.pose.position.x = one_pose["x"];
            first_planning_goal.pose.position.y = one_pose["y"];
            first_planning_goal.pose.position.z = one_pose["z"];
            goal_pose_pub.publish(first_planning_goal);

            std::cout << "set way point success  : " << i << std::endl;
            target_pose[0] = first_planning_goal.pose.position.x;
            target_pose[1] = first_planning_goal.pose.position.y;
            target_pose[2] = first_planning_goal.pose.position.z;

/*ros::Subscriber drone_state_sub = nh.subscribe("/prometheus/drone_state",10,drone_state_cb);
    while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
    */
    switch (uav_number)
    {
    case 1:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav1/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 2:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav2/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 3:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav3/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 4:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav4/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 5:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav5/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 6:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav6/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }
    }
            std::cout << " reach target : " << i << std::endl;
        }
        iusc_swarm_strategy::iusc_swarm room_arrange_success_msg;
        room_arrange_success_msg.target_number = target_number;
        room_arrange_success_msg.room_arrange_success = true;
        ros::Publisher room_arrange_pub = nh.advertise<iusc_swarm_strategy::iusc_swarm>("/able_to_take_off",10 ,true);
        for(int i = 0 ;i < 3 ; i++)
        {
            room_arrange_pub.publish(room_arrange_success_msg);
            rate_1.sleep();
            ros::spinOnce();
        }
        break;
    }
    case 4:
    {
        ros::Subscriber quad_other_room_arrange_sub = nh.subscribe("/able_to_take_off",10,quad_other_room_arrange_sub_cb);
        double time_out = .0;
    
        while (ros::ok() && (time_out < 30*3) && able_to_move_in_room == false)
        {
            std::cout << "waiting for target 1 arrived"<< std::endl;
            rate_1.sleep();
            time_out ++;
            ros::spinOnce();
        }
        quad_other_room_arrange_sub.shutdown();

        if(nh.getParam("/swarm_assemble/one_target_pos_4",one_pose_all)){
        } else ROS_ERROR("param set error");
        for(int i = 0 ;i < one_pose_all.size() ; i++){

            one_pose = one_pose_all[i];
            nh.setParam("/target_pos_x" , one_pose["x"]);
            nh.setParam("/target_pos_y" , one_pose["y"]);
            nh.setParam("/target_pos_z" , one_pose["z"]);
            // planning/goal
            geometry_msgs::PoseStamped first_planning_goal;
            first_planning_goal.header.stamp = ros::Time::now();
            first_planning_goal.pose.position.x = one_pose["x"];
            first_planning_goal.pose.position.y = one_pose["y"];
            first_planning_goal.pose.position.z = one_pose["z"];
            goal_pose_pub.publish(first_planning_goal);

            std::cout << "set way point success  : " << i << std::endl;
            target_pose[0] = first_planning_goal.pose.position.x;
            target_pose[1] = first_planning_goal.pose.position.y;
            target_pose[2] = first_planning_goal.pose.position.z;

/*ros::Subscriber drone_state_sub = nh.subscribe("/prometheus/drone_state",10,drone_state_cb);
    while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
    */
    switch (uav_number)
    {
    case 1:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav1/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 2:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav2/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 3:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav3/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 4:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav4/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 5:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav5/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 6:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav6/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }
    }
            std::cout << " reach target : " << i << std::endl;

        }
        iusc_swarm_strategy::iusc_swarm room_arrange_success_msg;
        room_arrange_success_msg.target_number = target_number;
        room_arrange_success_msg.room_arrange_success = true;
        ros::Publisher room_arrange_pub = nh.advertise<iusc_swarm_strategy::iusc_swarm>("/able_to_take_off",10 ,true);
        for(int i = 0 ;i < 3 ; i++)
        {
            room_arrange_pub.publish(room_arrange_success_msg);
            rate_1.sleep();
            ros::spinOnce();
        }
        break;
    }
    case 0:
    {
        ros::Subscriber quad_other_room_arrange_sub = nh.subscribe("/able_to_take_off",10,quad_other_room_arrange_sub_cb);
        double time_out = .0;
    
        while (ros::ok() && (time_out < 30*4) && able_to_move_in_room == false)
        {
            std::cout << "waiting for target 4 arrived"<< std::endl;
            rate_1.sleep();
            time_out ++;
            ros::spinOnce();
        }
        quad_other_room_arrange_sub.shutdown();

        if(nh.getParam("/swarm_assemble/one_target_pos_0",one_pose_all)){
        } else ROS_ERROR("param set error");

        for(int i = 0 ;i < one_pose_all.size() ; i++){

            one_pose = one_pose_all[i];
            nh.setParam("/target_pos_x" , one_pose["x"]);
            nh.setParam("/target_pos_y" , one_pose["y"]);
            nh.setParam("/target_pos_z" , one_pose["z"]);
            // planning/goal
            geometry_msgs::PoseStamped first_planning_goal;
            first_planning_goal.header.stamp = ros::Time::now();
            first_planning_goal.pose.position.x = one_pose["x"];
            first_planning_goal.pose.position.y = one_pose["y"];
            first_planning_goal.pose.position.z = one_pose["z"];
            goal_pose_pub.publish(first_planning_goal);

            std::cout << "set way point success  : " << i << std::endl;
            target_pose[0] = first_planning_goal.pose.position.x;
            target_pose[1] = first_planning_goal.pose.position.y;
            target_pose[2] = first_planning_goal.pose.position.z;

            /*ros::Subscriber drone_state_sub = nh.subscribe("/prometheus/drone_state",10,drone_state_cb);
    while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
    */
    switch (uav_number)
    {
    case 1:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav1/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 2:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav2/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 3:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav3/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 4:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav4/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 5:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav5/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 6:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav6/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }
    }
            std::cout << " reach target : " << i << std::endl;
        }
        iusc_swarm_strategy::iusc_swarm room_arrange_success_msg;
        room_arrange_success_msg.target_number = target_number;
        room_arrange_success_msg.room_arrange_success = true;
        ros::Publisher room_arrange_pub = nh.advertise<iusc_swarm_strategy::iusc_swarm>("/able_to_take_off",10 ,true);
        for(int i = 0 ;i < 3 ; i++)
        {
            room_arrange_pub.publish(room_arrange_success_msg);
            rate_1.sleep();
            ros::spinOnce();
        }
        break;
    }
    case 5:
    {
        ros::Subscriber quad_other_room_arrange_sub = nh.subscribe("/able_to_take_off",10,quad_other_room_arrange_sub_cb);
        double time_out = .0;
    
        while (ros::ok() && (time_out < 30*5) && able_to_move_in_room == false)
        {
            std::cout << "waiting for target 0 arrived"<< std::endl;
            rate_1.sleep();
            time_out ++;
            ros::spinOnce();
        }
        quad_other_room_arrange_sub.shutdown();
        if(nh.getParam("/swarm_assemble/one_target_pos_5",one_pose_all)){
        } else ROS_ERROR("param set error");

        for(int i = 0 ;i < one_pose_all.size() ; i++){

            one_pose = one_pose_all[i];
            nh.setParam("/target_pos_x" , one_pose["x"]);
            nh.setParam("/target_pos_y" , one_pose["y"]);
            nh.setParam("/target_pos_z" , one_pose["z"]);
            // planning/goal
            geometry_msgs::PoseStamped first_planning_goal;
            first_planning_goal.header.stamp = ros::Time::now();
            first_planning_goal.pose.position.x = one_pose["x"];
            first_planning_goal.pose.position.y = one_pose["y"];
            first_planning_goal.pose.position.z = one_pose["z"];
            goal_pose_pub.publish(first_planning_goal);

            std::cout << "set way point success  : " << i << std::endl;
            target_pose[0] = first_planning_goal.pose.position.x;
            target_pose[1] = first_planning_goal.pose.position.y;
            target_pose[2] = first_planning_goal.pose.position.z;

            /*ros::Subscriber drone_state_sub = nh.subscribe("/prometheus/drone_state",10,drone_state_cb);
    while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
    */
    switch (uav_number)
    {
    case 1:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav1/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 2:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav2/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 3:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav3/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 4:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav4/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 5:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav5/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 6:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav6/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }
    }
            std::cout << " reach target : " << i << std::endl;
        }

        iusc_swarm_strategy::iusc_swarm room_arrange_success_msg;
        room_arrange_success_msg.target_number = target_number;
        room_arrange_success_msg.room_arrange_success = true;
        ros::Publisher room_arrange_pub = nh.advertise<iusc_swarm_strategy::iusc_swarm>("/able_to_take_off",10 ,true);
        for(int i = 0 ;i < 3 ; i++)
        {
            room_arrange_pub.publish(room_arrange_success_msg);
            rate_1.sleep();
            ros::spinOnce();
        }
        break;
    }
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
    

    // 等待迷宫区域结束并获取target number
/*    ros::Subscriber target_number_sub = nh.subscribe("/target",10,target_number_sub_cb);

    while(ros::ok() && !(target_number>=0 && target_number<=5)){
        std::cout << " ------- wait target number --------" << std::endl;
        rate_2.sleep();
        ros::spinOnce();
    }
    target_number_sub.shutdown();
*/
    XmlRpc::XmlRpcValue target_number_receive;
    while(ros::ok() && !(target_number>=0 && target_number<=5)){
        std::cout << " ------- wait target number --------" << std::endl;
        if(nh.getParam("target_number",target_number_receive))
        {
            target_number = target_number_receive;}
            else 
            {ROS_ERROR("target number not received");}
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
        if(other_quad_pose_before_rect.size() == 5){
            int count = 0;
            for(auto tmp: other_quad_pose_before_rect)
                if(tmp.second) count ++;
            // reach!!!!
            if(count == 5)
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
        if(uav_number >=0 && uav_number <=5){
            rect_pose = rect_pose_all[target_number];
            nh.setParam("/target_pos_x" , rect_pose["x"]);
            nh.setParam("/target_pos_y" , rect_pose["y"]);
            nh.setParam("/target_pos_z" , rect_pose["z"]);
        // change target pose
            target_pose[0] = nh.param("/target_pos_x",.0);
            target_pose[1] = nh.param("/target_pos_y",.0);
            target_pose[2] = nh.param("/target_pos_z",.0);
	    // publish planning goal
            std::cout << "set way point after rect success!" << std::endl;
        }
        else{
            ROS_ERROR("wrong uav unmber!");
        }
    } else ROS_ERROR("param set error before rect");

    // wait for cross rect
    //ros::Subscriber drone_state_sub = nh.subscribe("/prometheus/drone_state",10,drone_state_cb);
/*
    while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
    */
    switch (uav_number)
    {
    case 1:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav1/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 2:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav2/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 3:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav3/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 4:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav4/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 5:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav5/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 6:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav6/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }
    }

/*   // publish to other uav
    iusc_swarm_strategy::iusc_swarm cross_rect_success_msg;
    cross_rect_success_msg.target_number = target_number;
    cross_rect_success_msg.cross_rect_success = true;
    ros::Publisher cross_rect_pub = nh.advertise<iusc_swarm_strategy::iusc_swarm>("/able_to_take_off",10 ,true);
    for(int i = 0 ;i < 3 ; i++)
    {
        cross_rect_pub.publish(cross_rect_success_msg);
        rate_1.sleep();
        ros::spinOnce();
    }

    ros::Subscriber quad_other_cross_rect_sub = nh.subscribe("/able_to_take_off",10,quad_other_cross_rect_sub_cb);
    double time_out_2 = .0;
    
    while (ros::ok() && (time_out < 5))
    {
        if(other_quad_pose_cross_rect.size() == 5){
            int count = 0;
            for(auto tmp: other_quad_pose_cross_rect)
                if(tmp.second) count ++;
            // reach!!!!
            if(count == 5)
                break;
        }
        rate_1.sleep();
        time_out_2 ++;
        ros::spinOnce();
    }
    std::cout << " -------- all crossed rect , use time: "<< time_out << "-------------- " << std::endl;
    quad_other_cross_rect_sub.shutdown();
    std::cout << "---- to finish impossible mission ---" << std::endl;
*/ 
    // when_to_cross_one(nh);
    impossible_mission(nh,goal_pose_pub);
/*    switch(target_number)
    {
        case 5:
        {
        iusc_swarm_strategy::iusc_swarm room_arrange_success_msg;
        room_arrange_success_msg.target_number = target_number;
        room_arrange_success_msg.room_arrange_success = true;
        ros::Publisher room_arrange_pub = nh.advertise<iusc_swarm_strategy::iusc_swarm>("/able_to_take_off",10 ,true);
        for(int i = 0 ;i < 6 ; i++)
        {
            room_arrange_pub.publish(room_arrange_success_msg);
            rate_1.sleep();
            ros::spinOnce();
        }
        break;
        }
        
        default:
        {
        ros::Subscriber quad_other_room_arrived_sub = nh.subscribe("/able_to_take_off",10,quad_other_room_arrived_sub_cb);
        double time_out = .0;
    
        while (ros::ok() && able_to_leave_room == false)
        {
            std::cout << "waiting for target 6 arrived"<< std::endl;
            rate_1.sleep();
            time_out ++;
            ros::spinOnce();
        }
        quad_other_room_arrived_sub.shutdown();
        break;
        }
    }
    // 通过增加time_out避碰
    switch(target_number)
    {
        case 2:
        {
        for(int i = 0 ;i < 1 ; i++)
        {
            rate_1.sleep();
            ros::spinOnce();
        }
        break;
        }
        
        case 3:
        {
        for(int i = 0 ;i < 3 ; i++)
        {
            rate_1.sleep();
            ros::spinOnce();
        }
        break;
        }

        case 1:
        {
        for(int i = 0 ;i < 5 ; i++)
        {
            rate_1.sleep();
            ros::spinOnce();
        }
        break;
        }
        
        case 4:
        {
        for(int i = 0 ;i < 7 ; i++)
        {
            rate_1.sleep();
            ros::spinOnce();
        }
        break;
        }

        case 0:
        {
        for(int i = 0 ;i < 9 ; i++)
        {
            rate_1.sleep();
            ros::spinOnce();
        }
        break;
        }
        
        case 5:
        {
        for(int i = 0 ;i < 11 ; i++)
        {
            rate_1.sleep();
            ros::spinOnce();
        }
        break;
        }

    if(nh.getParam("/swarm_assemble/out_of_room_pos",rect_pose_all)){
        if(uav_number >=1 && uav_number <=6){
            rect_pose = rect_pose_all[target_number];
            nh.setParam("/target_pos_x" , rect_pose["x"]);
            nh.setParam("/target_pos_y" , rect_pose["y"]);
            nh.setParam("/target_pos_z" , rect_pose["z"]);
        // change target pose
            target_pose[0] = nh.param("/target_pos_x",.0);
            target_pose[1] = nh.param("/target_pos_y",.0);
            target_pose[2] = nh.param("/target_pos_z",.0);
	    // publish planning goal
            std::cout << "set way point out of room success!" << std::endl;
        }
        else{
            ROS_ERROR("wrong uav unmber!");
        }
    } else ROS_ERROR("param set error before rect");

    ros::Subscriber drone_state_sub = nh.subscribe("/prometheus/drone_state",10,drone_state_cb);
    while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
    
    switch (uav_number)
    {
    case 1:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav1/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 2:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav2/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 3:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav3/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 4:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav4/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 5:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav5/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }

    case 6:
        {ros::Subscriber drone_state_sub = nh.subscribe("/uav6/mavros/local_position/pose",10,drone_state_cb);
        while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
        }
        drone_state_sub.shutdown(); // shutdown subscriber
        break;
        }
    }

}
*/
}
