#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
//#include "iusc_referee/MissionState.h"
//#include "iusc_referee/QuadPoseOther.h"
//#include "iusc_referee/StaticTargetOther.h"
#include "iusc_swarm_strategy/iusc_swarm.h"
#include <unistd.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <unordered_map>
#include <array>
#include <prometheus_msgs/DroneState.h>
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
void quad_other_zone_sub_cb(const iusc_referee::QuadPoseOther &tmp);
void quad_other_before_rect_sub_cb(const iusc_swarm_strategy::iusc_swarm &tmp);
void quad_other_cross_rect_sub_cb(const iusc_swarm_strategy::iusc_swarm &tmp);
void quad_other_room_arrange_sub_cb(const iusc_swarm_strategy::iusc_swarm &tmp);
void quad_other_room_arrived_sub_cb(const iusc_swarm_strategy::iusc_swarm &tmp);
void drone_state_cb(const prometheus_msgs::DroneState &tmp);
void when_to_set_up(ros::NodeHandle &nh);
void impossible_mission(ros::NodeHandle &nh, ros::Publisher& goal_pose_pub);
void able_to_take_off_recv_cb(const iusc_swarm_strategy::iusc_swarm &tmp);
void when_to_cross_one(ros::NodeHandle &nh);

float distance_3f(std::array<float,3> &self_pose,std::array<float,3> &other_pose){
    float distance = .0;
    for(int i = 0 ; i <self_pose.size() ; ++i){
        distance += pow(self_pose[i] - other_pose[i] , 2);
    }
    return sqrt(distance);
}


int main(int argc,char **argv){
    ros::init(argc,argv,"swarm");
    ros::NodeHandle nh("~");
    ros::Subscriber mission_state_sub = nh.subscribe("/MissionState", 10, mission_state_cb);

    ros::Publisher goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/planning/goal",10,true);
    
    ros::Rate rate(20);
    ros::Rate rate_1(1);
    
    // wait for armcommand 
 /*   nh.setParam("/finished_stage",NONE);
    
    while (ros::ok() && !(mission_state == TOARM))
    {
        std::cout << "---------- wait for arm command-----------" << std::endl;
        rate_1.sleep();       
        ros::spinOnce();
    }
    // receive arm command
    std::cout << "----------- receive arm command !!! --------" << std::endl;
    // shutdown this subscriber
    mission_state_sub.shutdown();
*/
    // 在此处进行起飞顺序的批准
    // 起飞顺序
    // when_to_set_up(nh);

/*
    XmlRpc::XmlRpcValue rect_pose_all;
    XmlRpc::XmlRpcValue rect_pose;
    if(nh.getParam("/swarm_assemble/first_target_pos",rect_pose_all)){
        if(uav_number >=0 && uav_number <=5){
            rect_pose = rect_pose_all[uav_number];
            nh.setParam("/target_pos_x" , rect_pose["x"]);
            nh.setParam("/target_pos_y" , rect_pose["y"]);
            nh.setParam("/target_pos_z" , rect_pose["z"]);
            // planning/goal
            geometry_msgs::PoseStamped first_planning_goal;
            first_planning_goal.header.stamp = ros::Time::now();
            first_planning_goal.pose.position.x = rect_pose["x"];
            first_planning_goal.pose.position.y = rect_pose["y"];
            first_planning_goal.pose.position.z = rect_pose["z"];
            goal_pose_pub.publish(first_planning_goal);

            std::cout << "set way point success , uav is : " << uav_number << std::endl;
        }
        else{
            ROS_ERROR("wrong uav unmber!");
        }
    } else ROS_ERROR("param set error");
*/

    //ready to cross zone
    std::cout << "--------- ready to cross the zone ---------" << std::endl;
    nh.setParam("/ready_to_cross_zone" , true);
    while (ros::ok() && !(nh.param("/finish_cross_zone" , false)))
    {
        std::cout << " -- crossing zone --" << std::endl;
        rate_1.sleep();
    }
    std::cout << " ----- crossed zone & standby in front of rect------ " << std::endl;
    nh.setParam("/finished_stage",zone);

    //  receive from other
    ros::Subscriber quad_other_zone_sub = nh.subscribe("/QuadPoseOther",10,quad_other_zone_sub_cb);
    double time_out = .0;
    
    // waiting for other uavs to cross the zone, but the waiting period need to be considered (now 20s)
    while (ros::ok() && (count < 20))
    {
        if(other_quad_pose_zone.size() == 5){
            int count = 0;
            for(auto tmp: other_quad_pose_zone)
                if(tmp.second) count ++;
            // reach!!!!
            if(count == 5)
                break;
        }
        rate_1.sleep();
        time_out ++;
        ros::spinOnce();
    }
    std::cout << " -------- all crossed zone , use time: "<< time_out << "-------------- " << std::endl;
    quad_other_zone_sub.shutdown();
    nh.setParam("/go_cross_rect",true);   
    // set flight point after rect
    // 对于本无人机到达了几号航点需要进行获取，topic名后续确定
    ros::Subscriber target_number_sub = nh.subscribe("/TargetNumber", 10, target_number_cb);
    double time_wait = .0;
    while(ros::ok() && !(target>=1 && target<=6)){
        std::cout << " ------- wait target number , use time: "<< time_wait << " --------" << std::endl;
        rate_1.sleep();
        ros::spinOnce();
    }
    target_number_sub.shutdown();
    std::cout << " ------- ready to cross rect --------" << std::endl;
    //
    if(nh.getParam("/swarm_assemble/rect_target_pos",rect_pose_all)){
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
            std::cout << "set way point before rect success!" << std::endl;
        }
        else{
            ROS_ERROR("wrong uav unmber!");
        }
    } else ROS_ERROR("param set error before rect");


    // subscribe drone state
    ros::Subscriber drone_state_sub = nh.subscribe("/prometheus/drone_state",10,drone_state_cb);
    while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
    }
    std::cout << "reach way point before rect success!" << std::endl;

    // set flight point cross rect
    std::cout << " ------- crossing rect --------" << std::endl;
 
    int time_out = 0;
 /*   while (ros::ok() && ( time_count < out_time ))
    {
        std::cout << "wait for other uav -----" << std::endl;
        rate_1.sleep();
        time_count ++;
        ros::spinOnce();
    } 
*/
  // waiting for other uavs to arrived before rect
  // publish to other uav
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
  // ros bridge need to add "/able_to_take_off"
    ros::Subscriber quad_other_before_rect_sub = nh.subscribe("/able_to_take_off",10,quad_other_before_rect_sub_cb);
    double time_out = .0;
    
    while (ros::ok() && (time_out < 5))
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
    while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
    }
    
    drone_state_sub.shutdown(); // shutdown subscriber
    nh.setParam("/cross_rect_success",true);
    // publish to other uav
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
    double time_out = .0;
    
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
        time_out ++;
        ros::spinOnce();
    }
    std::cout << " -------- all crossed rect , use time: "<< time_out << "-------------- " << std::endl;
    quad_other_cross_rect_sub.shutdown();
    std::cout << "---- to finish impossible mission ---" << std::endl;
    when_to_cross_one(nh);
    impossible_mission(nh,goal_pose_pub);
    switch(target_number)
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
        }
        
        default:
        {
        ros::Subscriber quad_other_room_arrived_sub = nh.subscribe("/able_to_take_off",10,quad_other_room_arrived_sub_cb);
        double time_out = .0;
    
        while (ros::ok() && able_to_leave_room = false)
        {
            std::cout << "waiting for target 6 arrived"<< std::endl;
            rate_1.sleep();
            time_out ++;
            ros::spinOnce();
        }
        quad_other_room_arrived_sub.shutdown();
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
        }
        
        case 3:
        {
        for(int i = 0 ;i < 3 ; i++)
        {
            rate_1.sleep();
            ros::spinOnce();
        }
        }

        case 1:
        {
        for(int i = 0 ;i < 5 ; i++)
        {
            rate_1.sleep();
            ros::spinOnce();
        }
        }
        
        case 4:
        {
        for(int i = 0 ;i < 7 ; i++)
        {
            rate_1.sleep();
            ros::spinOnce();
        }
        }

        case 0:
        {
        for(int i = 0 ;i < 9 ; i++)
        {
            rate_1.sleep();
            ros::spinOnce();
        }
        }
        
        case 5:
        {
        for(int i = 0 ;i < 11 ; i++)
        {
            rate_1.sleep();
            ros::spinOnce();
        }
        }

    if(nh.getParam("/swarm_assemble/out_of_room_pos",rect_pose_all)){
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
            std::cout << "set way point out of room success!" << std::endl;
        }
        else{
            ROS_ERROR("wrong uav unmber!");
        }
    } else ROS_ERROR("param set error before rect");

    // wait for cross room
    while(ros::ok()){
        float distance = distance_3f(target_pose,drone_state) ;
        std::cout << " --- distance to way point  : " << distance << std::endl;
        if ( distance_3f(target_pose,drone_state) < 0.2 ) break;
        rate_1.sleep();
        ros::spinOnce();
    }
    
    drone_state_sub.shutdown(); // shutdown subscriber

    

}
// 一字框穿越
void impossible_mission(ros::NodeHandle& nh,ros::Publisher& goal_pose_pub){
    XmlRpc::XmlRpcValue one_pose_all;
    XmlRpc::XmlRpcValue one_pose;
    ros::Rate rate_1(1.0);
        // subscribe drone state
    ros::Subscriber drone_state_sub = nh.subscribe("/prometheus/drone_state",10,drone_state_cb);


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

            while (ros::ok())
            {
                float distance = distance_3f(target_pose,drone_state) ;
                std::cout << " --- distance to way point  : " << distance << std::endl;
                if ( distance_3f(target_pose,drone_state) < 0.3 ) break;
                rate_1.sleep();
                ros::spinOnce();
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
    }
    case 3:
    {
        ros::Subscriber quad_other_room_arrange_sub = nh.subscribe("/able_to_take_off",10,quad_other_room_arrange_sub_cb);
        double time_out = .0;
    
        while (ros::ok() && (time_out < 30) && able_to_move_in_room = false)
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

            while (ros::ok())
            {
                float distance = distance_3f(target_pose,drone_state) ;
                std::cout << " --- distance to way point  : " << distance << std::endl;
                if ( distance_3f(target_pose,drone_state) < 0.3 ) break;
                rate_1.sleep();
                ros::spinOnce();
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
    }
    case 1:
    {
        ros::Subscriber quad_other_room_arrange_sub = nh.subscribe("/able_to_take_off",10,quad_other_room_arrange_sub_cb);
        double time_out = .0;
    
        while (ros::ok() && (time_out < 30*2) && able_to_move_in_room = false)
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

            while (ros::ok())
            {
                float distance = distance_3f(target_pose,drone_state) ;
                std::cout << " --- distance to way point  : " << distance << std::endl;
                if ( distance_3f(target_pose,drone_state) < 0.3 ) break;
                rate_1.sleep();
                ros::spinOnce();
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
    }
    case 4:
    {
        ros::Subscriber quad_other_room_arrange_sub = nh.subscribe("/able_to_take_off",10,quad_other_room_arrange_sub_cb);
        double time_out = .0;
    
        while (ros::ok() && (time_out < 30*3) && able_to_move_in_room = false)
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

            while (ros::ok())
            {
                float distance = distance_3f(target_pose,drone_state) ;
                std::cout << " --- distance to way point  : " << distance << std::endl;
                if ( distance_3f(target_pose,drone_state) < 0.3 ) break;
                rate_1.sleep();
                ros::spinOnce();
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
    }
    case 0:
    {
        ros::Subscriber quad_other_room_arrange_sub = nh.subscribe("/able_to_take_off",10,quad_other_room_arrange_sub_cb);
        double time_out = .0;
    
        while (ros::ok() && (time_out < 30*4) && able_to_move_in_room = false)
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

            while (ros::ok())
            {
                float distance = distance_3f(target_pose,drone_state) ;
                std::cout << " --- distance to way point  : " << distance << std::endl;
                if ( distance_3f(target_pose,drone_state) < 0.3 ) break;
                rate_1.sleep();
                ros::spinOnce();
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
    }
    case 5:
    {
        ros::Subscriber quad_other_room_arrange_sub = nh.subscribe("/able_to_take_off",10,quad_other_room_arrange_sub_cb);
        double time_out = .0;
    
        while (ros::ok() && (time_out < 30*5) && able_to_move_in_room = false)
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

            while (ros::ok())
            {
                float distance = distance_3f(target_pose,drone_state) ;
                std::cout << " --- distance to way point  : " << distance << std::endl;
                if ( distance_3f(target_pose,drone_state) < 0.3 ) break;
                rate_1.sleep();
                ros::spinOnce();
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
    }
    }
 

}
void mission_state_cb(const iusc_referee::MissionState& tmp)
{
    mission_state = tmp.arm_command;
    uav_number = tmp.uav_id;
    // set point pos
}

void quad_other_zone_sub_cb(const iusc_referee::QuadPoseOther &tmp){
    if( other_quad_pose_zone.count(tmp.uav_id_other)) return;
    // see crash as reaching target
    other_quad_pose_zone[tmp.uav_id_other] = bool(tmp.crash) || (tmp.finished_stage == zone); 
}

void quad_other_before_rect_sub_cb(const iusc_swarm_strategy::iusc_swarm &tmp){
    if( other_quad_pose_before_rect.count(tmp.uav_id_other)) return;
    other_quad_pose_before_rect[tmp.uav_id_other] = bool(tmp.before_rect_success) ;
}

void quad_other_cross_rect_sub_cb(const iusc_swarm_strategy::iusc_swarm &tmp){
    if( other_quad_pose_cross_rect.count(tmp.uav_id_other)) return;
    other_quad_pose_cross_rect[tmp.uav_id_other] = bool(tmp.cross_rect_success) ;
}

void quad_other_room_arrange_sub_cb(const iusc_swarm_strategy::iusc_swarm &tmp){
    switch(target_number)
    {
    case 3:{
        if (tmp.target_number_other = 2 && tmp.room_arrange_success = true){
            able_to_move_in_room = true;    
        }
    }
    case 1:{
        if (tmp.target_number_other = 3 && tmp.room_arrange_success = true){
            able_to_move_in_room = true;    
        }
    }
    case 4:{
        if (tmp.target_number_other = 1 && tmp.room_arrange_success = true){
            able_to_move_in_room = true;    
        }
    }
    case 0:{
        if (tmp.target_number_other = 4 && tmp.room_arrange_success = true){
            able_to_move_in_room = true;    
        }
    }
    case 5:{
        if (tmp.target_number_other = 0 && tmp.room_arrange_success = true){
            able_to_move_in_room = true;    
        }
    }
    }
}

void quad_other_room_arrived_sub_cb(const iusc_swarm_strategy::iusc_swarm &tmp){
    if( tmp.target_number_other = 5 ) 
    able_to_leave_room = tmp.room_arrange_success ;
}

void drone_state_cb(const prometheus_msgs::DroneState &tmp){
    drone_state[0] = tmp.position[0] ;
    drone_state[1] = tmp.position[1] ;
    drone_state[2] = tmp.position[2] ;
}

void target_number_cb(const int &tmp){
    if( tmp>=0 && tmp<=5 ) target_number = tmp ;
}

/*
    起飞的时间
*/
void when_to_set_up(ros::NodeHandle &nh){
    ros::Subscriber able_to_take_off_recv = nh.subscribe("/able_to_take_off_recv",10,able_to_take_off_recv_cb);
    ros::Rate rate(1);
    switch (uav_number)
    {
    case 0:{
        /* code */
        ros::Publisher able_to_take_off_pub = nh.advertise<iusc_swarm_strategy::iusc_swarm>("/able_to_take_off",10,true);

        // uav  4  and uav 5 takeoff first
        iusc_swarm_strategy::iusc_swarm take_off_1 , take_off_2 , take_off_3 , take_off_4 , take_off_5;
        take_off_1.uav_id = 4;
        take_off_1.able_to_take_off = true;
        take_off_2.uav_id = 5;
        take_off_2.able_to_take_off = true;
        take_off_3.uav_id = 3;
        take_off_3.able_to_take_off = true;
        take_off_4.uav_id = 2;
        take_off_4.able_to_take_off = true;
        take_off_5.uav_id = 1;
        take_off_5.able_to_take_off = true; 
        int pub_count = 0;
        while(ros::ok()) {
            if(pub_count > 3 ) break;
            able_to_take_off_pub.publish(take_off_1);
            rate.sleep();
            able_to_take_off_pub.publish(take_off_2);
            rate.sleep();
            able_to_take_off_pub.publish(take_off_3);
            rate.sleep();
            able_to_take_off_pub.publish(take_off_4);
            rate.sleep();
            able_to_take_off_pub.publish(take_off_5);
            pub_count ++;
            rate.sleep();
            ros::spinOnce();
        }


        std::cout << "-----all uav take off set! --------" << std::endl;
 
        break;
    }
    default:{
          /* code */
        while (ros::ok())
        {
            if(alle_to_take_off_sing) break;
            std::cout <<" --- wait for take off command -----" << std::endl;
            rate.sleep();
            ros::spinOnce();
        }
        std::cout << "--- receive take off command!! ---" << std::endl;
        break;
    }
    }

}
/* void when_to_cross_one(ros::NodeHandle &nh){
   // ros::Subscriber able_to_take_off_recv = nh.subscribe("/able_to_take_off_recv",10,able_to_take_off_recv_cb);
    ros::Rate rate(1);
    int time_out = 25 , time_count = 0;
    switch (uav_number)
    {
        case 1:{

            while (ros::ok() && time_count < time_out )
            {
                std::cout << " -- wait for impossiable mission : " << time_count << std::endl;
                rate.sleep();
                time_count ++;
                ros::spinOnce();
            }
            break;
        }
        case 3:{
            
            while (ros::ok() && time_count < time_out )
            {
                std::cout << " -- wait for impossiable mission : " << time_count << std::endl;
                rate.sleep();
                time_count ++;
                ros::spinOnce();
            }
	    break;

        }
        case 5:{

            while (ros::ok() && time_count < time_out )
            {
                std::cout << " -- wait for impossiable mission : " << time_count << std::endl;
                rate.sleep();
                time_count ++;
                ros::spinOnce();
            }
	    break;
        }
	default: break;
    }
}*/

void able_to_take_off_recv_cb(const iusc_swarm_strategy::iusc_swarm &tmp){
    // uav 0 charge other uav's takeoff , until all of the uav_takeofff

    switch (uav_number)
    {
    case 0:
        /* code */
        // collect information
        {
            if( tmp.uav_id == 0 ) break;
            if( tmp.cross_rect_success ) able_to_take_off_recv_map[tmp.uav_id] = tmp.cross_rect_success;
        break;
        }
    
    default:{
        // receive takeoff command
        if(uav_number == tmp.uav_id) alle_to_take_off_sing = tmp.able_to_take_off;
        break;
    }
    }
}
