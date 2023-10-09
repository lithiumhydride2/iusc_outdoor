#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "iusc_referee/MissionState.h"
#include "iusc_referee/QuadPoseOther.h"
#include "iusc_referee/StaticTargetOther.h"
#include <unistd.h>
#include <std_msgs/UInt8.h>
#include <unordered_map>
#include <array>

// mission state
// lowest height: 0.5m
// collision avoidance: how to realize? 
// crash detect: HtR?
int mission_state = 0;
enum {TOARM = 1, TOLAND = 2, MAYDAY = 3};
enum {NONE = 0, FOREST = 1, STATIC_RECTANGLE = 2, ROTATING_RECTANGE = 3, CIRCLE = 4, SEARCH = 5};
//  others_pose
std::unordered_map<int,bool> other_quad_pose;

void mission_state_cb(const iusc_referee::MissionState& tmp);
void quad_other_sub_cb(const iusc_referee::QuadPoseOther &tmp);
void arm_option();

int main(int argc,char **argv){
    ros::init(argc,argv,"swarm_search");
    ros::NodeHandle nh("~");
    ros::Subscriber mission_state_sub = nh.subscribe("/MissionState", 10, mission_state_cb);
    ros::Rate rate(20);
    ros::Rate rate_1(1);
    
    // wait for armcommand 
    nh.setParam("/finished_stage",NONE);
    while (ros::ok() && !(mission_state == TOARM))
    {
        std::cout << "----------wait for arm command-----------" << std::endl;
        rate.sleep();       
        ros::spinOnce();
    }
    // receive arm command
    std::cout << "----------- receive arm command !!! --------" << std::endl;
    // shutdown this subscriber
    mission_state_sub.shutdown();

    //ready to cross forest
    std::cout << "--------- ready to cross the forest ---------" << std::endl;
    nh.setParam("/ready_to_cross_forest" , true);
    while (ros::ok() && !(nh.param("/finish_cross_forest" , false)))
    {
        std::cout << " -- crossing forest --" << std::endl;
        rate_1.sleep();
    }
    std::cout << " ----- crossed forest & standby in front of rect------ " << std::endl;
    nh.setParam("/finished_stage",FOREST);

    //  receive from other
    ros::Subscriber quad_other_sub = nh.subscribe("/QuadPoseOther",10,quad_other_sub_cb);
    double time_out = 0;
    while (ros::ok() && (time_out < 30))
    {
        if(other_quad_pose.size() == 5){
            int count = 0;
            for(auto tmp: other_quad_pose)
                if(tmp.second) count ++;
            
            // reach!!!!
            if(count == 5)
                break;
        }
        rate_1.sleep();
        time_out ++;
        ros::spinOnce();
    }
    std::cout << " -------- all reach rect , use time: "<< time_out << "-------------- " << std::endl;
    quad_other_sub.shutdown();
    
}

void mission_state_cb(const iusc_referee::MissionState& tmp)
{
    mission_state = tmp.arm_command;
    // after arm, set target pos
    
}

void quad_other_sub_cb(const iusc_referee::QuadPoseOther &tmp){
    if( other_quad_pose.count(tmp.uav_id_other)) return;
    // see crash as reaching target
    other_quad_pose[tmp.uav_id_other] = bool(tmp.crash) || (tmp.finished_stage == FOREST); 
}

void arm_option(){
    // do something 
}