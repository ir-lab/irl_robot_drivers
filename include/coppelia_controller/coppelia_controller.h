#pragma once

#include <array>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <thread>
#include <atomic>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>

#include "sim_devices/robot_sim.h"

/*!
    A simple controller for controlling robots/objects in CoppeliaSim.
*/
class CoppeliaController
{
public:
    /*!
        Initializes the CoppeliaController instance.

        @param handle The ROS node handle.
        @param out_topic_names The topic names (specified in coppelia_controller.launch) to subscribe to for receiving inference commands. Only one robot will be receiving commands during testing, so only one topic is needed; however, multiple control topics are available for the user to use if needed.
        @param in_topic_names The topic names (specified in coppelia_controller.launch) to publish the robots' current CoppeliaSim state on.
        @param demo_topic_name is the name of the topic in the InteractionApplication (intprim_ros_framework package) that is specified by the demo_status_topic parameter. The subscriber to this topic listens for 1,3, and 5 messages. The 1 message indicates that the "train" option was selected from the InteractionApplication CLI. The 3 message indicates that the "test" option was selected from the CLI. The 5 message indicates that a user decided to stop the interaction early and serves as a stop signal.
        @param in_frequency The frequency at which data should be collected from CoppeliaSim.
        @param joint_lists The names of the joints (one list per robot) that correspond to the robot joints displayed in CoppeliaSim.
        @param ee_target_joints The Inverse Kinematics (IK) dummies in CoppeliaSim that the robot end effectors are tied to.
        @param ee_joints The names of the end effectors for each robot in the simulation.
        @param test_robot Specifies which robot is the non-controlled robot (UR5l) as specified in the coppelia_controller.launch file.
    */
    CoppeliaController(
        ros::NodeHandle handle,
        std::vector<std::string> out_topic_names,
        std::vector<std::string> in_topic_names,
        std::string demo_topic_name,
        std::string demo_reset_name,
        int in_frequency,
        std::vector<std::vector<std::string>> joint_lists,
        std::vector<std::string> ee_target_joints,
        std::vector<std::string> ee_joints,
        int test_robot);

    ~CoppeliaController();

    /*!
        Entry point for CoppeliaController.
    */
    void run();

private:
    std::vector<ros::Subscriber>            m_controller_subscribers;
    std::vector<ros::Publisher>             m_controller_publishers;
    ros::Subscriber                         m_demonstration_subscriber;
    ros::Publisher                          m_demonstration_publisher;
    ros::Subscriber                         m_demo_reset_subscriber;
    ros::Publisher                          m_demo_reset_publisher;
    std::vector<std::unique_ptr<RobotSim>>  m_robots;
    std::atomic<bool>                       m_in_progress;
    int                                     m_client_id;
    int                                     m_publisher_frequency;
    int                                     m_test_robot;
    std::mutex                              m_sim_mutex;

    enum joint_mode { force_all, ik_all, force_controlled, ik_noncontrolled };

    void demo_control(int demo_type);
    void control_callback(const sensor_msgs::JointState::ConstPtr& message, std::size_t robot_idx);
    void set_joints(joint_mode mode);
    void demo_callback(const std_msgs::Int32::ConstPtr& message);
    void reset_callback(const std_msgs::Int32::ConstPtr& message);
    void publish_status();
    std::array<float,3> generate_initial_point();
};
