#include <ros/ros.h>
#include <iostream>
#include <string>
#include <stdexcept>
#include <vector>

#include "coppelia_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coppelia_controller", ros::init_options::AnonymousName);

    // Private node handle
    ros::NodeHandle handle("~");

    int num_robots;
    if(!handle.getParam("num_robots", num_robots))
    {
        throw std::invalid_argument("Missing \"num_robots\" ROS private parameter. Please specify and re-run node.");
    }

    std::string demo_topic_name;
    if(!handle.getParam("demo_status_topic", demo_topic_name))
    {
        throw std::invalid_argument("Missing \"demo_status_topic\" ROS private parameter. Please specify and re-run node.");
    }

    std::string demo_reset_name;
    if(!handle.getParam("demo_reset_topic", demo_reset_name))
    {
        throw std::invalid_argument("Missing \"demo_reset_topic\" ROS private parameter. Please specify and re-run node.");
    }

    int in_frequency;
    if(!handle.getParam("in_frequency", in_frequency))
    {
        throw std::invalid_argument("Missing \"in_frequency\" ROS private parameter. Please specify and re-run node.");
    }

    int test_robot;
    if(!handle.getParam("test_robot", test_robot))
    {
        throw std::invalid_argument("Missing \"test_robot\" ROS private parameter. Please specify and re-run node.");
    }

    std::vector<std::string> out_topic_names;
    std::vector<std::string> in_topic_names;
    std::vector<std::vector<std::string>> joint_lists;
    std::vector<std::string> ee_target_joints;
    std::vector<std::string> ee_joints;
    for(int robot_idx = 1; robot_idx <= num_robots; robot_idx++)
    {
        std::string out_topic_name;
        std::string param_name = "out_topic" + std::to_string(robot_idx);
        if(!handle.getParam(param_name, out_topic_name))
        {
            throw std::invalid_argument("Missing \"" + param_name + "\" ROS private parameter. Please specify and re-run node.");
        }
        out_topic_names.push_back(out_topic_name);

        std::string in_topic_name;
        param_name = "in_topic" + std::to_string(robot_idx);
        if(!handle.getParam(param_name, in_topic_name))
        {
            throw std::invalid_argument("Missing \"" + param_name + "\" ROS private parameter. Please specify and re-run node.");
        }
        in_topic_names.push_back(in_topic_name);

        std::vector<std::string> joint_list;
        param_name = "joint_list" + std::to_string(robot_idx);
        if(!handle.getParam(param_name, joint_list))
        {
            throw std::invalid_argument("Missing \"" + param_name + "\" ROS private parameter. Please specify and re-run node.");
        }
        joint_lists.push_back(joint_list);

        std::string ee_target_joint;
        param_name = "ee_target_joint" + std::to_string(robot_idx);
        if(!handle.getParam(param_name, ee_target_joint))
        {
            throw std::invalid_argument("Missing \"" + param_name + "\" ROS private parameter. Please specify and re-run node.");
        }
        ee_target_joints.push_back(ee_target_joint);

        std::string ee_joint;
        param_name = "ee_joint" + std::to_string(robot_idx);
        if(!handle.getParam(param_name, ee_joint))
        {
            throw std::invalid_argument("Missing \"" + param_name + "\" ROS private parameter. Please specify and re-run node.");
        }
        ee_joints.push_back(ee_joint);
    }

    CoppeliaController controller(handle, out_topic_names, in_topic_names, demo_topic_name, demo_reset_name, in_frequency, joint_lists, ee_target_joints, ee_joints, test_robot);

    controller.run();
}
