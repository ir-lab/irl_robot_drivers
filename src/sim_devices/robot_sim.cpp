#include "sim_devices/robot_sim.h"
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <random>

extern "C" {
    #include "extApi.h"
}

const int INVALID_HANDLE = -1;
const float POSITION_THRESHOLD = 0.01;
const float JOINT_THRESHOLD = 0.01;

RobotSim::RobotSim(
    std::vector<std::string> joints,
    std::string ik_target,
    std::string ik_tip,
    int client_id,
    std::mutex &mtx) :

    m_joints(),
    m_name_handle_map(),
    m_ik_target(),
    m_ik_tip(),
    m_initial_angles(),
    m_ik_tip_initial_pos(),
    m_kill_interaction(false),
    m_client_id(client_id),
    m_current_state(),
    m_sim_mutex(mtx)
{
    // Set the IK tip object
    m_ik_tip.name = ik_tip;
    m_ik_tip.handle = get_handle(ik_tip);

    // Set the IK target object
    m_ik_target.name = ik_target;
    m_ik_target.handle = get_handle(ik_target);

    // Set the joints object
    for(int i = 0; i < NUM_JOINTS; i++)
    {
        m_joints[i].name = joints[i];
        m_joints[i].handle = get_handle(joints[i]);
    }

    // Set the name_handle_map
    for(int i = 0; i < NUM_JOINTS; i++)
    {
        m_name_handle_map[m_joints[i].name] = m_joints[i].handle;
    }

    // Capture the initial joint angles
    for(int i = 0; i < NUM_JOINTS; i++)
    {
        std::lock_guard<std::mutex> lock(m_sim_mutex);
        int status = simxGetJointPosition(m_client_id, m_joints[i].handle, &m_initial_angles[i], simx_opmode_blocking);
        if(status != simx_return_ok)
        {
            std::cout << "RobotSim(): Warning! Joint initial position for " << m_joints[i].name << " not obtained. Status code: " << status << std::endl;
        }
    }

    // Capture the initial IK tip position
    if(m_ik_tip.handle != INVALID_HANDLE)
    {
        std::lock_guard<std::mutex> lock(m_sim_mutex);
        int status = simxGetObjectPosition(m_client_id, m_ik_tip.handle, -1, m_ik_tip_initial_pos.data(), simx_opmode_blocking);
        if(status != simx_return_ok)
        {
            std::cout << "RobotSim(): Warning! IK tip initial position for " << m_ik_tip.name << " not obtained. Status code: " << status << std::endl;
        }
    }
}

void RobotSim::set_ik_pos(std::array<float, 3> coords)
{
    std::lock_guard<std::mutex> lock(m_sim_mutex);
    int status = simxSetObjectPosition(m_client_id, m_ik_target.handle, -1, coords.data(), simx_opmode_oneshot_wait);
    if(status != simx_return_ok)
    {
        std::cout << "set_ik_pos(): Warning! IK tip for " << m_ik_target.name << " was unable to be set. Status code: " << status << std::endl;
    }
}

bool RobotSim::wait_for_target_pos(std::array<float, 3> target_coords)
{
    bool target_reached = true;
    std::array<float, 3> current_coords = {0.0, 0.0, 0.0};
    do
    {
        target_reached = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::lock_guard<std::mutex> lock(m_sim_mutex);
        int status = simxGetObjectPosition(m_client_id, m_ik_tip.handle, -1, current_coords.data(), simx_opmode_streaming);
        if(status != simx_return_ok || get_position_difference(target_coords, current_coords) > POSITION_THRESHOLD)
        {
            target_reached = false;
        }
    } while(ros::ok() && !target_reached && !m_kill_interaction);
    return true;
}

void RobotSim::set_force_angles(std::array<float, NUM_JOINTS> angles)
{
    for(int i = 0; i < NUM_JOINTS; i++)
    {
        std::lock_guard<std::mutex> lock(m_sim_mutex);
        int status = simxSetJointTargetPosition(m_client_id, m_joints[i].handle, angles[i], simx_opmode_oneshot_wait);
        if(status != simx_return_ok)
        {
            std::cout << "set_force_angles(): Warning Joint Target Position for " << m_joints[i].name << " was unable to be set!" << std::endl;
        }
    }
}

bool RobotSim::wait_for_target_angles(std::array<float, NUM_JOINTS> target_angles)
{
    bool target_reached = true;
    do
    {
        target_reached = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        for(int i = 0; i < NUM_JOINTS; i++)
        {
            float value = 0.0;
            std::lock_guard<std::mutex> lock(m_sim_mutex);
            int status = simxGetJointPosition(m_client_id, m_joints[i].handle, &value, simx_opmode_streaming);
            if(status != simx_return_ok || get_joint_position_difference(target_angles[i], value) > JOINT_THRESHOLD)
            {
                target_reached = false;
            }
        }
        //std::cout << target_reached << m_kill_interaction << std::
    } while(ros::ok() && !target_reached && !m_kill_interaction);
    return true;
}

void RobotSim::reset_ik_target_to_initial()
{
    std::lock_guard<std::mutex> lock(m_sim_mutex);
    int status = simxSetObjectPosition(m_client_id, m_ik_target.handle, -1, m_ik_tip_initial_pos.data(), simx_opmode_oneshot_wait);
    if(status != simx_return_ok)
    {
        std::cout << "reset_ik_target_to_initial(): Failed to Set Object Position for " << m_ik_target.name << std::endl;
    }
}

void RobotSim::get_state(std::array<float, 6>& in_joints)
{
    float joint_angle = 0.0;
    for(int i = 0; i < NUM_JOINTS; i++)
    {
        std::lock_guard<std::mutex> lock(m_sim_mutex);
        int status = simxGetJointPosition(m_client_id, m_joints[i].handle, &joint_angle, simx_opmode_streaming);
        if(status == simx_return_ok)
        {
            in_joints[i] = joint_angle;
        }
        // Do not catch an error when we first query get_state (returns novalue on first call)
        else if(status != simx_return_novalue_flag)
        {
            std::cout << "get_state(): Failed to get Joint Position for " << m_joints[i].name << std::endl;
        }
    }
}

std::array<float, 3> RobotSim::get_ee_pos()
{
    std::array<float, 3> coords = {0.0, 0.0, 0.0};
    std::lock_guard<std::mutex> lock(m_sim_mutex);
    int status = simxGetObjectPosition(m_client_id, m_ik_tip.handle, -1, coords.data(), simx_opmode_blocking);
    if(status != simx_return_ok)
    {
        std::cout << "get_ee_pos(): Unbale to get position for handle " << m_ik_tip.name << std::endl;
    }
    return coords;
}

int RobotSim::get_handle(std::string name)
{
    int sim_handle = INVALID_HANDLE;
    std::lock_guard<std::mutex> lock(m_sim_mutex);
    simxInt status = simxGetObjectHandle(m_client_id, (simxChar*)name.c_str(), &sim_handle, simx_opmode_blocking);
    if(status != simx_return_ok)
    {
        std::cout << "get_handle(): Failed to retrieve virtual object with name " << name << std::endl;
    }

    return sim_handle;
}

float RobotSim::get_position_difference(std::array<float,3>& target, std::array<float,3>& current)
{
    return std::hypot(std::hypot(target[0] - current[0], target[1] - current[1]), target[2] - current[2]);
}

float RobotSim::get_joint_position_difference(float target, float current)
{
    return std::abs(target - current);
}

void RobotSim::control_robot(const sensor_msgs::JointState::ConstPtr& message)
{
    for(std::size_t joint_idx = 0; joint_idx < NUM_JOINTS; joint_idx++)
    {
        int handle = m_name_handle_map[message->name[joint_idx]];

        if(handle != INVALID_HANDLE)
        {
            std::lock_guard<std::mutex> lock(m_sim_mutex);
            int status = simxSetJointTargetPosition(m_client_id, m_joints[joint_idx].handle, message->position[joint_idx], simx_opmode_streaming);
            if(status != simx_return_ok)
            {
                std::cout << "control_robot(): Failed to Set Joint Target Position for " << m_joints[joint_idx].name << std::endl;
            }
        }
    }
}
