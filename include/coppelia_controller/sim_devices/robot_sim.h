#include <array>
#include <mutex>
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <string>
#include <vector>
#include <unordered_map>

/*!
    Robot class for interacting with CoppeliaSim
*/
class RobotSim
{
public:
    const static int NUM_JOINTS = 6;

    struct sim_object {
        std::string name;
        int handle;
    };

    RobotSim(
        std::vector<std::string> joints,
        std::string ik_target,
        std::string ik_tip,
        int client_id,
        std::mutex &mtx
      );

    /*!
      Set the IK target to an arbitrary 3D position (joint modes must be ik)
      @param coords The coordinates of the desired/target 3D position
    */
    void set_ik_pos(std::array<float, 3> coords);

    /*!
       Wait until the robot reaches the threshold of the desired/target 3D position
    */
    bool wait_for_target_pos(std::array<float, 3> target_coords);

    /*!
      Set the target angles for the robot (joint modes must be force)
      @param angles The array of desired/target angles
    */
    void set_force_angles(std::array<float, NUM_JOINTS> angles);

    /*!
       Wait until the robot reaches the threshold of the target angles
       @param angles The array of desired/target angles
    */
    bool wait_for_target_angles(std::array<float, NUM_JOINTS> target_angles);

    /*!
        Reset the IK targets to the initial position
    */
    void reset_ik_target_to_initial();

    /*!
        Get/Set the current state (joint angles) of the interaction
        @param joints Passed by reference; Sets the incomming array to the current joint angles
    */
    void get_state(std::array<float,6>& in_joints);


    /*!
        Get the position of the end effector
    */
    std::array<float, 3> get_ee_pos();

    /*!
        Get handle from simulator by name
        @param name Name of the handles
        @returns handle ID
    */
    int get_handle(std::string name);

    /*!
        Returns the difference between two positions.
        @param target Target position for end effector
        @param current Current position of end effector
    */
    float get_position_difference(std::array<float, 3>& target, std::array<float, 3>& current);

    /*!
        Returns the difference between two joint angles
        @param target Target angles for the robot
        @param current Current angles of the robot
    */
    float get_joint_position_difference(float target, float current);

    /*!
        Control the robot per the command received from CoppeliaController
        @param message The message that Intprim inference generates for the new position
    */
    void control_robot(const sensor_msgs::JointState::ConstPtr& message);

    std::array<sim_object, NUM_JOINTS>    m_joints;
    bool                                  m_kill_interaction;
    std::array<float, NUM_JOINTS>         m_initial_angles;
    std::mutex&                           m_sim_mutex;

private:
    std::unordered_map<std::string, int>  m_name_handle_map;
    std::array<float,NUM_JOINTS>          m_current_state;
    sim_object                            m_ik_target;
    sim_object                            m_ik_tip;
    std::array<float, 3>                  m_ik_tip_initial_pos;
    int                                   m_client_id;
};
