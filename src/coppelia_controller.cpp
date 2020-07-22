#include "coppelia_controller.h"
#include <boost/bind.hpp>
#include <random>
#include <exception>
#include <unordered_map>

extern "C" {
    #include "extApi.h"
}

const int INVALID_HANDLE = -1;
const float ROBOT_TOUCH_DISTANCE = 0.02;

CoppeliaController::CoppeliaController(
    ros::NodeHandle handle,
    std::vector<std::string> out_topic_names,
    std::vector<std::string> in_topic_names,
    std::string demo_topic_name,
    std::string demo_reset_name,
    int in_frequency,
    std::vector<std::vector<std::string>> joint_lists,
    std::vector<std::string> ee_target_joints,
    std::vector<std::string> ee_joints,
    int test_robot) :

    m_robots(),
    m_controller_subscribers(),
    m_controller_publishers(),
    m_demonstration_subscriber(),
    m_demonstration_publisher(),
    m_client_id(INVALID_HANDLE),
    m_publisher_frequency(in_frequency),
    m_test_robot(test_robot - 1),
    m_in_progress(false)
{
    simxFinish(-1);

    m_client_id = simxStart((simxChar*)"127.0.0.1", 19997, true, true, 2000, 5);

    if(m_client_id != INVALID_HANDLE)
    {
        std::cout << "Connected to V-REP remote API server..." << std::endl;
    }
    else
    {
        throw std::runtime_error("Failed to connect to the V-REP remote API server.");
    }

    simxStopSimulation(m_client_id, simx_opmode_oneshot_wait);
    // Sleep for 1 second to give CoppeliaSim time to reset
    std::this_thread::sleep_for(std::chrono::seconds(1));
    simxStartSimulation(m_client_id, simx_opmode_oneshot_wait);

    for(std::size_t robot_idx = 0; robot_idx < joint_lists.size(); robot_idx++)
    {
        std::unique_ptr<RobotSim> ur5 = std::unique_ptr<RobotSim>(
                    new RobotSim(joint_lists[robot_idx],
                                ee_target_joints[robot_idx],
                                ee_joints[robot_idx],
                                m_client_id,
                                m_sim_mutex)
                                );
        m_robots.push_back(std::move(ur5));

        if(robot_idx != m_test_robot)
        {
            m_controller_subscribers.push_back(handle.subscribe<sensor_msgs::JointState>(out_topic_names[robot_idx], 1, boost::bind(&CoppeliaController::control_callback, this, _1, robot_idx)));
        }
        m_controller_publishers.push_back(handle.advertise<sensor_msgs::JointState>(in_topic_names[robot_idx], 1));
    }
    // Setup Publisher and Subscriber callbacks for control and state topics
    m_demonstration_subscriber = handle.subscribe(demo_topic_name, 1, &CoppeliaController::demo_callback, this);
    m_demonstration_publisher = handle.advertise<std_msgs::Int32>(demo_topic_name, 1);

    // Subscribe to reset
    m_demo_reset_subscriber = handle.subscribe(demo_reset_name, 1, &CoppeliaController::reset_callback, this);
    m_demo_reset_publisher = handle.advertise<std_msgs::Int32>(demo_reset_name, 1);
}

CoppeliaController::~CoppeliaController()
{
    simxStopSimulation(m_client_id, simx_opmode_oneshot_wait);
    simxFinish(m_client_id);
    m_client_id = INVALID_HANDLE;
    m_robots.clear();
}

void CoppeliaController::control_callback(const sensor_msgs::JointState::ConstPtr& message, std::size_t robot_idx)
{
    m_robots[robot_idx]->control_robot(message);
}


void CoppeliaController::demo_control(int demo_type)
{
    std::array<float,3> random_initial_center = generate_initial_point();
    std::vector<std::array<float,3>> random_initial_centers;
    std::array<float,3> lead_center = random_initial_center;
    std::array<float,3> control_center = random_initial_center;

    lead_center[0] -= ROBOT_TOUCH_DISTANCE;
    random_initial_centers.push_back(lead_center);

    control_center[0] += ROBOT_TOUCH_DISTANCE;
    random_initial_centers.push_back(control_center);

    // If we are training
    m_in_progress.store(true);
    if(demo_type == 1)
    {

        set_joints(ik_all);
        // Robots perform action 1: Touch random centers
        for(int robot_idx = 0; robot_idx < m_robots.size(); robot_idx++)
        {
            m_robots[robot_idx]->set_ik_pos(random_initial_centers[robot_idx]);
        }

        // Wait to reach random centers
        for(int robot_idx = 0; robot_idx < m_robots.size(); robot_idx++)
        {
            m_robots[robot_idx]->wait_for_target_pos(random_initial_centers[robot_idx]);
        }

        // Send a value of 0 indicating the demonstration is finished.
        std_msgs::Int32 out_message;
        out_message.data = 0;
        m_demonstration_publisher.publish(out_message);
    }
    // If we are testing
    else
    {
        // Set controlled robot to force for responding to inference commands
        set_joints(force_controlled);
        // Set non-controlled robot to ik for reaching its center point
        set_joints(ik_noncontrolled);

        // Non-controlled robot moves to its center point
        m_robots[m_test_robot]->set_ik_pos(random_initial_centers[m_test_robot]);
        m_robots[m_test_robot]->wait_for_target_pos(random_initial_centers[m_test_robot]);

        // Grace period for controlled robot to finish
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Send a value of 0 indicating the demonstration is finished.
        std_msgs::Int32 out_message;
        out_message.data = 0;
        m_demonstration_publisher.publish(out_message);
    }
}


void CoppeliaController::reset_callback(const std_msgs::Int32::ConstPtr& message)
{
    // If we get the signal to reset the robots
    if(message->data == 1)
    {
        for(int robot_idx = 0; robot_idx < m_robots.size(); robot_idx++)
        {
            m_robots[robot_idx]->m_kill_interaction = false;
        }
        // Set the UR5s to force so that we can move them back to original positions
        set_joints(force_all);
        for(int robot_idx = 0; robot_idx < m_robots.size(); robot_idx++)
        {
            m_robots[robot_idx]->set_force_angles(m_robots[robot_idx]->m_initial_angles);
        }

        // wait to reach initial angles
        for(int robot_idx = 0; robot_idx < m_robots.size(); robot_idx++)
        {
            m_robots[robot_idx]->wait_for_target_angles(m_robots[robot_idx]->m_initial_angles);
        }

        // Reset the IK tips...
        for(int robot_idx = 0; robot_idx < m_robots.size(); robot_idx++)
        {
            m_robots[robot_idx]->reset_ik_target_to_initial();
        }
        m_in_progress.store(false);

        // Respond to reset caller signaling that we have reset
        std_msgs::Int32 out_message;
        out_message.data = 2;
        m_demo_reset_publisher.publish(out_message);
    }
}

void CoppeliaController::set_joints(joint_mode mode)
{
    int status;
    int retIntCnt;
    int* retInts;
    int retStrCnt;
    char* retStrs;
    std::lock_guard<std::mutex> lock(m_sim_mutex);
    switch(mode)
    {
        case force_all:
            status = simxCallScriptFunction(m_client_id,"ScriptDummy",1,"setJointsForceAll",0,NULL,0,NULL,0,NULL,0,NULL,&retIntCnt,&retInts,NULL,NULL,&retStrCnt,&retStrs,NULL,NULL,simx_opmode_blocking);
            break;
        case ik_all:
            status = simxCallScriptFunction(m_client_id,"ScriptDummy",1,"setJointsIkAll",0,NULL,0,NULL,0,NULL,0,NULL,&retIntCnt,&retInts,NULL,NULL,&retStrCnt,&retStrs,NULL,NULL,simx_opmode_blocking);
            break;
        case force_controlled:
            status = simxCallScriptFunction(m_client_id,"ScriptDummy",1,"setJointsForceControlled",0,NULL,0,NULL,0,NULL,0,NULL,&retIntCnt,&retInts,NULL,NULL,&retStrCnt,&retStrs,NULL,NULL,simx_opmode_blocking);
            break;
        case ik_noncontrolled:
            status = simxCallScriptFunction(m_client_id,"ScriptDummy",1,"setJointsIkNonControlled",0,NULL,0,NULL,0,NULL,0,NULL,&retIntCnt,&retInts,NULL,NULL,&retStrCnt,&retStrs,NULL,NULL,simx_opmode_blocking);
            break;
    }
}

void CoppeliaController::demo_callback(const std_msgs::Int32::ConstPtr& message)
{
    // A value of 1 indicates that the demonstration is starting
    if((message->data == 1 || message->data == 3) && !m_in_progress.load())
    {
        for(int robot_idx = 0; robot_idx < m_robots.size(); robot_idx++)
        {
            m_robots[robot_idx]->m_kill_interaction = false;
        }
        std::thread demo_thread = std::thread(&CoppeliaController::demo_control, this, message->data);
        demo_thread.detach();
    }
    else if (message->data == 5 && m_in_progress.load())
    {
        for(int robot_idx = 0; robot_idx < m_robots.size(); robot_idx++)
        {
            m_robots[robot_idx]->m_kill_interaction = true;
        }
    }
    else if (message->data != 0)
    {
        std::cout << "invalid message received: " << message->data << std::endl;
    }
}

void CoppeliaController::publish_status()
{
    for(int robot_idx = 0; robot_idx < m_robots.size(); robot_idx++)
    {
        sensor_msgs::JointState out_message;
        std::array<float,6> joint_vals;
        m_robots[robot_idx]->get_state(joint_vals);
        for(int i = 0; i < 6; i++)
        {
            out_message.name.push_back(m_robots[robot_idx]->m_joints[i].name);
            out_message.position.push_back(joint_vals[i]);
        }
        m_controller_publishers[robot_idx].publish(out_message);
    }
}

std::array<float,3> CoppeliaController::generate_initial_point()
{
    std::random_device rand;
    std::mt19937 rand_gen(rand());

    float x_coord = -0.536; // Fix the x coordinate
    std::uniform_real_distribution<float> y_gen(-1.62, -0.9);
    float y_coord = y_gen(rand_gen);
    std::uniform_real_distribution<float> z_gen(0.4, 0.94);
    float z_coord = z_gen(rand_gen);
    return {x_coord, y_coord, z_coord};
}

void CoppeliaController::run()
{
    ros::AsyncSpinner spinner(3);
    spinner.start();

    ros::WallRate rate(m_publisher_frequency);

    while(ros::ok())
    {
        publish_status();

        rate.sleep();
    }

    spinner.stop();
}
