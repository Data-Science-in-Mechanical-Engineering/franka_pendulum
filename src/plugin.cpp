#include <pinocchio/fwd.hpp>
#include <franka_pole/parameters.h>
#include <franka_pole/franka_model.h>

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <fcntl.h>
#include <string>
#include <iostream>

namespace franka_pole
{
    class Plugin : public gazebo::ModelPlugin
    {
    private:
        //Gazebo
        gazebo::physics::ModelPtr _model;
        gazebo::physics::JointPtr _joints[7];
        gazebo::physics::JointPtr _fingers[2];
        gazebo::physics::JointPtr _pole[2];
        gazebo::event::ConnectionPtr _connection;

        //Parameters
        Model _mod = Model::D1;
        Eigen::Matrix<double, 7, 1> _initial_joint_positions = Eigen::Matrix<double, 7, 1>::Zero();
        Eigen::Matrix<double, 2, 1> _initial_pole_positions = Eigen::Matrix<double, 2, 1>::Zero();
        Eigen::Matrix<double, 2, 1> _initial_pole_velocities = Eigen::Matrix<double, 2, 1>::Zero();

        //Reset flag
        sem_t *_software_reset_semaphore;

    public:
        Plugin();
        virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
        void SetDefault();
        void Update(const gazebo::common::UpdateInfo &info);
        virtual ~Plugin();
    };

    GZ_REGISTER_MODEL_PLUGIN(franka_pole::Plugin)
}

franka_pole::Plugin::Plugin()
{
}

void franka_pole::Plugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    try
    {
        //Read parameters
        ros::NodeHandle node_handle;
        Parameters parameters(node_handle);
        FrankaModel franka_model(node_handle);
        _mod = parameters.model();
        _initial_joint_positions = franka_model.inverse_kinematics(parameters.initial_effector_position(), parameters.initial_effector_orientation(), parameters.initial_joint0_position());
        _initial_pole_positions = parameters.initial_pole_positions();
        _initial_pole_velocities = parameters.initial_pole_velocities();

        //Init semaphore
        _software_reset_semaphore = sem_open("/franka_pole_software_reset", O_CREAT, 0644, 0);
        if (_software_reset_semaphore == SEM_FAILED) throw std::runtime_error("franka_emulator::emulator::Semaphore::Semaphore: sem_open failed");
        int value;
        sem_getvalue(_software_reset_semaphore, &value);
        while (value > 0) { sem_wait(_software_reset_semaphore); value--; }

        //Initializing joints
        _model = model;
        for (size_t i = 0; i < 7; i++) _joints[i] = model->GetJoint(std::string("panda::panda_joint") + std::to_string(i + 1));
        for (size_t i = 0; i < 2; i++) _fingers[i] = model->GetJoint(std::string("panda::panda_finger_joint") + std::to_string(i + 1));
        _pole[0] = model->GetJoint("panda::panda_pole_joint_x");
        try { _pole[1] = model->GetJoint("panda::panda_pole_joint_y"); } catch (...) { _pole[1] = nullptr; }
        
        //Initializing position
        SetDefault();

        //Subscribe to update
        class Subscriber
        {
        private:
            Plugin *_plugin;
        public:
            Subscriber(Plugin *plugin) : _plugin(plugin) {}
            void operator()(const gazebo::common::UpdateInfo &info) { _plugin->Update(info); }
        } subscriber(this);
        _connection = gazebo::event::Events::ConnectWorldUpdateBegin(subscriber);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("Failed to load franka_pole_plugin: " << e.what());
    }
    ROS_INFO_STREAM("Loaded franka_pole_plugin.");
}

void franka_pole::Plugin::SetDefault()
{
    for (size_t i = 0; i < 7; i++)
    {
        _joints[i]->SetPosition(0, _initial_joint_positions(i));
        _joints[i]->SetVelocity(0, 0.0);
    }
    for (size_t i = 0; i < 2; i++)
    {
        _fingers[i]->SetPosition(0, 0.0);
        _fingers[i]->SetVelocity(0, 0.0);
    }
    _pole[0]->SetPosition(0, _initial_pole_positions(0));
    _pole[0]->SetVelocity(0, _initial_pole_velocities(0));
    if (_pole[1] != nullptr)
    {
        _pole[1]->SetPosition(0, _initial_pole_positions(1) + ((_mod == Model::D2) ? (M_PI / 6) : 0));
        _pole[1]->SetVelocity(0, _initial_pole_velocities(1));
    }
}

void franka_pole::Plugin::Update(const gazebo::common::UpdateInfo &info)
{
    int value;
    sem_getvalue(_software_reset_semaphore, &value);
    if (value > 0)
    {
        SetDefault();
        sem_wait(_software_reset_semaphore);
    }
}

franka_pole::Plugin::~Plugin()
{
    ROS_INFO_STREAM("franka_pole::Plugin unloaded");
}