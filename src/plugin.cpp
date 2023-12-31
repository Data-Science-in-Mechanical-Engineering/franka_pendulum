#include <pinocchio/fwd.hpp>
#include <franka_pendulum/parameters.h>
#include <franka_pendulum/parameter_reader.h>
#include <franka_pendulum/franka_model.h>

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <fcntl.h>
#include <string>
#include <iostream>
#include <mutex>

namespace franka_pendulum
{
    class Plugin : public gazebo::ModelPlugin
    {
    private:
        //Gazebo
        gazebo::physics::ModelPtr _model;
        gazebo::physics::JointPtr _joints[7];
        gazebo::physics::JointPtr _fingers[2];
        gazebo::physics::JointPtr _pendulum[2];
        gazebo::event::ConnectionPtr _connection;

        //Components
        std::mutex _mutex;
        Parameters *_parameters = nullptr;
        FrankaModel *_franka_model = nullptr;

        //Reset flag
        sem_t *_software_reset_semaphore;

    public:
        Plugin();
        virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
        void Reset();
        void Update(const gazebo::common::UpdateInfo &info);
        virtual ~Plugin();
    };

    GZ_REGISTER_MODEL_PLUGIN(franka_pendulum::Plugin)
}

franka_pendulum::Plugin::Plugin()
{
}

void franka_pendulum::Plugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    try
    {
        std::lock_guard<std::mutex> guard(_mutex);
        ros::TransportHints().tcpNoDelay();
        
        //Create components
        ros::NodeHandle node_handle;
        ParameterReader parameter_reader(node_handle);
        _parameters = new Parameters(&_mutex, parameter_reader, node_handle, false);
        _franka_model = new FrankaModel(_parameters);
        
        //Init semaphore
        _software_reset_semaphore = sem_open(("/" + _parameters->namespacee + "_" + _parameters->arm_id + "_software_reset").c_str(), O_CREAT, 0644, 0);
        if (_software_reset_semaphore == SEM_FAILED) throw std::runtime_error("franka_pendulum::Plugin::Load(): sem_open failed");
        int value;
        sem_getvalue(_software_reset_semaphore, &value);
        while (value > 0) { sem_wait(_software_reset_semaphore); value--; }

        //Initializing joints
        std::string name_base = _parameters->arm_id + "::" + _parameters->arm_id;
        for (size_t i = 0; i < 7; i++)
        {
            _joints[i] = model->GetJoint(name_base + "_joint" + std::to_string(i + 1));
            if (_joints[i] == nullptr) throw std::runtime_error("franka_pendulum::Plugin::Load(): model->GetJoint() failed");
        }
        for (size_t i = 0; i < 2; i++)
        {
            _fingers[i] = model->GetJoint(name_base + "_finger_joint" + std::to_string(i + 1));
            if (_fingers[i] == nullptr) throw std::runtime_error("franka_pendulum::Plugin::Load(): model->GetJoint() failed");
        }
        if (get_model_freedom(_parameters->model) >= 1)
        {
            _pendulum[0] = model->GetJoint(name_base + "_pendulum_joint_x");
            if (_pendulum[0] == nullptr) throw std::runtime_error("franka_pendulum::Plugin::Load(): model->GetJoint() failed");
        }
        if (get_model_freedom(_parameters->model) == 2)
        {
            _pendulum[1] = model->GetJoint(name_base + "_pendulum_joint_y");
            if (_pendulum[1] == nullptr) throw std::runtime_error("franka_pendulum::Plugin::Load(): model->GetJoint() failed");
        }
        
        //Initializing
        Reset();

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
        ROS_ERROR_STREAM("Failed to load franka_pendulum_plugin: " << e.what());
    }
    ROS_INFO_STREAM("Loaded franka_pendulum_plugin.");
}

void franka_pendulum::Plugin::Reset()
{
    const double hint[] = { 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4 };
    Eigen::Matrix<double, 7, 1> initial_joint_positions = _franka_model->effector_inverse_kinematics(_parameters->initial_effector_position, _parameters->initial_effector_orientation, _parameters->initial_joint_weights, Eigen::Matrix<double, 7, 1>::Map(hint));

    for (size_t i = 0; i < 7; i++)
    {
        _joints[i]->SetPosition(0, initial_joint_positions(i));
        _joints[i]->SetVelocity(0, 0.0);
    }
    for (size_t i = 0; i < 2; i++)
    {
        _fingers[i]->SetPosition(0, 0.0);
        _fingers[i]->SetVelocity(0, 0.0);
    }
    if (_pendulum[0] != nullptr)
    {
        _pendulum[0]->SetPosition(0, _parameters->initial_pendulum_positions(0));
        _pendulum[0]->SetVelocity(0, _parameters->initial_pendulum_velocities(0));
    }
    if (_pendulum[1] != nullptr)
    {
        if (_parameters->model == Model::D2) _pendulum[1]->SetPosition(0, _parameters->initial_pendulum_positions(1) + M_PI/6);
        else _pendulum[1]->SetPosition(0, _parameters->initial_pendulum_positions(1));
        _pendulum[1]->SetVelocity(0, _parameters->initial_pendulum_velocities(1));
    }
}

void franka_pendulum::Plugin::Update(const gazebo::common::UpdateInfo &info)
{
    std::lock_guard<std::mutex> guard(_mutex);
    int value;
    sem_getvalue(_software_reset_semaphore, &value);
    if (value > 0)
    {
        Reset();
        sem_wait(_software_reset_semaphore);
    }
}

franka_pendulum::Plugin::~Plugin()
{
    ROS_INFO_STREAM("franka_pendulum::Plugin unloaded");
}