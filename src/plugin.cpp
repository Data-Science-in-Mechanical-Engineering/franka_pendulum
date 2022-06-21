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
        ROS_WARN_STREAM("Failed to load franka_pole_plugin: " << e.what());
    }
    ROS_INFO_STREAM("Loaded franka_pole_plugin.");
}

void franka_pole::Plugin::SetDefault()
{
    _joints[0]->SetPosition(0, 0.0);
    _joints[1]->SetPosition(0, 0.0);
    _joints[2]->SetPosition(0, 0.0);
    _joints[3]->SetPosition(0, -M_PI / 2);
    _joints[4]->SetPosition(0, 0.0);
    _joints[5]->SetPosition(0, M_PI / 2);
    _joints[6]->SetPosition(0, M_PI / 4);

    _fingers[0]->SetPosition(0, 0.0);
    _fingers[1]->SetPosition(0, 0.0);

    _pole[0]->SetPosition(0, 0.0);
    if (_pole[1] != nullptr) _pole[1]->SetPosition(0, 0.0);
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
    std::cerr << "franka_pole::Plugin unloaded" << std::endl;
}