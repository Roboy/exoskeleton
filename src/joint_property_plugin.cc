#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>

#include <gazebo/gazebo.hh>

#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>

#include <gazebo/physics/physics.hh>
#include <gazebo/physics/World.hh>

#include <gazebo/physics/opensim/OpensimPhysics.hh>
#include <gazebo/physics/opensim/OpensimPhysicsPrivate.hh>
#include <gazebo/physics/opensim/OpensimModel.hh>
#include <gazebo/physics/opensim/OpensimMuscle.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>

#include <ros/time.h>

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <SimTKcommon.h>
#include <thread>
#include <vector>
#include <string>
#include <stdio.h>
#include <mutex>
#include <sstream>
#include <math.h>

namespace gazebo {
    class JointPropertyPlugin : public ModelPlugin {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
            try {
                // Store the pointer to the model
                this->g_model = boost::dynamic_pointer_cast<physics::OpensimModel>(_parent);

                // Listen to the update event. This event is broadcast every
                // simulation iteration.
                this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                        boost::bind(&JointPropertyPlugin::OnUpdate, this, _1));
            } catch (const std::exception &e) {
                ROS_ERROR("Caught an while model cast and event connection : %s", e.what());
                throw e;
            }

            // ---- Joint and ros related stuff ----
            auto joints = this->g_model->GetJoints();

            for (auto &joint : joints) {

                std::stringstream ss;
                ss << g_model->GetName() << "/" << joint->GetName() << "/" << "angles";
                angle_pub.push_back(rosNode->advertise<std_msgs::Float64MultiArray>(ss.str(), 1000));
                ROS_INFO("Added new topic %s for joint %s", ss.str().c_str(), joint->GetName().c_str());

                ss.str("");
                ss << g_model->GetName() << "/" << joint->GetName() << "/" << "accelerations";
                accel_pub.push_back(rosNode->advertise<std_msgs::Float64MultiArray>(ss.str(), 1000));
                ROS_INFO("Added new topic %s for joint %s", ss.str().c_str(), joint->GetName().c_str());

                ss.str("");
                ss << g_model->GetName() << "/" << joint->GetName() << "/" << "velocities";
                vel_pub.push_back(rosNode->advertise<std_msgs::Float64MultiArray>(ss.str(), 1000));
                ROS_INFO("Added new topic %s for joint %s", ss.str().c_str(), joint->GetName().c_str());
            }
        }

        // Called by the world update start event
        void OnUpdate(const common::UpdateInfo &_info) {
            auto joints = g_model->GetJoints();
            for (int i = 0; i < joints.size(); i++) {
                // create the data vector
                std::vector<double> angle = {joints.at(i)->GetAngle(0).Degree(),
                                             joints.at(i)->GetAngle(1).Degree(),
                                             joints.at(i)->GetAngle(2).Degree()};

                std_msgs::Float64MultiArray msg;

                // set up dimensions
                msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
                msg.layout.dim[0].size = angle.size();
                msg.layout.dim[0].stride = 1;
                msg.layout.dim[0].label = "angle";

                // copy in the data
                msg.data.clear();
                msg.data.insert(msg.data.end(), angle.begin(), angle.end());
                angle_pub.at(i).publish(msg);

                // create the data vector
                std::vector<double> accel = {joints.at(i)->GetAcceleration(0),
                                             joints.at(i)->GetAcceleration(1),
                                             joints.at(i)->GetAcceleration(2)};

                // set up dimensions
                msg.layout.dim[0].size = accel.size();
                msg.layout.dim[0].stride = 1;
                msg.layout.dim[0].label = "acceleration";

                // copy in the data
                msg.data.clear();
                msg.data.insert(msg.data.end(), accel.begin(), accel.end());
                accel_pub.at(i).publish(msg);

                // create the data vector
                std::vector<double> vel = {joints.at(i)->GetVelocity(0),
                                           joints.at(i)->GetVelocity(1),
                                           joints.at(i)->GetVelocity(2)};

                // set up dimensions
                msg.layout.dim[0].size = vel.size();
                msg.layout.dim[0].stride = 1;
                msg.layout.dim[0].label = "velocity";

                // copy in the data
                msg.data.clear();
                msg.data.insert(msg.data.end(), vel.begin(), vel.end());
                vel_pub.at(i).publish(msg);
            }
        }


        JointPropertyPlugin() {
            if (!ros::isInitialized()) {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "MetabolicCostAnalysis",
                          ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
            }

            rosNode = std::make_unique<ros::NodeHandle>("joint_property_plugin");
        }

        ~JointPropertyPlugin() {
            rosNode->shutdown();
        }

        void Init() {
            gazebo::ModelPlugin::Init();
            if (!g_model || !engine) return;
        }

    private:
        // Pointer to the model
        physics::OpensimModelPtr g_model;
        physics::OpensimPhysicsPtr engine;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;

        std::unique_ptr<ros::NodeHandle> rosNode;

        std::vector<ros::Publisher> angle_pub;
        std::vector<ros::Publisher> accel_pub;
        std::vector<ros::Publisher> vel_pub;
    };

// Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(JointPropertyPlugin)

}
