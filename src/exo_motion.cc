#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>

#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>

#include <gazebo/physics/physics.hh>
#include <gazebo/physics/World.hh>

#include <gazebo/physics/opensim/OpensimPhysics.hh>
#include <gazebo/physics/opensim/OpensimModel.hh>
#include <gazebo/physics/opensim/OpensimMuscle.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>

#include "OpenSim/Simulation/Model/Model.h"
#include "OpenSim/Simulation/Model/PathActuator.h"
#include "OpenSim/Simulation/Model/Umberger2010MuscleMetabolicsProbe.h"

#include "OpenSim/Analyses/ProbeReporter.h"

#include <ros/time.h>
#include <ros/ros.h>

#include <std_msgs/Float64.h>

#include <thread>
#include <vector>
#include <string>
#include <stdio.h>
#include <mutex>

namespace gazebo {
    class ExoMotion : public ModelPlugin {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
            // Store the pointer to the model
            this->model = boost::dynamic_pointer_cast<physics::OpensimModel>(_parent);

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&ExoMotion::OnUpdate, this, _1));
            updateConnectionEnd = event::Events::ConnectWorldUpdateEnd(boost::bind(&ExoMotion::OnUpdateEnd, this));

            o_model.setName("metabolics_model");

            umbergerTotal = new OpenSim::Umberger2010MuscleMetabolicsProbe(true, true, true, true);
            umbergerTotal->setName("umbergerTotal");
            umbergerTotal->setOperation("integrate");
            umbergerTotal->setInitialConditions(SimTK::Vector(SimTK::Vec1(0)));

            probeReporter = new OpenSim::ProbeReporter(&o_model);



            const auto &muscles = model->GetMuscles();
            for (int k = 0; k < muscles.size(); ++k)
            {
                auto muscle = muscles[k];
                const std::string name = muscle->GetName();

                o_model.addForce(muscle->GetOsimActuator());
                umbergerTotal->addMuscle(muscle->GetOsimActuator()->getName(), 0.5);
                ROS_INFO("Added Umberger2010 probe: total energy liberation (%s)",
                        muscle->GetOsimActuator()->getName().c_str());
            }

            total_pub = rosNode->advertise<std_msgs::Float64>("metabolic_total", 1000);

        }

        // Called by the world update start event
        void OnUpdate(const common::UpdateInfo & /*_info*/) {
            // Apply a small linear velocity to the model.
            //this->model->SetLinearVel(math::Vector3(.03, 0, 0));
        }

        void OnUpdateEnd(/*const common::UpdateInfo & _info*/) {
            // publish some stuff
            double simTime = model->GetWorld()->GetSimTime().Double();
            OpenSim::Array<double> probeData;
            probeData.setSize(1);
            const OpenSim::Storage &probeStorage = probeReporter->getProbeStorage();
            probeStorage.getDataAtTime(simTime, 1, probeData);

            std_msgs::Float64 msg;
            msg.data = probeData[probeStorage.getColumnIndicesForIdentifier("umbergerTotal_TOTAL")[0]-1];

            total_pub.publish(msg);

        }

        ExoMotion() {
            if (!ros::isInitialized()) {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "ForceCompensatingHalterung",
                        ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
            }

            rosNode = std::make_unique<ros::NodeHandle>("exo_motion");
        }

        ~ExoMotion() {
            rosNode->shutdown();
        }

        void Init() {
            gazebo::ModelPlugin::Init();
            if (!model || !engine) return;

            const auto &muscles = model->GetMuscles();

        }




        // Pointer to the model
    private:
        physics::OpensimModelPtr model;
        physics::OpensimPhysicsPtr engine;

        OpenSim::Model o_model;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
        event::ConnectionPtr updateConnectionEnd;

        std::unique_ptr<ros::NodeHandle> rosNode;

        OpenSim::Umberger2010MuscleMetabolicsProbe* umbergerTotal;
        OpenSim::ProbeReporter* probeReporter;

        ros::Publisher total_pub;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ExoMotion)
}
