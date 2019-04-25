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
#include "OpenSim/Simulation/Model/Muscle.h"

#include "OpenSim/Simulation/SimbodyEngine/Body.h"
#include "OpenSim/Simulation/SimbodyEngine/PinJoint.h"

#include "OpenSim/Analyses/ProbeReporter.h"

#include <ros/time.h>
#include <ros/ros.h>

#include <std_msgs/Float64.h>

#include <SimTKcommon.h>

#include <thread>
#include <vector>
#include <string>
#include <stdio.h>
#include <mutex>
#include <sstream>

namespace gazebo {
    class ExoMotion : public ModelPlugin {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
            try {
                // Store the pointer to the model
                this->g_model = boost::dynamic_pointer_cast<physics::OpensimModel>(_parent);

                // Listen to the update event. This event is broadcast every
                // simulation iteration.
                this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                        boost::bind(&ExoMotion::OnUpdate, this, _1));
                updateConnectionEnd = event::Events::ConnectWorldUpdateEnd(boost::bind(&ExoMotion::OnUpdateEnd, this));
            } catch (const std::exception &e) {
                ROS_ERROR("Caught an while model cast and event connection : %s", e.what());
                throw e;
            }
            try {
                o_model.setName("metabolics_model");

                // create the Metabolics probe & its reporter and add it to the opensim model
                umbergerTotal = new OpenSim::Umberger2010MuscleMetabolicsProbe(true, true, true, true);
                umbergerTotal->setName("umbergerTotal");
                umbergerTotal->setOperation("integrate");
                umbergerTotal->setInitialConditions(SimTK::Vector(SimTK::Vec1(0)));

                probeReporter = new OpenSim::ProbeReporter(&o_model);
            } catch (const std::exception &e) {
                ROS_ERROR("Caught an exception while createing the probe : %s", e.what());
                throw e;
            }

            // add the links as bodies
            int body_count = 1;
            const auto &links = g_model->GetLinks();

            auto link = links[0];
            std::stringstream ss;
            std::string name = link->GetName();

            ss << name << "_" << body_count++;
            name = ss.str();

            ROS_INFO("reference sdf of %s : %s", name.c_str(), link->GetSDF()->GetName().c_str());

            // Body(const std::string &aName,double aMass,const SimTK::Vec3& aMassCenter,const SimTK::Inertia& aInertia);
            OpenSim::Body *base = new OpenSim::Body(name,
                                                    link->GetInertial()->GetMass(),
                                                    SimTK::Vec3(0),
                                                    link->GetInertial()->GetMass() *
                                                    SimTK::Inertia::brick(SimTK::Vec3(0.1 / 2)));

            link = links[1];
            name = link->GetName();

            ss.str("");
            ss << name << "_" << body_count++;
            name = ss.str();

            ROS_INFO("reference sdf of %s : %s", name.c_str(), link->GetSDF()->GetName().c_str());

            // Body(const std::string &aName,double aMass,const SimTK::Vec3& aMassCenter,const SimTK::Inertia& aInertia);
            OpenSim::Body *r_humerus = new OpenSim::Body(name,
                                                         link->GetInertial()->GetMass(),
                                                         SimTK::Vec3(0),
                                                         link->GetInertial()->GetMass() *
                                                         SimTK::Inertia::brick(SimTK::Vec3(0.1 / 2)));

            link = links[2];
            name = link->GetName();

            ss.str("");
            ss << name << "_" << body_count++;
            name = ss.str();

            ROS_INFO("reference sdf of %s : %s", name.c_str(), link->GetSDF()->GetName().c_str());

            // Body(const std::string &aName,double aMass,const SimTK::Vec3& aMassCenter,const SimTK::Inertia& aInertia);
            OpenSim::Body *r_ulna_radius_hand = new OpenSim::Body(name,
                                                                  link->GetInertial()->GetMass(),
                                                                  SimTK::Vec3(0),
                                                                  link->GetInertial()->GetMass() *
                                                                  SimTK::Inertia::brick(SimTK::Vec3(0.1 / 2)));


            // add joints before you add the bodys to the model

            const auto &joints = g_model->GetJoints();

            auto joint = joints[0];
            ROS_INFO("Joint %s", joint->GetName().c_str());

            OpenSim::Body *parent;
            if (joint->GetName() == "offset") {
                parent = &o_model.getGroundBody();
            } else if (joint->GetParent()->GetName() == "base") {
                parent = base;
            } else if (joint->GetParent()->GetName() == "r_humerus") {
                parent = r_humerus;
            } else if (joint->GetParent()->GetName() == "r_ulna_radius_hand") {
                parent = r_ulna_radius_hand;
            } else {
                ROS_ERROR("Couldn't find a parent for %s", joint->GetParent()->GetName().c_str());
                exit(1);
            }
            ROS_INFO("  - parent: %s", parent->getName().c_str());


            OpenSim::Body *child;

            if (joint->GetChild()->GetName() == "base") {
                child = base;
            } else if (joint->GetChild()->GetName() == "r_humerus") {
                child = r_humerus;
            } else if (joint->GetChild()->GetName() == "r_ulna_radius_hand") {
                child = r_ulna_radius_hand;
            } else {
                ROS_ERROR("Couldn't find a parent for %s", joint->GetChild()->GetName().c_str());
                exit(1);
            }
            ROS_INFO("  - child: %s", child->getName().c_str());
            //PinJoint(const std::string &name, OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
            //					OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody,
            //					bool reverse=false);
            OpenSim::PinJoint offset(joint->GetName(),
                                       *parent,
                                       SimTK::Vec3(joint->GetParentWorldPose().pos.x,
                                                   joint->GetParentWorldPose().pos.y,
                                                   joint->GetParentWorldPose().pos.z),
                                       SimTK::Vec3(joint->GetParentWorldPose().rot.GetAsEuler().x,
                                                   joint->GetParentWorldPose().rot.GetAsEuler().y,
                                                   joint->GetParentWorldPose().rot.GetAsEuler().z),
                                       *child,
                                       SimTK::Vec3(joint->GetInitialAnchorPose().pos.x,
                                                   joint->GetInitialAnchorPose().pos.y,
                                                   joint->GetInitialAnchorPose().pos.z),
                                       SimTK::Vec3(joint->GetInitialAnchorPose().rot.GetAsEuler().x,
                                                   joint->GetInitialAnchorPose().rot.GetAsEuler().y,
                                                   joint->GetInitialAnchorPose().rot.GetAsEuler().z)
            );

            ROS_INFO("  -> Connected");

            joint = joints[1];
            ROS_INFO("Joint %s", joint->GetName().c_str());

            if (joint->GetName() == "offset") {
                parent = &o_model.getGroundBody();
            } else if (joint->GetParent()->GetName() == "base") {
                parent = base;
            } else if (joint->GetParent()->GetName() == "r_humerus") {
                parent = r_humerus;
            } else if (joint->GetParent()->GetName() == "r_ulna_radius_hand") {
                parent = r_ulna_radius_hand;
            } else {
                ROS_ERROR("Couldn't find a parent for %s", joint->GetParent()->GetName().c_str());
                exit(1);
            }
            ROS_INFO("  - parent: %s", parent->getName().c_str());

            if (joint->GetChild()->GetName() == "base") {
                child = base;
            } else if (joint->GetChild()->GetName() == "r_humerus") {
                child = r_humerus;
            } else if (joint->GetChild()->GetName() == "r_ulna_radius_hand") {
                child = r_ulna_radius_hand;
            } else {
                ROS_ERROR("Couldn't find a parent for %s", joint->GetChild()->GetName().c_str());
                exit(1);
            }
            ROS_INFO("  - child: %s", child->getName().c_str());
            //PinJoint(const std::string &name, OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
            //					OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody,
            //					bool reverse=false);
            OpenSim::PinJoint r_elbow(joint->GetName(),
                                       *parent,
                                       SimTK::Vec3(joint->GetParentWorldPose().pos.x,
                                                   joint->GetParentWorldPose().pos.y,
                                                   joint->GetParentWorldPose().pos.z),
                                       SimTK::Vec3(joint->GetParentWorldPose().rot.GetAsEuler().x,
                                                   joint->GetParentWorldPose().rot.GetAsEuler().y,
                                                   joint->GetParentWorldPose().rot.GetAsEuler().z),
                                       *child,
                                       SimTK::Vec3(joint->GetInitialAnchorPose().pos.x,
                                                   joint->GetInitialAnchorPose().pos.y,
                                                   joint->GetInitialAnchorPose().pos.z),
                                       SimTK::Vec3(joint->GetInitialAnchorPose().rot.GetAsEuler().x,
                                                   joint->GetInitialAnchorPose().rot.GetAsEuler().y,
                                                   joint->GetInitialAnchorPose().rot.GetAsEuler().z)
            );

            ROS_INFO("  -> Connected");

            joint = joints[2];
            ROS_INFO("Joint %s", joint->GetName().c_str());

            if (joint->GetName() == "offset") {
                parent = &o_model.getGroundBody();
            } else if (joint->GetParent()->GetName() == "base") {
                parent = base;
            } else if (joint->GetParent()->GetName() == "r_humerus") {
                parent = r_humerus;
            } else if (joint->GetParent()->GetName() == "r_ulna_radius_hand") {
                parent = r_ulna_radius_hand;
            } else {
                ROS_ERROR("Couldn't find a parent for %s", joint->GetParent()->GetName().c_str());
                exit(1);
            }
            ROS_INFO("  - parent: %s", parent->getName().c_str());

            if (joint->GetChild()->GetName() == "base") {
                child = base;
            } else if (joint->GetChild()->GetName() == "r_humerus") {
                child = r_humerus;
            } else if (joint->GetChild()->GetName() == "r_ulna_radius_hand") {
                child = r_ulna_radius_hand;
            } else {
                ROS_ERROR("Couldn't find a parent for %s", joint->GetChild()->GetName().c_str());
                exit(1);
            }
            ROS_INFO("  - child: %s", child->getName().c_str());
            //PinJoint(const std::string &name, OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
            //					OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody,
            //					bool reverse=false);
            OpenSim::PinJoint r_shoulder(joint->GetName(),
                                       *parent,
                                       SimTK::Vec3(joint->GetParentWorldPose().pos.x,
                                                   joint->GetParentWorldPose().pos.y,
                                                   joint->GetParentWorldPose().pos.z),
                                       SimTK::Vec3(joint->GetParentWorldPose().rot.GetAsEuler().x,
                                                   joint->GetParentWorldPose().rot.GetAsEuler().y,
                                                   joint->GetParentWorldPose().rot.GetAsEuler().z),
                                       *child,
                                       SimTK::Vec3(joint->GetInitialAnchorPose().pos.x,
                                                   joint->GetInitialAnchorPose().pos.y,
                                                   joint->GetInitialAnchorPose().pos.z),
                                       SimTK::Vec3(joint->GetInitialAnchorPose().rot.GetAsEuler().x,
                                                   joint->GetInitialAnchorPose().rot.GetAsEuler().y,
                                                   joint->GetInitialAnchorPose().rot.GetAsEuler().z)
            );

            ROS_INFO("  -> Connected");

            std::stringstream jointMsg;
            jointMsg << "hasJoint: " << std::endl <<
            "base : " << base->hasJoint() << std::endl <<
            "r_humerus: " << r_humerus->hasJoint() << std::endl <<
            "r_ulna_radius_hand: " << r_ulna_radius_hand->hasJoint() << std::endl;

            ROS_INFO(jointMsg.str().c_str());

            o_model.addBody(base);
            o_model.addBody(r_humerus);
            o_model.addBody(r_ulna_radius_hand);

//            for (int k = 0; k< links.size(); ++k) {
//                auto link = links[k];
//                std::stringstream ss;
//                std::string name = link->GetName();
//
//                ss << name << "_" << body_count++;
//                name = ss.str();
//
//                ROS_INFO("reference sdf of %s : %s", name.c_str(), link->GetSDF()->GetName().c_str());
//
//                // Body(const std::string &aName,double aMass,const SimTK::Vec3& aMassCenter,const SimTK::Inertia& aInertia);
//                OpenSim::Body * body = new OpenSim::Body(name,
//                        link->GetInertial()->GetMass(),
//                        SimTK::Vec3(0),
//                        link->GetInertial()->GetMass() * SimTK::Inertia::brick(SimTK::Vec3(0.1/2)));
//                o_model.addBody(body);
//            }

            // add muscles to the opensim model
            const auto &muscles = g_model->GetMuscles();
            for (
                    int k = 0;
                    k < muscles.

                            size();

                    ++k) {
                auto muscle = muscles[k];
                const std::string name = muscle->GetName();

                try {
                    o_model.addForce(muscle->GetOsimActuator());
                } catch (const std::exception &e) {
                    ROS_ERROR("Caught an exception while adding the muscle to the model : %s", e.what());
                    throw e;
                }

                try {
                    umbergerTotal->addMuscle(muscle->GetOsimActuator()->getName(), 0.5);
                } catch (std::exception &e) {
                    ROS_ERROR("Caught an exception while adding the muscle to the probe: %s", e.what());
                    throw e;
                }
                ROS_INFO("Added Umberger2010 probe: total energy liberation (%s)",
                         muscle->GetOsimActuator()->getName().c_str());
            }


            total_pub = rosNode->advertise<std_msgs::Float64>("metabolic_total", 1000);
            SimTK::State &state = o_model.initSystem();
            for (int i = 0; i < o_model.getMuscles().getSize(); ++i)
                o_model.getMuscles().get(i).setIgnoreActivationDynamics(state, true);
            o_model.getMultibodySystem().realize(state, SimTK::Stage::Dynamics);
            o_model.equilibrateMuscles(state);
        }

        // Called by the world update start event
        void OnUpdate(const common::UpdateInfo & /*_info*/) {
            // Apply a small linear velocity to the model.
            //this->model->SetLinearVel(math::Vector3(.03, 0, 0));
        }

        void OnUpdateEnd(/*const common::UpdateInfo & _info*/) {
            // publish some stuff
//            double simTime = g_model->GetWorld()->GetSimTime().Double();
//            OpenSim::Array<double> probeData;
//            probeData.setSize(1);
//            const OpenSim::Storage &probeStorage = probeReporter->getProbeStorage();
//            probeStorage.getDataAtTime(simTime, 1, probeData);
//
//            std_msgs::Float64 msg;
//            msg.data = probeData[probeStorage.getColumnIndicesForIdentifier("umbergerTotal_TOTAL")[0]-1];
//
//            total_pub.publish(msg);

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

        ~

        ExoMotion() {
            rosNode->shutdown();
        }

        void Init() {
            gazebo::ModelPlugin::Init();
            if (!g_model || !engine) return;

            const auto &muscles = g_model->GetMuscles();

        }




        // Pointer to the model
    private:
        physics::OpensimModelPtr g_model;
        physics::OpensimPhysicsPtr engine;

        OpenSim::Model o_model;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
        event::ConnectionPtr updateConnectionEnd;

        std::unique_ptr<ros::NodeHandle> rosNode;

        OpenSim::Umberger2010MuscleMetabolicsProbe *umbergerTotal;
        OpenSim::ProbeReporter *probeReporter;

        ros::Publisher total_pub;
    };

// Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ExoMotion)

}
