#include <boost/bind.hpp>

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

#include "OpenSim/Simulation/Control/Controller.h"

#include <OpenSim/Simulation/Manager/Manager.h>

#include "OpenSim/Simulation/Model/Model.h"
#include "OpenSim/Simulation/Model/PathActuator.h"
#include "OpenSim/Simulation/Model/Umberger2010MuscleMetabolicsProbe.h"
#include "OpenSim/Simulation/Model/Muscle.h"

#include "OpenSim/Simulation/SimbodyEngine/Body.h"

#include "OpenSim/Simulation/SimbodyEngine/PinJoint.h"
#include "OpenSim/Analyses/ProbeReporter.h"

#include "OpenSim/Analyses/MuscleAnalysis.h"
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

#include <roboy_simulation_msgs/MetabolicCost.h>

//==============================================================================
//                    CONSTANT-EXCITATION MUSCLE CONTROLLER
//==============================================================================
// Simple controller to maintain all muscle excitations at the same constant
// value. The metabolic probes depend on both excitation and activation.
class ConstantExcitationMuscleController : public OpenSim::Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(ConstantExcitationMuscleController, OpenSim::Controller);
public:
    ConstantExcitationMuscleController(double u) : _u(u) {}

    void computeControls(const SimTK::State &s, SimTK::Vector &controls) const
    OVERRIDE_11 {
        for (int i = 0; i < _model->getMuscles().getSize(); ++i)
            controls[i] = _u;
    }

    void setConstantExcitation(double u) { _u = u; }

private:
    double _u;
};

namespace gazebo {
    class MetabolicPlugin : public ModelPlugin {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
            try {
                // Store the pointer to the model
                this->g_model = boost::dynamic_pointer_cast<physics::OpensimModel>(_parent);

                // Listen to the update event. This event is broadcast every
                // simulation iteration.
                this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                        boost::bind(&MetabolicPlugin::OnUpdate, this, _1));
                updateConnectionEnd = event::Events::ConnectWorldUpdateEnd(
                        boost::bind(&MetabolicPlugin::OnUpdateEnd, this));
                simTime0 = 0.0;
            } catch (const std::exception &e) {
                ROS_ERROR("Caught an while model cast and event connection : %s", e.what());
                throw e;
            }

            try {

                //--------------------------------------------------------------------------
                // Build the arm26 OpenSim Model
                //--------------------------------------------------------------------------
                o_model = &g_model->getPriv()->osimModel;

                ROS_INFO("Loaded model");

                OpenSim::Body &ground = o_model->getGroundBody();


                double desiredActivation = 1.0;

                // Attach muscle controller.
                controller =
                        new ConstantExcitationMuscleController(desiredActivation);
                controller->setActuators(o_model->updActuators());
                //o_model->addController(controller);
                ROS_INFO("Added muscle controller");

                //--------------------------------------------------------------------------
                // Attach Umberger2010 and Bhargava2004 muscle metabolics probes.
                //--------------------------------------------------------------------------
                int probeCounter = 0;       // Number of probes attached.
                int extraColumns = 0;       // Number of columns expected in storage file is
                // probeCounter + extraColumns.
                const int w = 4;

                auto muscles = o_model->getMuscles();
                std::stringstream ss;
                ss << "num muscles: " << muscles.getSize() << std::endl;
                ROS_INFO("%s", ss.str().c_str());

                // Attach Umberger2010 probes to record individual heat rate and mechanical
                // power components at each point in time for muscle1.
                OpenSim::Umberger2010MuscleMetabolicsProbe *umbergerActMaint_rate[muscles.getSize()];
                try {
                    for (int i = 0; i < muscles.getSize(); i++) {
                        umbergerActMaint_rate[i] = new
                                OpenSim::Umberger2010MuscleMetabolicsProbe(true, false, false, false);
                        o_model->addProbe(umbergerActMaint_rate[i]);
                        std::stringstream name;
                        name << "umbergerActMaint_rate_m" << i;
                        umbergerActMaint_rate[i]->setName(name.str());
                        umbergerActMaint_rate[i]->setOperation("value");
                        umbergerActMaint_rate[i]->addMuscle(muscles[i].getName(), 0.5);
                        ss.str("");
                        ss << std::setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
                           << "activation and maintenance heat rate (" << muscles[i].getName() << ")";
                        ROS_INFO("%s", ss.str().c_str());
                    }
                } catch (std::exception &e) {
                    ROS_ERROR("Excpetion while attaching Activation and maintenance heat reate : %s", e.what());
                    throw e;
                }

                OpenSim::Umberger2010MuscleMetabolicsProbe *umbergerShorten_rate[muscles.getSize()];
                try {
                    for (int i = 0; i < muscles.getSize(); i++) {
                        umbergerShorten_rate[i] = new
                                OpenSim::Umberger2010MuscleMetabolicsProbe(false, true, false, false);
                        o_model->addProbe(umbergerShorten_rate[i]);
                        std::stringstream name;
                        name << "umbergerShorten_rate_m" << i;
                        umbergerShorten_rate[i]->setName(name.str());
                        umbergerShorten_rate[i]->setOperation("value");
                        umbergerShorten_rate[i]->addMuscle(muscles[i].getName(), 0.5);
                        ss.str("");
                        ss << std::setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
                           << "shortening and lengthening heat rate (" << muscles[i].getName() << ")";
                        ROS_DEBUG("%s", ss.str().c_str());
                    }
                } catch (std::exception &e) {
                    ROS_ERROR("Excpetion while attaching shortening and lengthening heat rate : %s", e.what());
                    throw e;
                }


                OpenSim::Umberger2010MuscleMetabolicsProbe *umbergerBasal_rate[muscles.getSize()];
                try {
                    for (int i = 0; i < muscles.getSize(); i++) {
                        umbergerBasal_rate[i] = new
                                OpenSim::Umberger2010MuscleMetabolicsProbe(false, false, true, false);
                        o_model->addProbe(umbergerBasal_rate[i]);
                        std::stringstream name;
                        name << "umbergerBasal_rate_m" << i;
                        umbergerBasal_rate[i]->setName(name.str());
                        umbergerBasal_rate[i]->setOperation("value");
                        umbergerBasal_rate[i]->addMuscle(muscles[i].getName(), 0.5);
                        ss.str("");
                        ss << std::setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
                           << "basal heat rate (" << muscles[i].getName() << ")";
                        ROS_DEBUG("%s", ss.str().c_str());
                    }
                } catch (std::exception &e) {
                    ROS_ERROR("Excpetion while attaching basal heat rate : %s", e.what());
                    throw e;
                }

                OpenSim::Umberger2010MuscleMetabolicsProbe *umbergerMechWork_rate[muscles.getSize()];
                try {
                    for (int i = 0; i < muscles.getSize(); i++) {
                        umbergerMechWork_rate[i] = new
                                OpenSim::Umberger2010MuscleMetabolicsProbe(false, false, false, true);
                        o_model->addProbe(umbergerMechWork_rate[i]);
                        std::stringstream name;
                        name << "umbergerMechWork_rate_m" << i;
                        umbergerMechWork_rate[i]->setName(name.str());
                        umbergerMechWork_rate[i]->setOperation("value");
                        umbergerMechWork_rate[i]->addMuscle(muscles[i].getName(), 0.5);
                        ss.str("");
                        ss << std::setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
                           << "mechanical power (" << muscles[i].getName() << ")";
                        ROS_DEBUG("%s", ss.str().c_str());
                    }
                } catch (std::exception &e) {
                    ROS_ERROR("Excpetion while attaching mechanical power : %s", e.what());
                    throw e;
                }

                // Attach Umberger2010 probe to record total rate of energy liberation at
                // each point in time for muscle1.
                OpenSim::Umberger2010MuscleMetabolicsProbe *umbergerTotal_rate[muscles.getSize()];
                try {
                    for (int i = 0; i < muscles.getSize(); i++) {
                        umbergerTotal_rate[i] = new
                                OpenSim::Umberger2010MuscleMetabolicsProbe(true, true, true, true);
                        o_model->addProbe(umbergerTotal_rate[i]);
                        std::stringstream name;
                        name << "umbergerTotal_rate_m" << i;
                        umbergerTotal_rate[i]->setName(name.str());
                        umbergerTotal_rate[i]->setOperation("value");
                        umbergerTotal_rate[i]->addMuscle(muscles[i].getName(), 0.5);
                        ss.str("");
                        ss << std::setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
                           << "total rate of energy liberation (" << muscles[i].getName() << ")";
                        ROS_DEBUG("%s", ss.str().c_str());
                    }
                } catch (std::exception &e) {
                    ROS_ERROR("Excpetion while attaching total rate of energy liberation : %s", e.what());
                    throw e;
                }

                // Attach Umberger2010 probes to record total energy liberation over the
                // entire simulation for (a) muscle1, (b) muscle2, (c) total for both
                // muscles, and (d) total for both muscles with all components reported.
                OpenSim::Umberger2010MuscleMetabolicsProbe *umbergerTotal[muscles.getSize()];
                try {
                    for (int i = 0; i < muscles.getSize(); i++) {
                        umbergerTotal[i] = new
                                OpenSim::Umberger2010MuscleMetabolicsProbe(true, true, true, true);
                        o_model->addProbe(umbergerTotal[i]);
                        std::stringstream name;
                        name << "umbergerTotal_m" << i;
                        umbergerTotal[i]->setName(name.str());
                        umbergerTotal[i]->setOperation("integrate");
                        umbergerTotal[i]->setInitialConditions(SimTK::Vector(SimTK::Vec1(0)));
                        umbergerTotal[i]->addMuscle(muscles[i].getName(), 0.5);
                        ss.str("");
                        ss << std::setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
                           << "total energy liberation (" << muscles[i].getName() << ")";
                        ROS_DEBUG("%s", ss.str().c_str());
                    }
                } catch (std::exception &e) {
                    ROS_ERROR("Excpetion while attaching total energy liberation : %s", e.what());
                    throw e;
                }

                extraColumns += 3;

                // Attach reporters.
                probeReporter = new OpenSim::ProbeReporter(o_model);
                o_model->addAnalysis(probeReporter);
                muscleAnalysis = new OpenSim::MuscleAnalysis(o_model);
                o_model->addAnalysis(muscleAnalysis);

                // Print the model.
                printf("\n");
                o_model->printBasicInfo(std::cout);
                printf("\n");
                const std::string baseFilename = "testMuscleMetabolicsProbes";

                manager.setModel(*o_model);

                // Initialize model and state.
                ss.str("");
                ss << "- initializing" << std::endl;
                ROS_DEBUG("%s", ss.str().c_str());
                SimTK::State &state = o_model->initSystem();
                for (int i = 0; i < o_model->getMuscles().getSize(); ++i)
                    o_model->getMuscles().get(i).setIgnoreActivationDynamics(state, true);
                o_model->getMultibodySystem().realize(state, SimTK::Stage::Dynamics);
                o_model->equilibrateMuscles(state);


                ROS_INFO("Finished test");

            } catch (const std::exception &e) {
                ROS_ERROR("Caught an exception while doing the metabolic shit: %s", e.what());
                throw e;
            }

            // ---- ros stuff ----
            metabolic_pub = rosNode->advertise<roboy_simulation_msgs::MetabolicCost>("metabolic_cost", 1000);
        }

        // Called by the world update start event
        void OnUpdate(const common::UpdateInfo &_info) {
            // Apply a small linear velocity to the model.
            //this->model->SetLinearVel(math::Vector3(.03, 0, 0));
            double simTime1 = _info.simTime.Double();
            ROS_DEBUG("simTime0: %f <-> simTime1: %f", simTime0, simTime1);

            //--------------------------------------------------------------------------
            // Run simulation.
            //--------------------------------------------------------------------------
            try {


                // pub muscle activation
                std::stringstream ss;

                for (int i = 0; i < o_model->getMuscles().getSize(); i++) {
                    ss << o_model->getMuscles().get(i).getName() << " - activation : " <<
                       o_model->getMuscles().get(i).getActivation(o_model->getWorkingState()) << std::endl;
                }
                ROS_DEBUG("%s", ss.str().c_str());

                // Prepare integrator.
                const double integrationAccuracy = 1.0e-8;
                SimTK::RungeKuttaMersonIntegrator integrator(o_model->getMultibodySystem());
                integrator.setAccuracy(integrationAccuracy);
                manager.setIntegrator(&integrator);
                manager.setInitialTime(simTime0);
                manager.setFinalTime(simTime1);

                // Simulate.
                const clock_t tStart = clock();
                ss.str("");
                ss << "- integrating from " << simTime0 << " to " << simTime1 << "s" << std::endl;
                ROS_DEBUG("%s", ss.str().c_str());
                SimTK::State state = o_model->getWorkingState();
                manager.integrate(state, 1.0e-3);
                ss.str("");
                ss << "- simulation complete (" << (double) (clock() - tStart) / CLOCKS_PER_SEC
                   << " seconds elapsed)" << std::endl;
                ROS_DEBUG("%s", ss.str().c_str());

                // Release integrator from manager.
                manager.setIntegrator(0);
            } catch (std::exception &e) {
                ROS_ERROR("Excpetion while simulating : %s", e.what());
                throw e;
            }

            simTime0 = simTime1;


        }

        void OnUpdateEnd(/*const common::UpdateInfo & _info*/) {
            // Store column indices.
            const OpenSim::Storage &probeStorage = probeReporter->getProbeStorage();
            const int numProbeOutputs = probeStorage.getColumnLabels().getSize();

            try {
                roboy_simulation_msgs::MetabolicCost msg;
                msg.simTimestamp = simTime0;
                OpenSim::Array<double> probeData;
                probeData.setSize(numProbeOutputs);
                probeStorage.getDataAtTime(simTime0, numProbeOutputs, probeData);

                for (int i = 0; i < numProbeOutputs - 1; i++) {
                    const auto currLabel = probeStorage.getColumnLabels().get(i + 1);
                    if (currLabel.find("umbergerActMaint") != std::string::npos) {
                        msg.umbergerActMaint_rate.push_back(probeData[i]);
                    } else if (currLabel.find("umbergerShorten_rate") != std::string::npos) {
                        msg.umbergerShorten_rate.push_back(probeData[i]);
                    } else if (currLabel.find("umbergerBasal_rate") != std::string::npos) {
                        msg.umbergerBasal_rate.push_back(probeData[i]);
                    } else if (currLabel.find("umbergerMechWork_rate") != std::string::npos) {
                        msg.umbergerMechWork_rate.push_back(probeData[i]);
                    } else if (currLabel.find("umbergerTotal_rate") != std::string::npos) {
                        msg.umbergerTotal_rate.push_back(probeData[i]);
                    } else if (currLabel.find("umbergerTotal") != std::string::npos) {
                        msg.umbergerTotal.push_back(probeData[i]);
                    } else {
                        ROS_ERROR("No matching array for %s", currLabel.c_str());
                    }
                }
                metabolic_pub.publish(msg);
            } catch (std::exception &e) {
                ROS_ERROR("Excpetion while accessing and printing the data : %s", e.what());
                throw e;
            }
        }

        MetabolicPlugin() {
            if (!ros::isInitialized()) {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "MetabolicCostAnalysis",
                          ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
            }

            rosNode = std::make_unique<ros::NodeHandle>("metabolic_plugin");
        }

        ~MetabolicPlugin() {
            rosNode->shutdown();
        }

        void Init() {
            gazebo::ModelPlugin::Init();
            if (!g_model || !engine) return;

            const auto &muscles = g_model->GetMuscles();

        }




    private:
        // Pointer to the model
        physics::OpensimModelPtr g_model;
        physics::OpensimPhysicsPtr engine;

        OpenSim::Model *o_model;
        ConstantExcitationMuscleController *controller;
        OpenSim::Manager manager;
        OpenSim::ProbeReporter *probeReporter;
        OpenSim::MuscleAnalysis *muscleAnalysis;
        double simTime0;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
        event::ConnectionPtr updateConnectionEnd;

        std::unique_ptr<ros::NodeHandle> rosNode;

        ros::Publisher metabolic_pub;
    };

// Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(MetabolicPlugin)

}
