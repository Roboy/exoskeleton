#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>
#include <sstream>

#include "OpenSim/Simulation/Control/Controller.h"

#include "OpenSim/Simulation/Manager/Manager.h"

#include "OpenSim/Simulation/Model/Model.h"
#include "OpenSim/Simulation/Model/Umberger2010MuscleMetabolicsProbe.h"
#include "OpenSim/Simulation/Model/Bhargava2004MuscleMetabolicsProbe.h"
#include "OpenSim/Simulation/Model/Muscle.h"

#include "OpenSim/Simulation/SimbodyEngine/SliderJoint.h"
#include "OpenSim/Simulation/SimbodyEngine/Body.h"

#include "OpenSim/Actuators/Thelen2003Muscle.h"
#include "OpenSim/Actuators/Millard2012EquilibriumMuscle.h"
#include "OpenSim/Analyses/ProbeReporter.h"
#include "OpenSim/Analyses/MuscleAnalysis.h"

#include "OpenSim/Common/Sine.h"

#include <SimTKcommon.h>

#include <gazebo/physics/World.hh>


const bool OUTPUT_FILES = false;

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//==============================================================================
//                    CONSTANT-EXCITATION MUSCLE CONTROLLER
//==============================================================================
// Simple controller to maintain all muscle excitations at the same constant
// value. The metabolic probes depend on both excitation and activation.
class ConstantExcitationMuscleController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(ConstantExcitationMuscleController, Controller);
public:
    ConstantExcitationMuscleController(double u) : _u(u) {}

    void computeControls(const SimTK::State& s, SimTK::Vector &controls) const
    OVERRIDE_11
    {
        for (int i=0; i<_model->getMuscles().getSize(); ++i)
            controls[i] = _u;
    }

    void setConstantExcitation(double u) { _u = u; }

private:
    double _u;
};

template <typename T>
void ASSERT_EQUAL(T expected, T found, T tolerance, std::string file="", int line=-1, std::string message="") {
    if (found < expected - tolerance || found > expected + tolerance)
        throw OpenSim::Exception(message, file, line);
}
inline void ASSERT(bool cond, std::string file="", int line=-1, std::string message="Exception") {
    if (!cond) throw OpenSim::Exception(message, file, line);
}

//==============================================================================
//      TEST UMBERGER AND BHARGAVA PROBES USING MILLARD EQUILIBRIUM MUSCLE
//==============================================================================
// Builds an OpenSim model consisting of two Millard2012Equilibrium muscles,
// attaches several Umberger2010 and Bhargava2004 muscle metabolics probes, and
// confirms that the probes are functioning properly:
//   - probes and muscles can be added and removed
//   - muscle mass parameter is correctly handled
//   - probe components are correctly reported individually and combined
//   - mechanical work rate is calculated correctly
//   - total energy at final time equals integral of total rate
//   - multiple muscles are correctly handled
//   - less energy is liberated with lower activation
void simulateModel(Model& model, Manager& manager, double t0, double t1)
{
    // Initialize model and state.
    cout << "- initializing" << endl;
    SimTK::State& state = model.initSystem();
    for (int i=0; i<model.getMuscles().getSize(); ++i)
        model.getMuscles().get(i).setIgnoreActivationDynamics(state, true);
    model.getMultibodySystem().realize(state, SimTK::Stage::Dynamics);
    model.equilibrateMuscles(state);

    // Prepare integrator.
    const double integrationAccuracy = 1.0e-8;
    SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    integrator.setAccuracy(integrationAccuracy);
    manager.setIntegrator(&integrator);
    manager.setInitialTime(t0);
    manager.setFinalTime(t1);

    // Simulate.
    const clock_t tStart = clock();
    cout << "- integrating from " << t0 << " to " << t1 << "s" << endl;
    manager.integrate(state, 1.0e-3);
    cout << "- simulation complete (" << (double)(clock()-tStart)/CLOCKS_PER_SEC
         << " seconds elapsed)" << endl;

    // Release integrator from manager.
    manager.setIntegrator(0);
}

void testProbesUsingMillardMuscleSimulation()
{
    //--------------------------------------------------------------------------
    // Build an OpenSim model consisting of two Millard2012Equilibrium muscles.
    //--------------------------------------------------------------------------
    Model model;
    model.setName("testModel_metabolics");
    OpenSim::Body& ground = model.getGroundBody();

    // Create block.
    const double blockMass       = 1.0;
    const double blockSideLength = 0.1;
    Inertia blockInertia = blockMass * Inertia::brick(Vec3(blockSideLength/2));
    OpenSim::Body *block = new OpenSim::Body("block", blockMass, Vec3(0),
                                             blockInertia);
    block->addDisplayGeometry("block.vtp");

    // Create slider joint between ground and block.
    OpenSim::SliderJoint prismatic("prismatic", ground, Vec3(0), Vec3(0),
                                   *block, Vec3(0), Vec3(0));
    CoordinateSet& prisCoordSet = prismatic.upd_CoordinateSet();
    prisCoordSet[0].setName("xTranslation");
    prisCoordSet[0].setRangeMin(-1);
    prisCoordSet[0].setRangeMax(1);

    // Prescribe motion.
    Sine motion(0.1, SimTK::Pi, 0);
    prisCoordSet[0].setPrescribedFunction(motion);
    prisCoordSet[0].setDefaultIsPrescribed(true);
    model.addBody(block);

    // Create muscles attached to ground and block.
    //                    _______
    //                   |       |
    // x---[ muscle1 ]---| block |---[ muscle2 ]---x
    //                   |_______|
    //
    //                       0 --> +
    const double optimalFiberLength = 0.1;
    const double tendonSlackLength  = 0.2;
    const double anchorDistance     = optimalFiberLength + tendonSlackLength
                                      + blockSideLength/2;
    double desiredActivation        = 1.0;

    Millard2012EquilibriumMuscle *muscle1 = new Millard2012EquilibriumMuscle(
            "muscle1", 100, optimalFiberLength, tendonSlackLength, 0);
    muscle1->addNewPathPoint("m1_ground", ground, Vec3(-anchorDistance,0,0));
    muscle1->addNewPathPoint("m1_block",  *block, Vec3(-blockSideLength/2,0,0));
    muscle1->setDefaultActivation(desiredActivation);
    model.addForce(muscle1);

    Millard2012EquilibriumMuscle *muscle2 = new Millard2012EquilibriumMuscle(
            "muscle2", 100, optimalFiberLength, tendonSlackLength, 0);
    muscle2->addNewPathPoint("m2_ground", ground, Vec3(anchorDistance,0,0));
    muscle2->addNewPathPoint("m2_block",  *block, Vec3(blockSideLength/2,0,0));
    muscle2->setDefaultActivation(desiredActivation);
    model.addForce(muscle2);

    // Attach muscle controller.
    ConstantExcitationMuscleController* controller =
            new ConstantExcitationMuscleController(desiredActivation);
    controller->setActuators(model.updActuators());
    model.addController(controller);

    //--------------------------------------------------------------------------
    // Attach Umberger2010 and Bhargava2004 muscle metabolics probes.
    //--------------------------------------------------------------------------
    int probeCounter = 0;       // Number of probes attached.
    int extraColumns = 0;       // Number of columns expected in storage file is
    // probeCounter + extraColumns.
    const int w = 4;

    // Attach Umberger2010 probes to record individual heat rate and mechanical
    // power components at each point in time for muscle1.
    Umberger2010MuscleMetabolicsProbe* umbergerActMaint_rate_m1 = new
            Umberger2010MuscleMetabolicsProbe(true, false, false, false);
    model.addProbe(umbergerActMaint_rate_m1);
    umbergerActMaint_rate_m1->setName("umbergerActMaint_rate_m1");
    umbergerActMaint_rate_m1->setOperation("value");
    umbergerActMaint_rate_m1->addMuscle(muscle1->getName(), 0.5);
    cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
         << "activation and maintenance heat rate (muscle 1)" << endl;

    Umberger2010MuscleMetabolicsProbe* umbergerShorten_rate_m1 = new
            Umberger2010MuscleMetabolicsProbe(false, true, false, false);
    model.addProbe(umbergerShorten_rate_m1);
    umbergerShorten_rate_m1->setName("umbergerShorten_rate_m1");
    umbergerShorten_rate_m1->setOperation("value");
    umbergerShorten_rate_m1->addMuscle(muscle1->getName(), 0.5);
    cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
         << "shortening and lengthening heat rate (muscle 1)" << endl;

    Umberger2010MuscleMetabolicsProbe* umbergerBasal_rate_m1 = new
            Umberger2010MuscleMetabolicsProbe(false, false, true, false);
    model.addProbe(umbergerBasal_rate_m1);
    umbergerBasal_rate_m1->setName("umbergerBasal_rate_m1");
    umbergerBasal_rate_m1->setOperation("value");
    umbergerBasal_rate_m1->addMuscle(muscle1->getName(), 0.5);
    cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
         << "basal heat rate (muscle 1)" << endl;

    Umberger2010MuscleMetabolicsProbe* umbergerMechWork_rate_m1 = new
            Umberger2010MuscleMetabolicsProbe(false, false, false, true);
    model.addProbe(umbergerMechWork_rate_m1);
    umbergerMechWork_rate_m1->setName("umbergerMechWork_rate_m1");
    umbergerMechWork_rate_m1->setOperation("value");
    umbergerMechWork_rate_m1->addMuscle(muscle1->getName(), 0.5);
    cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
         << "mechanical power (muscle 1)" << endl;

    // Attach Umberger2010 probe to record total rate of energy liberation at
    // each point in time for muscle1.
    Umberger2010MuscleMetabolicsProbe* umbergerTotal_rate_m1 = new
            Umberger2010MuscleMetabolicsProbe(true, true, true, true);
    model.addProbe(umbergerTotal_rate_m1);
    umbergerTotal_rate_m1->setName("umbergerTotal_rate_m1");
    umbergerTotal_rate_m1->setOperation("value");
    umbergerTotal_rate_m1->addMuscle(muscle1->getName(), 0.5);
    cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
         << "total rate of energy liberation (muscle 1)" << endl;

    // Attach Umberger2010 probes to record total energy liberation over the
    // entire simulation for (a) muscle1, (b) muscle2, (c) total for both
    // muscles, and (d) total for both muscles with all components reported.
    Umberger2010MuscleMetabolicsProbe* umbergerTotal_m1 = new
            Umberger2010MuscleMetabolicsProbe(true, true, true, true);
    model.addProbe(umbergerTotal_m1);
    umbergerTotal_m1->setName("umbergerTotal_m1");
    umbergerTotal_m1->setOperation("integrate");
    umbergerTotal_m1->setInitialConditions(Vector(Vec1(0)));
    umbergerTotal_m1->addMuscle(muscle1->getName(), 0.5);
    cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
         << "total energy liberation (muscle 1)" << endl;

    Umberger2010MuscleMetabolicsProbe* umbergerTotal_m2 = new
            Umberger2010MuscleMetabolicsProbe(true, true, true, true);
    model.addProbe(umbergerTotal_m2);
    umbergerTotal_m2->setName("umbergerTotal_m2");
    umbergerTotal_m2->setOperation("integrate");
    umbergerTotal_m2->setInitialConditions(Vector(Vec1(0)));
    umbergerTotal_m2->addMuscle(muscle2->getName(), 0.5);
    cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
         << "total energy liberation (muscle 2)" << endl;

    Umberger2010MuscleMetabolicsProbe* umbergerTotal_both = new
            Umberger2010MuscleMetabolicsProbe(true, true, true, true);
    model.addProbe(umbergerTotal_both);
    umbergerTotal_both->setName("umbergerTotal_both");
    umbergerTotal_both->setOperation("integrate");
    umbergerTotal_both->setInitialConditions(Vector(Vec1(0)));
    umbergerTotal_both->addMuscle(muscle1->getName(), 0.5);
    umbergerTotal_both->addMuscle(muscle2->getName(), 0.5);
    cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
         << "total energy liberation (both muscles, total only)" << endl;

    Umberger2010MuscleMetabolicsProbe* umbergerTotalAllPieces_both = new
            Umberger2010MuscleMetabolicsProbe(true, true, true, true);
    model.addProbe(umbergerTotalAllPieces_both);
    umbergerTotalAllPieces_both->setName("umbergerTotalAllPieces_both");
    umbergerTotalAllPieces_both->setOperation("integrate");
    umbergerTotalAllPieces_both->set_report_total_metabolics_only(false);
    umbergerTotalAllPieces_both->setInitialConditions(Vector(Vec4(0)));
    umbergerTotalAllPieces_both->addMuscle(muscle1->getName(), 0.5);
    umbergerTotalAllPieces_both->addMuscle(muscle2->getName(), 0.5);
    cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
         << "total energy liberation (both muscles, all components)" << endl;
    extraColumns += 3;

    // Attach reporters.
    ProbeReporter* probeReporter = new ProbeReporter(&model);
    model.addAnalysis(probeReporter);
    MuscleAnalysis* muscleAnalysis = new MuscleAnalysis(&model);
    model.addAnalysis(muscleAnalysis);

    // Print the model.
    printf("\n"); model.printBasicInfo(cout); printf("\n");
    const std::string baseFilename = "testMuscleMetabolicsProbes";
    if (OUTPUT_FILES) {
        const std::string fname = baseFilename + "Model.osim";
        model.print(fname);
        cout << "+ saved model file: " << fname << endl;
    }

    //--------------------------------------------------------------------------
    // Run simulation.
    //--------------------------------------------------------------------------
    const double t0 = 0.0;
    const double t1 = 2.0;
    Manager manager(model);
    simulateModel(model, manager, t0, t1);

    // Output results files.
    if (OUTPUT_FILES) {
        std::string fname = baseFilename + "_states.sto";
        Storage stateStorage(manager.getStateStorage());
        stateStorage.print(fname);
        cout << "+ saved state storage file: " << fname << endl;

        fname = baseFilename + "_probes.sto";
        probeReporter->getProbeStorage().print(fname);
        cout << "+ saved probe storage file: " << fname << endl;

        fname = baseFilename + "_activeFiberForce.sto";
        muscleAnalysis->getActiveFiberForceStorage()->print(fname);
        cout << "+ saved active fiber force storage file: " << fname << endl;

        fname = baseFilename + "_fiberVelocity.sto";
        muscleAnalysis->getFiberVelocityStorage()->print(fname);
        cout << "+ saved fiber velocity storage file: " << fname << endl;
    }

    // Store column indices.
    const Storage& probeStorage = probeReporter->getProbeStorage();
    const int numProbeOutputs = probeStorage.getColumnLabels().getSize()-1;
    ASSERT(numProbeOutputs == probeCounter+extraColumns, __FILE__, __LINE__,
           "Incorrect number of columns in probe storage.");
    cout << "size columns labels: " << probeStorage.getColumnLabels().getSize() << std::endl;
    for(int i = 0; i < probeStorage.getColumnLabels().getSize(); i++){
        cout << "  - " << probeStorage.getColumnLabels().get(i) << std::endl;
    }

    std::map<std::string, int> probeCol;
    probeCol["umbActMaint_rate_m1"] = probeStorage
                                              .getColumnIndicesForIdentifier("umbergerActMaint_rate_m1_TOTAL")[0]-1;
    probeCol["umbShorten_rate_m1"] = probeStorage
                                             .getColumnIndicesForIdentifier("umbergerShorten_rate_m1_TOTAL")[0]-1;
    probeCol["umbBasal_rate_m1"] = probeStorage
                                           .getColumnIndicesForIdentifier("umbergerBasal_rate_m1_TOTAL")[0]-1;
    probeCol["umbMechWork_rate_m1"] = probeStorage
                                              .getColumnIndicesForIdentifier("umbergerMechWork_rate_m1_TOTAL")[0]-1;
    probeCol["umbTotal_rate_m1"] = probeStorage
                                           .getColumnIndicesForIdentifier("umbergerTotal_rate_m1_TOTAL")[0]-1;
    probeCol["umbTotal_m1"] = probeStorage
                                      .getColumnIndicesForIdentifier("umbergerTotal_m1_TOTAL")[0]-1;
    probeCol["umbTotal_m2"] = probeStorage
                                      .getColumnIndicesForIdentifier("umbergerTotal_m2_TOTAL")[0]-1;
    probeCol["umbTotal_both"] = probeStorage
                                        .getColumnIndicesForIdentifier("umbergerTotal_both_TOTAL")[0]-1;
    probeCol["umbTotalAllPieces_both_total"] = probeStorage
                                                       .getColumnIndicesForIdentifier("umbergerTotalAllPieces_both_TOTAL")[0]-1;
    probeCol["umbTotalAllPieces_both_basal"] = probeStorage
                                                       .getColumnIndicesForIdentifier("umbergerTotalAllPieces_both_BASAL")[0]-1;
    probeCol["umbTotalAllPieces_both_muscle1"] = probeStorage
                                                         .getColumnIndicesForIdentifier("umbergerTotalAllPieces_both_muscle1")[0]-1;
    probeCol["umbTotalAllPieces_both_muscle2"] = probeStorage
                                                         .getColumnIndicesForIdentifier("umbergerTotalAllPieces_both_muscle2")[0]-1;

    Storage* forceStorage = muscleAnalysis->getActiveFiberForceStorage();
    const int numForceOutputs = forceStorage->getColumnLabels().getSize()-1;
    ASSERT(numForceOutputs == 2, __FILE__, __LINE__,
           "Incorrect number of columns in active fiber force storage.");
    const int idx_force_muscle1 = forceStorage
                                          ->getColumnIndicesForIdentifier(muscle1->getName())[0]-1;

    Storage* fibVelStorage = muscleAnalysis->getFiberVelocityStorage();
    const int numFibVelOutputs = fibVelStorage->getColumnLabels().getSize()-1;
    ASSERT(numFibVelOutputs == 2, __FILE__, __LINE__,
           "Incorrect number of columns in fiber velocity storage.");
    const int idx_fibVel_muscle1 = fibVelStorage
                                           ->getColumnIndicesForIdentifier(muscle1->getName())[0]-1;

    //--------------------------------------------------------------------------
    // Check instantaneous power results at numPoints evenly-spaced instants.
    //--------------------------------------------------------------------------
    cout << "- checking instantaneous power results" << endl;
    const int numPoints = 21;
    for (int i=0; i<numPoints; ++i) {
        double t = t0 + (double)i/(numPoints-1)*(t1-t0);

        // Get data from probe at current time.
        Array<double> probeData;
        probeData.setSize(numProbeOutputs);
        probeStorage.getDataAtTime(t, numProbeOutputs, probeData);

        // Output from the probes reporting individual heat rates and mechanical
        // power must sum to the output from the probe reporting the total rate
        // of energy liberation.
        ASSERT_EQUAL(probeData[probeCol["umbActMaint_rate_m1"]]
                     + probeData[probeCol["umbShorten_rate_m1"]]
                     + probeData[probeCol["umbBasal_rate_m1"]]
                     + probeData[probeCol["umbMechWork_rate_m1"]],
                     probeData[probeCol["umbTotal_rate_m1"]],
                     100*SimTK::SignificantReal, __FILE__, __LINE__,
                     "Umberger2010: wrong sum of individual rates and mechanical power.");

        // Total rate of energy liberation reported must not depend on whether
        // the individual components are reported as well.
        ASSERT_EQUAL(probeData[probeCol["umbTotal_both"]],
                     probeData[probeCol["umbTotalAllPieces_both_total"]],
                     100*SimTK::SignificantReal, __FILE__, __LINE__,
                     "Umberger2010: total heat rate changes if components are reported.");

        // Mechanical work rates should agree with fiber velocity and active
        // fiber force data.
        Array<double> forceData;
        forceData.setSize(numForceOutputs);
        forceStorage->getDataAtTime(t, numForceOutputs, forceData);

        Array<double> fibVelData;
        fibVelData.setSize(numFibVelOutputs);
        fibVelStorage->getDataAtTime(t, numFibVelOutputs, fibVelData);

        const double powerExpected = -forceData[idx_force_muscle1]
                                     * fibVelData[idx_fibVel_muscle1];
        ASSERT_EQUAL(probeData[probeCol["umbMechWork_rate_m1"]], powerExpected,
                     1.0e-2, __FILE__, __LINE__,
                     "Umberger2010: mechanical power disagrees with muscle analysis.");
    }

    //--------------------------------------------------------------------------
    // Integrate rates and check total energy liberation results at time t1.
    //--------------------------------------------------------------------------
    Storage* probeStorageInt = probeStorage.integrate(t0, t1);
    if (OUTPUT_FILES) {
        std::string fname = baseFilename + "_probesInteg.sto";
        probeStorageInt->print(fname);
        cout << "+ saved integrated probe storage file: " << fname << endl;
    }
    cout << "- checking total energy liberation results" << endl;

    // Get data from probes at final time.
    Array<double> probeData_t1;
    probeData_t1.setSize(numProbeOutputs);
    probeStorage.getDataAtTime(t1, numProbeOutputs, probeData_t1);

    Array<double> probeDataInt;
    probeDataInt.setSize(numProbeOutputs);
    probeStorageInt->getDataAtTime(t1, numProbeOutputs, probeDataInt);

    // Total energy at final time must equal integral of total rate.
    ASSERT_EQUAL(probeData_t1[probeCol["umbTotal_m1"]],
                 probeDataInt[probeCol["umbTotal_rate_m1"]],
                 1.0e-2, __FILE__, __LINE__,
                 "Umberger2010: integral of total rate differs from final total energy.");


    // Check reporting of metabolic probe components: Umberger2010.
    ASSERT_EQUAL(probeData_t1[probeCol["umbTotalAllPieces_both_basal"]],
                 probeDataInt[probeCol["umbBasal_rate_m1"]],
                 1.0e-2, __FILE__, __LINE__,
                 "Umberger2010: error in reporting components of metabolic probe.");

    ASSERT_EQUAL(probeData_t1[probeCol["umbTotalAllPieces_both_muscle1"]],
                 probeData_t1[probeCol["umbTotal_m1"]]
                 - probeDataInt[probeCol["umbBasal_rate_m1"]],
                 1.0e-2, __FILE__, __LINE__,
                 "Umberger2010: error in reporting components of metabolic probe.");

    ASSERT_EQUAL(probeData_t1[probeCol["umbTotalAllPieces_both_muscle2"]],
                 probeData_t1[probeCol["umbTotal_m2"]]
                 - probeDataInt[probeCol["umbBasal_rate_m1"]],
                 1.0e-2, __FILE__, __LINE__,
                 "Umberger2010: error in reporting components of metabolic probe.");

    // Check reporting for multiple muscles.
    //   Total energy for muscle1      = basal + heat1 + work1
    //   Total energy for muscle2      = basal + heat2 + work2
    //   Total energy for both muscles = basal + heat1 + heat2 + work1 + work2
    ASSERT_EQUAL(probeData_t1[probeCol["umbTotal_both"]],
                 probeData_t1[probeCol["umbTotal_m1"]]
                 + probeData_t1[probeCol["umbTotal_m2"]]
                 - probeDataInt[probeCol["umbBasal_rate_m1"]],
                 1.0e-2, __FILE__, __LINE__,
                 "Umberger2010: error in reporting data for multiple muscles.");

    Array<Array<double>> idData;
    probeStorage.getDataForIdentifier(probeStorage.getColumnLabels().get(1), idData);
    cout << "getDataForIdentifier, first size: " << idData.size() << std::endl;
    cout << "second size: " << idData.get(0).size() << std::endl;
    //--------------------------------------------------------------------------
    // Run simulation with lower activation and ensure less energy is liberated.
    //--------------------------------------------------------------------------
    cout << "- reconfiguring with lower activation and fewer probes" << endl;
    desiredActivation = 0.5;
    muscle1->setDefaultActivation(desiredActivation);
    muscle2->setDefaultActivation(desiredActivation);
    controller->setConstantExcitation(desiredActivation);

    // Retain only the probes that report the total energy liberated.
    model.removeAnalysis(probeReporter);
    model.removeAnalysis(muscleAnalysis);
    int idx = 0;
    while (idx < model.getProbeSet().getSize()) {
        std::string thisName = model.getProbeSet().get(idx).getName();
        if (thisName=="umbergerTotal_m1" || thisName=="umbergerTotal_m2")
            idx++;
        else
            model.removeProbe(&model.getProbeSet().get(idx));
    }
    ProbeReporter* probeReporter2 = new ProbeReporter(&model);
    model.addAnalysis(probeReporter2);

    // Print the model.
    printf("\n"); model.printBasicInfo(cout); printf("\n");
    if (OUTPUT_FILES) {
        const std::string fname = baseFilename + "Model2.osim";
        model.print(fname);
        cout << "+ saved model file: " << fname << endl;
    }

    // Simulate.
    Manager manager2(model);
    simulateModel(model, manager2, t0, t1);

    if (OUTPUT_FILES) {
        std::string fname = baseFilename + "_states2.sto";
        Storage stateStorage2(manager2.getStateStorage());
        stateStorage2.print(fname);
        cout << "+ saved state storage file: " << fname << endl;

        fname = baseFilename + "_probes2.sto";
        probeReporter2->getProbeStorage().print(fname);
        cout << "+ saved probe storage file: " << fname << endl;
    }

    // Total energy liberated must be less than that liberated in the first
    // simulation, where muscle activation was higher.
    cout << "- checking total energy liberation results" << endl;

    const Storage& probeStorage2 = probeReporter2->getProbeStorage();
    const int numProbeOutputs2 = probeStorage2.getColumnLabels().getSize()-1;
    ASSERT(numProbeOutputs2 == 2, __FILE__, __LINE__,
           "Incorrect number of columns in probe storage.");

    Array<double> probeData2_t1;
    probeData2_t1.setSize(numProbeOutputs2);
    probeStorage2.getDataAtTime(t1, numProbeOutputs2, probeData2_t1);

    std::map<std::string, int> probeCol2;
    probeCol2["umbTotal_m1"] = probeStorage2
                                       .getColumnIndicesForIdentifier("umbergerTotal_m1_TOTAL")[0]-1;
    probeCol2["umbTotal_m2"] = probeStorage2
                                       .getColumnIndicesForIdentifier("umbergerTotal_m2_TOTAL")[0]-1;

    ASSERT(probeData_t1[probeCol["umbTotal_m1"]] >
           probeData2_t1[probeCol2["umbTotal_m1"]], __FILE__, __LINE__,
           "Umberger2010: total energy must decrease with lower activation.");
    ASSERT(probeData_t1[probeCol["umbTotal_m2"]] >
           probeData2_t1[probeCol2["umbTotal_m2"]], __FILE__, __LINE__,
           "Umberger2010: total energy must decrease with lower activation.");
}

void testProbesUsingArm26()
{
    //--------------------------------------------------------------------------
    // Build an OpenSim model consisting of two Millard2012Equilibrium muscles.
    //--------------------------------------------------------------------------
    Model model("/home/parallels/Documents/NRP/GazeboRosPackages/src/exoskeleton/input/arm26.osim");
    model.setName("testModel_metabolics");
    OpenSim::Body& ground = model.getGroundBody();

    double desiredActivation        = 1.0;

    // Attach muscle controller.
    ConstantExcitationMuscleController* controller =
            new ConstantExcitationMuscleController(desiredActivation);
    controller->setActuators(model.updActuators());
    model.addController(controller);

    //--------------------------------------------------------------------------
    // Attach Umberger2010 and Bhargava2004 muscle metabolics probes.
    //--------------------------------------------------------------------------
    int probeCounter = 0;       // Number of probes attached.
    int extraColumns = 0;       // Number of columns expected in storage file is
    // probeCounter + extraColumns.
    const int w = 4;

    auto muscles = model.getMuscles();
    cout << "num muscles: " << muscles.getSize() << std::endl;

    // Attach Umberger2010 probes to record individual heat rate and mechanical
    // power components at each point in time for muscle1.
    Umberger2010MuscleMetabolicsProbe* umbergerActMaint_rate[muscles.getSize()];
    for(int i = 0; i < muscles.getSize() ; i++) {
        umbergerActMaint_rate[i] = new
                Umberger2010MuscleMetabolicsProbe(true, false, false, false);
        model.addProbe(umbergerActMaint_rate[i]);
        std::stringstream ss;
        ss << "umbergerActMaint_rate_m" << i;
        umbergerActMaint_rate[i]->setName(ss.str());
        umbergerActMaint_rate[i]->setOperation("value");
        umbergerActMaint_rate[i]->addMuscle(muscles[i].getName(), 0.5);
        cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
             << "activation and maintenance heat rate (" << muscles[i].getName() << ")" << endl;
    }

    Umberger2010MuscleMetabolicsProbe* umbergerShorten_rate[muscles.getSize()];
    for(int i = 0; i < muscles.getSize() ; i++) {
        umbergerShorten_rate[i] = new
                Umberger2010MuscleMetabolicsProbe(false, true, false, false);
        model.addProbe(umbergerShorten_rate[i]);
        std::stringstream ss;
        ss << "umbergerShorten_rate_m" << i;
        umbergerShorten_rate[i]->setName(ss.str());
        umbergerShorten_rate[i]->setOperation("value");
        umbergerShorten_rate[i]->addMuscle(muscles[i].getName(), 0.5);
        cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
             << "shortening and lengthening heat rate (" << muscles[i].getName() << ")" << endl;
    }

    Umberger2010MuscleMetabolicsProbe* umbergerBasal_rate[muscles.getSize()];

    for(int i = 0; i < muscles.getSize() ; i++) {
        umbergerBasal_rate[i] = new
                Umberger2010MuscleMetabolicsProbe(false, false, true, false);
        model.addProbe(umbergerBasal_rate[i]);
        std::stringstream ss;
        ss << "umbergerBasal_rate_m" << i;
        umbergerBasal_rate[i]->setName(ss.str());
        umbergerBasal_rate[i]->setOperation("value");
        umbergerBasal_rate[i]->addMuscle(muscles[i].getName(), 0.5);
        cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
             << "basal heat rate (" << muscles[i].getName() << ")" << endl;
    }

    Umberger2010MuscleMetabolicsProbe* umbergerMechWork_rate[muscles.getSize()];
    for(int i = 0; i < muscles.getSize() ; i++) {
        umbergerMechWork_rate[i] = new
                Umberger2010MuscleMetabolicsProbe(false, false, false, true);
        model.addProbe(umbergerMechWork_rate[i]);
        std::stringstream ss;
        ss << "umbergerMechWork_rate_m" << i;
        umbergerMechWork_rate[i]->setName(ss.str());
        umbergerMechWork_rate[i]->setOperation("value");
        umbergerMechWork_rate[i]->addMuscle(muscles[i].getName(), 0.5);
        cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
             << "mechanical power (" << muscles[i].getName() << ")" << endl;
    }

    // Attach Umberger2010 probe to record total rate of energy liberation at
    // each point in time for muscle1.
    Umberger2010MuscleMetabolicsProbe* umbergerTotal_rate[muscles.getSize()];
    for(int i = 0; i < muscles.getSize() ; i++) {
        umbergerTotal_rate[i] = new
                Umberger2010MuscleMetabolicsProbe(true, true, true, true);
        model.addProbe(umbergerTotal_rate[i]);
        std::stringstream ss;
        ss << "umbergerTotal_rate_m" << i;
        umbergerTotal_rate[i]->setName(ss.str());
        umbergerTotal_rate[i]->setOperation("value");
        umbergerTotal_rate[i]->addMuscle(muscles[i].getName(), 0.5);
        cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
             << "total rate of energy liberation (" << muscles[i].getName() << ")" << endl;
    }

    // Attach Umberger2010 probes to record total energy liberation over the
    // entire simulation for (a) muscle1, (b) muscle2, (c) total for both
    // muscles, and (d) total for both muscles with all components reported.
    Umberger2010MuscleMetabolicsProbe* umbergerTotal[muscles.getSize()];

    for(int i = 0; i < muscles.getSize() ; i++) {
        umbergerTotal[i] = new
                Umberger2010MuscleMetabolicsProbe(true, true, true, true);
        model.addProbe(umbergerTotal[i]);
        std::stringstream ss;
        ss << "umbergerTotal_m" << i;
        umbergerTotal[i]->setName(ss.str());
        umbergerTotal[i]->setOperation("integrate");
        umbergerTotal[i]->setInitialConditions(Vector(Vec1(0)));
        umbergerTotal[i]->addMuscle(muscles[i].getName(), 0.5);
        cout << setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
             << "total energy liberation (" << muscles[i].getName() << ")" << endl;
    }

    extraColumns += 3;

    // Attach reporters.
    ProbeReporter* probeReporter = new ProbeReporter(&model);
    model.addAnalysis(probeReporter);
    MuscleAnalysis* muscleAnalysis = new MuscleAnalysis(&model);
    model.addAnalysis(muscleAnalysis);

    // Print the model.
    printf("\n"); model.printBasicInfo(cout); printf("\n");
    const std::string baseFilename = "testMuscleMetabolicsProbes";
    if (OUTPUT_FILES) {
        const std::string fname = baseFilename + "Model.osim";
        model.print(fname);
        cout << "+ saved model file: " << fname << endl;
    }

    //--------------------------------------------------------------------------
    // Run simulation.
    //--------------------------------------------------------------------------
    const double t0 = 0.0;
    const double t1 = 0.5;
    Manager manager(model);

    // Initialize model and state.
    cout << "- initializing" << endl;
    SimTK::State& state = model.initSystem();
    for (int i=0; i<model.getMuscles().getSize(); ++i)
        model.getMuscles().get(i).setIgnoreActivationDynamics(state, true);
    model.getMultibodySystem().realize(state, SimTK::Stage::Dynamics);
    model.equilibrateMuscles(state);

    // Prepare integrator.
    const double integrationAccuracy = 1.0e-8;
    SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    integrator.setAccuracy(integrationAccuracy);
    manager.setIntegrator(&integrator);
    manager.setInitialTime(t0);
    manager.setFinalTime(t1);

    // Simulate.
    const clock_t tStart = clock();
    cout << "- integrating from " << t0 << " to " << t1 << "s" << endl;
    manager.integrate(state, 1.0e-3);
    cout << "- simulation complete (" << (double)(clock()-tStart)/CLOCKS_PER_SEC
         << " seconds elapsed)" << endl;

    // Release integrator from manager.
    manager.setIntegrator(0);

    // Output results files.
    if (OUTPUT_FILES) {
        std::string fname = baseFilename + "_states.sto";
        Storage stateStorage(manager.getStateStorage());
        stateStorage.print(fname);
        cout << "+ saved state storage file: " << fname << endl;

        fname = baseFilename + "_probes.sto";
        probeReporter->getProbeStorage().print(fname);
        cout << "+ saved probe storage file: " << fname << endl;

        fname = baseFilename + "_activeFiberForce.sto";
        muscleAnalysis->getActiveFiberForceStorage()->print(fname);
        cout << "+ saved active fiber force storage file: " << fname << endl;

        fname = baseFilename + "_fiberVelocity.sto";
        muscleAnalysis->getFiberVelocityStorage()->print(fname);
        cout << "+ saved fiber velocity storage file: " << fname << endl;
    }

    // Store column indices.
    const Storage& probeStorage = probeReporter->getProbeStorage();
    const int numProbeOutputs = probeStorage.getColumnLabels().getSize();
//    ASSERT(numProbeOutputs == probeCounter+extraColumns, __FILE__, __LINE__,
//           "Incorrect number of columns in probe storage.");

    std::map<std::string, int> probeCol;
    cout << "size columns labels: " << probeStorage.getColumnLabels().getSize() << std::endl;
//    cout << "Identifier data : " << std::endl;
//    for(int i = 0; i < probeStorage.getColumnLabels().getSize(); i++){
//        //cout << "  - " << probeStorage.getColumnLabels().get(i) << std::endl;
//        Array<Array<double>> idData;
//        probeStorage.getDataForIdentifier(probeStorage.getColumnLabels().get(i), idData);
//        cout << probeStorage.getColumnLabels().get(i) << " - " << idData << std::endl;
//    }

    Array<double> probeData;
    probeData.setSize(numProbeOutputs);
    probeStorage.getDataAtTime(0.5, numProbeOutputs, probeData);


    cout << "DATA: " << std::endl;
    for(int i = 0; i < numProbeOutputs; i++) {
        cout << probeStorage.getColumnLabels().get(i) << " - " << probeData[i] << std::endl;
    }
}

void testRosPlugin() {
    //--------------------------------------------------------------------------
    // Build the arm26 OpenSim Model
    //--------------------------------------------------------------------------
    OpenSim::Model model(
            "/home/parallels/Documents/NRP/GazeboRosPackages/src/exoskeleton/input/arm26.osim");
    model.setName("testModel_metabolics");

    ROS_INFO("Loaded model");

    OpenSim::Body &ground = model.getGroundBody();


    double desiredActivation = 1.0;

    // Attach muscle controller.
    ConstantExcitationMuscleController *controller =
            new ConstantExcitationMuscleController(desiredActivation);
    controller->setActuators(model.updActuators());
    model.addController(controller);
    ROS_INFO("Added muscle controller");

    //--------------------------------------------------------------------------
    // Attach Umberger2010 and Bhargava2004 muscle metabolics probes.
    //--------------------------------------------------------------------------
    int probeCounter = 0;       // Number of probes attached.
    int extraColumns = 0;       // Number of columns expected in storage file is
    // probeCounter + extraColumns.
    const int w = 4;

    auto muscles = model.getMuscles();
    std::stringstream ss;
    ss << "num muscles: " << muscles.getSize();
    ROS_INFO(ss.str().c_str());

    // Attach Umberger2010 probes to record individual heat rate and mechanical
    // power components at each point in time for muscle1.
    OpenSim::Umberger2010MuscleMetabolicsProbe *umbergerActMaint_rate[muscles.getSize()];
    try {
        for (int i = 0; i < muscles.getSize(); i++) {
            umbergerActMaint_rate[i] = new
                    OpenSim::Umberger2010MuscleMetabolicsProbe(true, false, false, false);
            model.addProbe(umbergerActMaint_rate[i]);
            std::stringstream name;
            name << "umbergerActMaint_rate_m" << i;
            umbergerActMaint_rate[i]->setName(name.str());
            umbergerActMaint_rate[i]->setOperation("value");
            umbergerActMaint_rate[i]->addMuscle(muscles[i].getName(), 0.5);
            ss.str("");
            ss << std::setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
               << "activation and maintenance heat rate (" << muscles[i].getName() << ")";
            ROS_INFO(ss.str().c_str());
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
            model.addProbe(umbergerShorten_rate[i]);
            std::stringstream name;
            name << "umbergerShorten_rate_m" << i;
            umbergerShorten_rate[i]->setName(name.str());
            umbergerShorten_rate[i]->setOperation("value");
            umbergerShorten_rate[i]->addMuscle(muscles[i].getName(), 0.5);
            ss.str("");
            ss << std::setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
               << "shortening and lengthening heat rate (" << muscles[i].getName() << ")";
            ROS_INFO(ss.str().c_str());
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
            model.addProbe(umbergerBasal_rate[i]);
            std::stringstream name;
            name << "umbergerBasal_rate_m" << i;
            umbergerBasal_rate[i]->setName(name.str());
            umbergerBasal_rate[i]->setOperation("value");
            umbergerBasal_rate[i]->addMuscle(muscles[i].getName(), 0.5);
            ss.str("");
            ss << std::setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
               << "basal heat rate (" << muscles[i].getName() << ")";
            ROS_INFO(ss.str().c_str());
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
            model.addProbe(umbergerMechWork_rate[i]);
            std::stringstream name;
            name << "umbergerMechWork_rate_m" << i;
            umbergerMechWork_rate[i]->setName(name.str());
            umbergerMechWork_rate[i]->setOperation("value");
            umbergerMechWork_rate[i]->addMuscle(muscles[i].getName(), 0.5);
            ss.str("");
            ss << std::setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
               << "mechanical power (" << muscles[i].getName() << ")";
            ROS_INFO(ss.str().c_str());
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
            model.addProbe(umbergerTotal_rate[i]);
            std::stringstream name;
            name << "umbergerTotal_rate_m" << i;
            umbergerTotal_rate[i]->setName(name.str());
            umbergerTotal_rate[i]->setOperation("value");
            umbergerTotal_rate[i]->addMuscle(muscles[i].getName(), 0.5);
            ss.str("");
            ss << std::setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
               << "total rate of energy liberation (" << muscles[i].getName() << ")";
            ROS_INFO(ss.str().c_str());
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
            model.addProbe(umbergerTotal[i]);
            std::stringstream name;
            name << "umbergerTotal_m" << i;
            umbergerTotal[i]->setName(name.str());
            umbergerTotal[i]->setOperation("integrate");
            umbergerTotal[i]->setInitialConditions(SimTK::Vector(SimTK::Vec1(0)));
            umbergerTotal[i]->addMuscle(muscles[i].getName(), 0.5);
            ss.str("");
            ss << std::setw(w) << ++probeCounter << ") Added Umberger2010 probe: "
               << "total energy liberation (" << muscles[i].getName() << ")";
            ROS_INFO(ss.str().c_str());
        }
    } catch (std::exception &e) {
        ROS_ERROR("Excpetion while attaching total energy liberation : %s", e.what());
        throw e;
    }

    extraColumns += 3;

    // Attach reporters.
    OpenSim::ProbeReporter *probeReporter = new OpenSim::ProbeReporter(&model);
    model.addAnalysis(probeReporter);
    OpenSim::MuscleAnalysis *muscleAnalysis = new OpenSim::MuscleAnalysis(&model);
    model.addAnalysis(muscleAnalysis);

    // Print the model.
    printf("\n");
    model.printBasicInfo(std::cout);
    printf("\n");
    const std::string baseFilename = "testMuscleMetabolicsProbes";

    //--------------------------------------------------------------------------
    // Run simulation.
    //--------------------------------------------------------------------------
    const double t0 = 0.0;
    const double t1 = 0.5;
    OpenSim::Manager manager(model);

    try {
        // Initialize model and state.
        ss.str("");
        ss << "- initializing";
        ROS_INFO(ss.str().c_str());
        SimTK::State &state = model.initSystem();
        for (int i = 0; i < model.getMuscles().getSize(); ++i)
            model.getMuscles().get(i).setIgnoreActivationDynamics(state, true);
        model.getMultibodySystem().realize(state, SimTK::Stage::Dynamics);
        model.equilibrateMuscles(state);

        // Prepare integrator.
        const double integrationAccuracy = 1.0e-8;
        SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
        integrator.setAccuracy(integrationAccuracy);
        manager.setIntegrator(&integrator);
        manager.setInitialTime(t0);
        manager.setFinalTime(t1);

        // Simulate.
        const clock_t tStart = clock();
        ss.str("");
        ss << "- integrating from " << t0 << " to " << t1 << "s";
        ROS_INFO(ss.str().c_str());
        manager.integrate(state, 1.0e-3);
        ss.str("");
        ss << "- simulation complete (" << (double) (clock() - tStart) / CLOCKS_PER_SEC
           << " seconds elapsed)";
        ROS_INFO(ss.str().c_str());

        // Release integrator from manager.
        manager.setIntegrator(0);
    } catch (std::exception &e) {
        ROS_ERROR("Excpetion while simulating : %s", e.what());
        throw e;
    }
    // Store column indices.
    const OpenSim::Storage &probeStorage = probeReporter->getProbeStorage();
    const int numProbeOutputs = probeStorage.getColumnLabels().getSize();

    try {
        std::map<std::string, int> probeCol;
        ss.str("");
        ss << "size columns labels: " << probeStorage.getColumnLabels().getSize();
        ROS_INFO(ss.str().c_str());

        OpenSim::Array<double> probeData;
        probeData.setSize(numProbeOutputs);
        probeStorage.getDataAtTime(0.5, numProbeOutputs, probeData);


        ss.str("");
        ss << "DATA: ";
        for (int i = 0; i < numProbeOutputs; i++) {
            ss.str("");
            ss << probeStorage.getColumnLabels().get(i) << " - " << probeData[i];
            ROS_INFO(ss.str().c_str());
        }
    } catch (std::exception &e) {
        ROS_ERROR("Excpetion while accessing and printing the data : %s", e.what());
        throw e;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    testRosPlugin();

    return 0;
}
