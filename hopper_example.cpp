//
// Created by cryscan on 2/24/21.
//

#include <cmath>
#include <iostream>

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <towr/initialization/monoped_gait_generator.h>
#include <ifopt/ipopt_solver.h>


using namespace towr;

// A minimal example how to build a trajectory optimization problem using TOWR.
//
// The more advanced example that includes ROS integration, GUI, rviz
// visualization and plotting can be found here:
// towr_ros/src/towr_ros_app.cc
int main() {
    NlpFormulation formulation;

    // terrain
    formulation.terrain_ = HeightMap::MakeTerrain(HeightMap::TerrainID::FlatID);

    // Kinematic limits and dynamic parameters of the hopper
    formulation.model_ = RobotModel(RobotModel::Biped);

    // set the initial position of the hopper
    formulation.initial_base_.lin.at(kPos).z() = 0.5;
    // formulation.initial_ee_W_.push_back(Eigen::Vector3d::Zero());
    formulation.initial_ee_W_ = formulation.model_.kinematic_model_->GetNominalStanceInBase();
    for (auto& pose : formulation.initial_ee_W_) pose.z() = 0;

    // define the desired goal state of the hopper
    formulation.final_base_.lin.at(towr::kPos) << 2.0, 0.0, 0.5;

    // Parameters that define the motion. See c'tor for default values or
    // other values that can be modified.
    // First we define the initial phase durations, that can however be changed
    // by the optimizer. The number of swing and stance phases however is fixed.
    // alternating stance and swing:     ____-----_____-----_____-----_____
    // formulation.params_.ee_phase_durations_.push_back({0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.2});
    // formulation.params_.ee_in_contact_at_start_.push_back(true);
    int ee_count = formulation.model_.dynamic_model_->GetEECount();
    auto gait = GaitGenerator::MakeGaitGenerator(ee_count);
    gait->SetCombo(GaitGenerator::Combos::C0);
    for (int i = 0; i < ee_count; ++i) {
        formulation.params_.ee_phase_durations_.push_back(gait->GetPhaseDurations(2.0, i));
        formulation.params_.ee_in_contact_at_start_.push_back(gait->IsInContactAtStart(i));
    }
    // formulation.params_.OptimizePhaseDurations();

    // Initialize the nonlinear-programming problem with the variables,
    // constraints and costs.
    ifopt::Problem nlp;
    SplineHolder solution;
    for (auto& c : formulation.GetVariableSets(solution))
        nlp.AddVariableSet(c);
    for (auto& c : formulation.GetConstraints(solution))
        nlp.AddConstraintSet(c);
    for (auto& c : formulation.GetCosts())
        nlp.AddCostSet(c);

    // You can add your own elements to the nlp as well, simply by calling:
    // nlp.AddVariablesSet(your_custom_variables);
    // nlp.AddConstraintSet(your_custom_constraints);

    // Choose ifopt solver (IPOPT or SNOPT), set some parameters and solve.
    // solver->SetOption("derivative_test", "first-order");
    auto solver = std::make_shared<ifopt::IpoptSolver>();
    solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
    // solver->SetOption("max_cpu_time", 20.0);
    solver->Solve(nlp);

    // Can directly view the optimization variables through:
    // Eigen::VectorXd x = nlp.GetVariableValues()
    // However, it's more convenient to access the splines constructed from these
    // variables and query their values at specific times:
    using namespace std;
    cout.precision(2);
    nlp.PrintCurrent(); // view variable-set, constraint violations, indices,...
    cout << fixed;
    cout << "\n====================\nMonoped trajectory:\n====================\n";

    double t = 0.0;
    while (t <= solution.base_linear_->GetTotalTime() + 1e-5) {
        cout << "t=" << t << "\n";
        cout << "Base linear position x,y,z:   \t";
        cout << solution.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;

        cout << "Base Euler roll, pitch, yaw:  \t";
        Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
        cout << (rad / M_PI * 180).transpose() << "\t[deg]" << endl;

        cout << "Foot position x,y,z:          \t";
        cout << solution.ee_motion_.at(0)->GetPoint(t).p().transpose() << "\t[m]" << endl;

        cout << "Contact force x,y,z:          \t";
        cout << solution.ee_force_.at(0)->GetPoint(t).p().transpose() << "\t[N]" << endl;

        bool contact = solution.phase_durations_.at(0)->IsContactPhase(t);
        std::string foot_in_contact = contact ? "yes" : "no";
        cout << "Foot in contact:              \t" + foot_in_contact << endl;

        cout << endl;

        t += 0.2;
    }
}
