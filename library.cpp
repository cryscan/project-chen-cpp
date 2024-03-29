//
// Created by cryscan on 2/26/21.
//

#include <vector>
#include <bitset>
#include <variant>
#include <memory>
#include <future>
#include <mutex>
#include <stdexcept>
#include <iostream>

#include <Eigen/Core>

#include <towr/nlp_formulation.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/initialization/gait_generator.h>
#include <ifopt/ipopt_solver.h>

#include "library.h"
#include "utils.h"
#include "path_constraint.h"
#include "terrain.h"


using towr::NlpFormulation;
using towr::SplineHolder;
using towr::GaitGenerator;

using Lock = std::unique_lock<std::mutex>;

Terrain::Ptr get_terrain(int terrain);


/* Sessions */

static std::mutex solver_mutex;

struct Formulation {
    using Ptr = std::shared_ptr<Formulation>;
    using PathPointVec = std::vector<PathPoint>;

    NlpFormulation nlp_formulation;
    double duration = 0;

    PathPointVec path_points;
    std::vector<GaitGenerator::Gaits> gaits;

    double max_cpu_time = 0;
    int max_iter = 0;

    std::mutex mutex;
};

struct Solution {
    using Ptr = std::shared_ptr<Solution>;

    std::variant<std::future<SplineHolder>, SplineHolder> variant;
    std::mutex mutex;
};

struct {
    std::vector<Formulation::Ptr> formulations;
    std::vector<Solution::Ptr> solutions;
    std::mutex mutex;
} static sessions;


void check_session(int session, Lock&) {
    if (session >= sessions.formulations.size() || sessions.formulations.at(session) == nullptr) {
        std::string error = "Invalid Session Id " + std::to_string(session);
        throw std::runtime_error(error);
    }
}

std::tuple<Formulation::Ptr, Lock> get_formulation(int session) {
    auto lock = Lock(sessions.mutex);
    check_session(session, lock);

    auto formulation = sessions.formulations.at(session);
    Lock(formulation->mutex).swap(lock);

    return std::make_tuple(formulation, std::move(lock));
}

std::tuple<Solution::Ptr, Lock> get_solution(int session) {
    auto lock = Lock(sessions.mutex);
    check_session(session, lock);

    auto solution = sessions.solutions.at(session);
    Lock(solution->mutex).swap(lock);

    return std::make_tuple(solution, std::move(lock));
}

void init_gait(Formulation& formulation, Lock&) {
    auto& f = formulation.nlp_formulation;
    auto ee_count = f.model_.dynamic_model_->GetEECount();

    auto generator = GaitGenerator::MakeGaitGenerator(ee_count);
    if (formulation.gaits.empty()) generator->SetCombo(towr::GaitGenerator::C0);
    else generator->SetGaits(formulation.gaits);

    f.params_.ee_phase_durations_.clear();
    f.params_.ee_in_contact_at_start_.clear();

    for (int id = 0; id < ee_count; ++id) {
        f.params_.ee_phase_durations_.push_back(generator->GetPhaseDurations(formulation.duration, id));
        f.params_.ee_in_contact_at_start_.push_back(generator->IsInContactAtStart(id));
    }
}

int create_session(int model) {
    auto lock = std::unique_lock(sessions.mutex);
    auto iter = std::find(sessions.formulations.begin(), sessions.formulations.end(), nullptr);

    auto formulation = std::make_shared<Formulation>();
    formulation->nlp_formulation.model_ = static_cast<towr::RobotModel::Robot>(model);
    formulation->nlp_formulation.terrain_ = towr::HeightMap::MakeTerrain(towr::HeightMap::FlatID);

    auto solution = std::make_shared<Solution>();
    auto index = iter - sessions.formulations.begin();

    if (iter != sessions.formulations.end()) {
        *iter = std::move(formulation);
        sessions.solutions.at(index) = solution;
    } else {
        sessions.formulations.push_back(std::move(formulation));
        sessions.solutions.push_back(std::move(solution));
    }

    return index;
}

void end_session(int session) {
    auto lock = std::unique_lock(sessions.mutex);
    check_session(session, lock);

    sessions.formulations.at(session) = nullptr;
    sessions.solutions.at(session) = nullptr;
}

void get_model(int session, Model* model) {
    auto[formulation, lock] = get_formulation(session);
    auto& f = formulation->nlp_formulation;

    using Eigen::Vector3d;
    using Eigen::Map;

    Map<Vector3d>(model->max_deviation, 3) = f.model_.kinematic_model_->GetMaximumDeviationFromNominal();

    auto nominal_stance = f.model_.kinematic_model_->GetNominalStanceInBase();
    for (int id = 0; id < f.model_.kinematic_model_->GetNumberOfEndeffectors(); ++id)
        Map<Vector3d>(model->nominal_stance[id], 3) = nominal_stance.at(id);

    model->mass = f.model_.dynamic_model_->m();
    model->ee_count = f.model_.dynamic_model_->GetEECount();
}

int get_ee_count(int session) {
    auto[formulation, lock] = get_formulation(session);
    return formulation->nlp_formulation.model_.dynamic_model_->GetEECount();
}

void set_terrain(int session, int terrain) {
    auto[formulation, lock]  = get_formulation(session);
    formulation->nlp_formulation.terrain_ = get_terrain(terrain);
}

void set_params(int session, const Parameters* parameters) {
    auto[formulation, lock] = get_formulation(session);
    auto& f = formulation->nlp_formulation;

    using Eigen::Map;
    using Eigen::Vector3d;
    using towr::kPos;
    using towr::kVel;

    f.initial_base_.lin.at(kPos) = Map<const Vector3d>(parameters->initial_base_lin_pos, 3);
    f.initial_base_.lin.at(kVel) = Map<const Vector3d>(parameters->initial_base_lin_vel, 3);
    f.initial_base_.ang.at(kPos) = Map<const Vector3d>(parameters->initial_base_ang_pos, 3);
    f.initial_base_.ang.at(kVel) = Map<const Vector3d>(parameters->initial_base_ang_vel, 3);

    f.final_base_.lin.at(kPos) = Map<const Vector3d>(parameters->final_base_lin_pos, 3);
    f.final_base_.lin.at(kVel) = Map<const Vector3d>(parameters->final_base_lin_vel, 3);
    f.final_base_.ang.at(kPos) = Map<const Vector3d>(parameters->final_base_ang_pos, 3);
    f.final_base_.ang.at(kVel) = Map<const Vector3d>(parameters->final_base_ang_vel, 3);

    f.initial_ee_W_.clear();
    for (int id = 0; id < f.model_.dynamic_model_->GetEECount(); ++id)
        f.initial_ee_W_.push_back(Map<const Vector3d>(parameters->initial_ee_pos[id], 3));

    f.params_.bounds_final_lin_pos_ = convert_bound_dims(parameters->bounds_final_lin_pos);
    f.params_.bounds_final_lin_vel_ = convert_bound_dims(parameters->bounds_final_lin_vel);
    f.params_.bounds_final_ang_pos_ = convert_bound_dims(parameters->bounds_final_ang_pos);
    f.params_.bounds_final_ang_vel_ = convert_bound_dims(parameters->bounds_final_ang_vel);
}

void set_options(int session, const Options* options) {
    auto[formulation, lock] = get_formulation(session);
    formulation->max_cpu_time = options->max_cpu_time;
    formulation->max_iter = options->max_iter;
    if (options->optimize_phase_durations) formulation->nlp_formulation.params_.OptimizePhaseDurations();
}

void set_duration(int session, double duration) {
    auto[formulation, lock] = get_formulation(session);
    formulation->duration = duration;
}

void push_path_point(int session, const PathPoint* path_point) {
    auto[formulation, lock] = get_formulation(session);
    formulation->path_points.push_back(*path_point);
}

void push_gait(int session, int gait) {
    auto[formulation, lock] = get_formulation(session);
    formulation->gaits.push_back(static_cast<GaitGenerator::Gaits>(gait));
}

auto build_path_constraint(const Formulation::PathPointVec& path_points,
                           const SplineHolder& spline_holder) {
    using Eigen::VectorXd;
    using Eigen::Map;

    std::vector<double> dts;
    std::vector<VectorXd> path_linear, path_angular;
    std::vector<uint8_t> path_bounds;

    for (auto& path_point: path_points) {
        dts.push_back(path_point.time);
        path_linear.emplace_back(Map<const VectorXd>(path_point.linear, 3));
        path_angular.emplace_back(Map<const VectorXd>(path_point.angular, 3));
        path_bounds.push_back(path_point.bounds);
    }

    return std::make_shared<PathConstraint>(dts, path_linear, path_angular, path_bounds, spline_holder);
}

SplineHolder async_optimize(int session) {
    using ifopt::ConstraintSet;
    using ifopt::Problem;
    using ifopt::IpoptSolver;

    auto[formulation, lock] = get_formulation(session);

    init_gait(*formulation, lock);

    Problem problem;
    SplineHolder solution;

    auto& formulation_ = formulation->nlp_formulation;
    for (auto& c: formulation_.GetVariableSets(solution))
        problem.AddVariableSet(c);
    for (auto& c: formulation_.GetConstraints(solution))
        problem.AddConstraintSet(c);
    for (auto& c: formulation_.GetCosts())
        problem.AddCostSet(c);

    auto path_constraint = build_path_constraint(formulation->path_points, solution);
    problem.AddConstraintSet(path_constraint);

    IpoptSolver solver;
    solver.SetOption("jacobian_approximation", "exact");
    if (formulation->max_iter > 0) solver.SetOption("max_iter", formulation->max_iter);
    if (formulation->max_cpu_time > 0) solver.SetOption("max_cpu_time", formulation->max_cpu_time);

    Lock(solver_mutex).swap(lock);
    solver.Solve(problem);

    return std::move(solution);
}

void start_optimization(int session) {
    auto[solution, lock] = get_solution(session);
    solution->variant = std::async(std::launch::async, async_optimize, session);
}

bool update_solution(Solution& solution, Lock&) {
    if (auto future = std::get_if<std::future<SplineHolder>>(&solution.variant)) {
        if (!future->valid()) return false;
        if (future->wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            solution.variant = future->get();
        else return false;
    }
    return true;
}

bool solution_ready(int session) {
    auto[solution, lock] = get_solution(session);
    return update_solution(*solution, lock);
}

bool get_solution_state(int session, double time, State* state) {
    auto[solution, lock] = get_solution(session);
    if (!update_solution(*solution, lock)) return false;
    auto& current = std::get<SplineHolder>(solution->variant);

    using Eigen::Map;
    using Eigen::Vector3d;

    auto point = current.base_linear_->GetPoint(time);
    Map<Vector3d>(state->base_lin_pos, 3) = point.p();
    Map<Vector3d>(state->base_lin_vel, 3) = point.v();

    point = current.base_angular_->GetPoint(time);
    Map<Vector3d>(state->base_ang_pos, 3) = point.p();
    Map<Vector3d>(state->base_ang_vel, 3) = point.v();

    for (int id = 0; id < current.ee_motion_.size(); ++id)
        Map<Vector3d>(state->ee_motions[id], 3) = current.ee_motion_.at(id)->GetPoint(time).p();

    for (int id = 0; id < current.ee_force_.size(); ++id)
        Map<Vector3d>(state->ee_forces[id], 3) = current.ee_force_.at(id)->GetPoint(time).p();

    for (int id = 0; id < current.phase_durations_.size(); ++id)
        state->contacts[id] = current.phase_durations_.at(id)->IsContactPhase(time);

    return true;
}


/* Terrains */

struct {
    std::vector<Terrain::Ptr> terrains;
    std::mutex mutex;
} static terrains;


void check_terrain(int terrain, Lock&) {
    if (terrain >= terrains.terrains.size() || terrains.terrains.at(terrain) == nullptr) {
        std::string error = "Invalid Terrain Id " + std::to_string(terrain);
        throw std::runtime_error(error);
    }
}

Terrain::Ptr get_terrain(int terrain) {
    auto lock = Lock(terrains.mutex);
    check_terrain(terrain, lock);
    return terrains.terrains.at(terrain);
}

int create_terrain(double pos_x, double pos_y, double pos_z, uint x, uint y, double unit_size) {
    using Eigen::Vector3d;

    auto lock = Lock(terrains.mutex);
    auto iter = std::find(terrains.terrains.begin(), terrains.terrains.end(), nullptr);

    Vector3d pos(pos_x, pos_y, pos_z);
    auto terrain = std::make_shared<Terrain>(pos, x, y, unit_size);

    auto index = iter - terrains.terrains.begin();
    if (iter != terrains.terrains.end()) *iter = std::move(terrain);
    else terrains.terrains.push_back(std::move(terrain));

    return index;
}

void end_terrain(int terrain) {
    auto lock = Lock(terrains.mutex);
    check_terrain(terrain, lock);
    terrains.terrains.at(terrain) = nullptr;
}

void set_height(int terrain, uint x, uint y, double height) {
    get_terrain(terrain)->SetHeight(x, y, height);
}

double get_height(int terrain, double x, double y) {
    return get_terrain(terrain)->GetHeight(x, y);
}

void get_height_derivatives(int terrain, double x, double y, double* dx, double* dy) {
    *dx = get_terrain(terrain)->GetHeightDerivWrtX(x, y);
    *dy = get_terrain(terrain)->GetHeightDerivWrtY(x, y);
}