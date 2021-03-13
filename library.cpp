//
// Created by cryscan on 2/26/21.
//

#include <iostream>
#include <vector>
#include <variant>
#include <memory>
#include <future>
#include <mutex>
#include <exception>

#include <Eigen/Core>

#include <towr/nlp_formulation.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/initialization/gait_generator.h>
#include <ifopt/ipopt_solver.h>

#include "library.h"


using towr::NlpFormulation;
using towr::SplineHolder;
using towr::GaitGenerator;

using Lock = std::unique_lock<std::mutex>;

std::mutex solver_mutex;

struct Session {
    struct Formulation {
        NlpFormulation nlp_formulation;

        // Options
        double max_cpu_time = 0;
        int max_iter = 0;
    } formulation;
    std::mutex formulation_mutex;

    using Solution = std::variant<std::future<SplineHolder>, SplineHolder>;
    Solution solution = std::future<SplineHolder>();
    std::mutex solution_mutex;
};

struct InvalidSessionError : std::exception {
    explicit InvalidSessionError(int session) : error() {
        sprintf(error, "Invalid session %d", session);
    }

    [[nodiscard]] const char* what() const noexcept override {
        return error;
    }

private:
    char error[32];
};

struct {
    std::vector<std::unique_ptr<Session>> data;
    std::mutex mutex;
} static sessions;

void check_session(int session, Lock&) {
    if (session >= sessions.data.size() || sessions.data[session] == nullptr)
        throw InvalidSessionError(session);
}

std::tuple<Session::Formulation*, Lock> get_formulation(int session) {
    auto lock = std::unique_lock(sessions.mutex);
    try { check_session(session, lock); }
    catch (InvalidSessionError& error) { std::cerr << error.what() << std::endl; }

    auto lock_ = std::move(lock);
    auto formulation = &sessions.data[session]->formulation;
    lock = std::unique_lock(sessions.data[session]->formulation_mutex);

    return std::make_tuple(formulation, std::move(lock));
}

std::tuple<Session::Solution*, Lock> get_solution(int session) {
    auto lock = std::unique_lock(sessions.mutex);
    try { check_session(session, lock); }
    catch (InvalidSessionError& error) { std::cerr << error.what() << std::endl; }

    auto lock_ = std::move(lock);
    auto solution = &sessions.data[session]->solution;
    lock = std::unique_lock(sessions.data[session]->solution_mutex);

    return std::make_tuple(solution, std::move(lock));
}

void init_gait(Session::Formulation& formulation, double duration, Lock&) {
    auto& f = formulation.nlp_formulation;
    auto ee_count = f.model_.dynamic_model_->GetEECount();

    auto gait = GaitGenerator::MakeGaitGenerator(ee_count);
    gait->SetCombo(GaitGenerator::C0);

    f.params_.ee_phase_durations_.clear();
    f.params_.ee_in_contact_at_start_.clear();

    for (int id = 0; id < ee_count; ++id) {
        f.params_.ee_phase_durations_.push_back(gait->GetPhaseDurations(duration, id));
        f.params_.ee_in_contact_at_start_.push_back(gait->IsInContactAtStart(id));
    }
}

int create_session(int model) {
    auto lock = std::unique_lock(sessions.mutex);
    auto iter = std::find(sessions.data.begin(), sessions.data.end(), nullptr);
    auto session = std::make_unique<Session>();

    auto& formulation = session->formulation;
    formulation.nlp_formulation.model_ = static_cast<towr::RobotModel::Robot>(model);
    formulation.nlp_formulation.terrain_ = towr::HeightMap::MakeTerrain(towr::HeightMap::FlatID);

    if (iter != sessions.data.end()) {
        *iter = std::move(session);
        return iter - sessions.data.begin();
    } else {
        auto index = sessions.data.size();
        sessions.data.push_back(std::move(session));
        return index;
    }
}

void end_session(int session) {
    auto lock = std::unique_lock(sessions.mutex);
    try { check_session(session, lock); }
    catch (InvalidSessionError& error) {
        std::cerr << error.what() << std::endl;
        return;
    }
    sessions.data[session] = nullptr;
}

void get_robot_model(int session, RobotModel* robot_model) {
    auto[formulation, lock] = get_formulation(session);
    auto& f = formulation->nlp_formulation;

    using Eigen::Vector3d;
    using Eigen::Map;

    Map<Vector3d>(robot_model->max_deviation, 3) = f.model_.kinematic_model_->GetMaximumDeviationFromNominal();

    auto nominal_stance = f.model_.kinematic_model_->GetNominalStanceInBase();
    for (int id = 0; id < f.model_.kinematic_model_->GetNumberOfEndeffectors(); ++id)
        Map<Vector3d>(robot_model->nominal_stance[id], 3) = nominal_stance[id];

    robot_model->mass = f.model_.dynamic_model_->m();
    robot_model->ee_count = f.model_.dynamic_model_->GetEECount();
}

int get_ee_count(int session) {
    auto[formulation, lock] = get_formulation(session);
    return formulation->nlp_formulation.model_.dynamic_model_->GetEECount();
}

void set_bound(int session, const Bound* bound) {
    auto[formulation, lock] = get_formulation(session);
    auto& f = formulation->nlp_formulation;

    using Eigen::Map;
    using Eigen::Vector3d;
    using towr::kPos;
    using towr::kVel;

    f.initial_base_.lin.at(kPos) = Map<const Vector3d>(bound->initial_base_linear_position, 3);
    f.initial_base_.lin.at(kVel) = Map<const Vector3d>(bound->initial_base_linear_velocity, 3);
    f.initial_base_.ang.at(kPos) = Map<const Vector3d>(bound->initial_base_angular_position, 3);
    f.initial_base_.ang.at(kVel) = Map<const Vector3d>(bound->initial_base_angular_velocity, 3);

    f.final_base_.lin.at(kPos) = Map<const Vector3d>(bound->final_base_linear_position, 3);
    f.final_base_.lin.at(kVel) = Map<const Vector3d>(bound->final_base_linear_velocity, 3);
    f.final_base_.ang.at(kPos) = Map<const Vector3d>(bound->final_base_angular_position, 3);
    f.final_base_.ang.at(kVel) = Map<const Vector3d>(bound->final_base_angular_velocity, 3);

    f.initial_ee_W_.clear();
    for (int id = 0; id < f.model_.dynamic_model_->GetEECount(); ++id)
        f.initial_ee_W_.push_back(Map<const Vector3d>(bound->initial_ee_positions[id], 3));

    init_gait(*formulation, bound->duration, lock);
}

void set_option(int session, const Option* option) {
    auto[formulation, lock] = get_formulation(session);
    formulation->max_cpu_time = option->max_cpu_time;
    formulation->max_iter = option->max_iter;
    if (option->optimize_phase_durations) formulation->nlp_formulation.params_.OptimizePhaseDurations();
}

SplineHolder async_optimize(int session) {
    auto[formulation, lock] = get_formulation(session);

    ifopt::Problem problem;
    SplineHolder solution;

    auto& formulation_ = formulation->nlp_formulation;
    for (auto& c: formulation_.GetVariableSets(solution))
        problem.AddVariableSet(c);
    for (auto& c: formulation_.GetConstraints(solution))
        problem.AddConstraintSet(c);
    for (auto& c: formulation_.GetCosts())
        problem.AddCostSet(c);

    ifopt::IpoptSolver solver;
    solver.SetOption("jacobian_approximation", "exact");
    if (formulation->max_iter > 0) solver.SetOption("max_iter", formulation->max_iter);
    if (formulation->max_cpu_time > 0) solver.SetOption("max_cpu_time", formulation->max_cpu_time);

    lock = Lock(solver_mutex);
    solver.Solve(problem);

    return std::move(solution);
}

void start_optimization(int session) {
    auto[solution, lock] = get_solution(session);
    *solution = std::async(std::launch::async, async_optimize, session);
}

bool update_solution(Session::Solution* solution, Lock&) {
    if (auto future = std::get_if<std::future<SplineHolder>>(solution)) {
        if (!future->valid()) return false;
        if (future->wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            *solution = future->get();
        else return false;
    }
    return true;
}

bool solution_ready(int session) {
    auto[solution, lock] = get_solution(session);

    if (auto future = std::get_if<std::future<SplineHolder>>(solution))
        if (!future->valid()) return true;

    return update_solution(solution, lock);
}

bool get_solution_state(int session, double time, State* state) {
    auto[solution, lock] = get_solution(session);
    if (!update_solution(solution, lock)) return false;
    auto& current = std::get<SplineHolder>(*solution);

    using Eigen::Map;
    using Eigen::Vector3d;

    auto point = current.base_linear_->GetPoint(time);
    Map<Vector3d>(state->base_linear_position, 3) = point.p();
    Map<Vector3d>(state->base_linear_velocity, 3) = point.v();

    point = current.base_angular_->GetPoint(time);
    Map<Vector3d>(state->base_angular_position, 3) = point.p();
    Map<Vector3d>(state->base_angular_velocity, 3) = point.v();

    for (int id = 0; id < current.ee_motion_.size(); ++id)
        Map<Vector3d>(state->ee_motions[id], 3) = current.ee_motion_[id]->GetPoint(time).p();

    for (int id = 0; id < current.ee_force_.size(); ++id)
        Map<Vector3d>(state->ee_forces[id], 3) = current.ee_force_[id]->GetPoint(time).p();

    for (int id = 0; id < current.phase_durations_.size(); ++id)
        state->contacts[id] = current.phase_durations_[id]->IsContactPhase(time);

    return true;
}