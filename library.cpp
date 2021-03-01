//
// Created by cryscan on 2/26/21.
//

#include <iostream>
#include <vector>
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


using Lock = std::unique_lock<std::mutex>;

struct Session {
    towr::NlpFormulation formulation;
    std::mutex formulation_mutex;

    struct Solution {
        std::future<towr::SplineHolder> future;
        towr::SplineHolder current;
        bool ready = false;
    } solution;
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

std::tuple<towr::NlpFormulation*, Lock> get_formulation(int session) {
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

void init_gait_combos(towr::NlpFormulation& formulation,
                      towr::GaitGenerator::Combos combos,
                      double duration,
                      Lock&) {
    auto ee_count = formulation.model_.dynamic_model_->GetEECount();

    auto gait = towr::GaitGenerator::MakeGaitGenerator(ee_count);
    gait->SetCombo(static_cast<towr::GaitGenerator::Combos>(combos));

    for (int i = 0; i < ee_count; ++i) {
        formulation.params_.ee_phase_durations_.push_back(gait->GetPhaseDurations(duration, i));
        formulation.params_.ee_in_contact_at_start_.push_back(gait->IsInContactAtStart(i));
    }
}

int create_session() {
    auto lock = std::unique_lock(sessions.mutex);
    auto iter = std::find(sessions.data.begin(), sessions.data.end(), nullptr);
    auto session = std::make_unique<Session>();

    // TODO: make this more flexible.
    auto& formulation = session->formulation;
    formulation.model_ = towr::RobotModel::Monoped;
    formulation.terrain_ = towr::HeightMap::MakeTerrain(towr::HeightMap::FlatID);

    init_gait_combos(formulation, towr::GaitGenerator::C0, 2.0, lock);

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

void set_initial_base_linear_position(int session, double x, double y, double z) {
    auto[formulation, lock] = get_formulation(session);
    formulation->initial_base_.lin.at(towr::kPos) << x, y, z;
}

void set_initial_base_linear_velocity(int session, double x, double y, double z) {
    auto[formulation, lock] = get_formulation(session);
    formulation->initial_base_.lin.at(towr::kVel) << x, y, z;
}

void set_initial_base_angular_position(int session, double x, double y, double z) {
    auto[formulation, lock] = get_formulation(session);
    formulation->initial_base_.ang.at(towr::kPos) << x, y, z;
}

void set_initial_base_angular_velocity(int session, double x, double y, double z) {
    auto[formulation, lock] = get_formulation(session);
    formulation->initial_base_.ang.at(towr::kVel) << x, y, z;
}

void set_final_base_linear_position(int session, double x, double y, double z) {
    auto[formulation, lock] = get_formulation(session);
    formulation->final_base_.lin.at(towr::kPos) << x, y, z;
}

void set_final_base_linear_velocity(int session, double x, double y, double z) {
    auto[formulation, lock] = get_formulation(session);
    formulation->final_base_.lin.at(towr::kVel) << x, y, z;
}

void set_final_base_angular_position(int session, double x, double y, double z) {
    auto[formulation, lock] = get_formulation(session);
    formulation->final_base_.ang.at(towr::kPos) << x, y, z;
}

void set_final_base_angular_velocity(int session, double x, double y, double z) {
    auto[formulation, lock] = get_formulation(session);
    formulation->final_base_.ang.at(towr::kVel) << x, y, z;
}

void set_initial_end_effector_position(int session, int id, double x, double y) {
    auto[formulation, lock] = get_formulation(session);
    auto z = formulation->terrain_->GetHeight(x, y);
    formulation->initial_ee_W_[id] << x, y, z;
}

towr::SplineHolder async_optimize(int session) {
    auto[formulation, lock] = get_formulation(session);

    ifopt::Problem problem;
    towr::SplineHolder solution;

    for (auto& vars: formulation->GetVariableSets(solution))
        problem.AddVariableSet(vars);
    for (auto& constrains: formulation->GetConstraints(solution))
        problem.AddConstraintSet(constrains);
    for (auto& costs: formulation->GetCosts())
        problem.AddCostSet(costs);

    ifopt::IpoptSolver solver;
    solver.SetOption("jacobian_approximation", "exact");
    solver.Solve(problem);

    return std::move(solution);
}

void start_optimization(int session) {
    auto[solution, lock] = get_solution(session);
    solution->future = std::async(async_optimize, session);
    solution->ready = false;
}

bool update_solution(Session::Solution& solution, Lock&) {
    if (!solution.ready) {
        if (solution.future.valid()) {
            solution.current = solution.future.get();
            solution.ready = true;
        } else return false;
    }
    return true;
}

bool get_base_linear_position(int session, double time, double* output) {
    auto[solution, lock] = get_solution(session);
    if (!update_solution(*solution, lock)) return false;

    auto point = solution->current.base_linear_->GetPoint(time).p();
    Eigen::Map<Eigen::VectorXd>(output, point.rows(), 1) = point;
    return true;
}

bool get_base_angular_position(int session, double time, double* output) {
    auto[solution, lock] = get_solution(session);
    if (!update_solution(*solution, lock)) return false;

    auto point = solution->current.base_angular_->GetPoint(time).p();
    Eigen::Map<Eigen::VectorXd>(output, point.rows(), 1) = point;
    return true;
}

bool get_end_effector_position(int session, int id, double time, double* output) {
    auto[solution, lock] = get_solution(session);
    if (!update_solution(*solution, lock)) return false;

    auto point = solution->current.ee_motion_.at(id)->GetPoint(time).p();
    Eigen::Map<Eigen::VectorXd>(output, point.rows(), 1) = point;
    return true;
}

bool get_end_effector_force(int session, int id, double time, double* output) {
    auto[solution, lock] = get_solution(session);
    if (!update_solution(*solution, lock)) return false;

    auto point = solution->current.ee_force_.at(id)->GetPoint(time).p();
    Eigen::Map<Eigen::VectorXd>(output, point.rows(), 1) = point;
    return true;
}

bool get_end_effector_contact(int session, int id, double time, bool* output) {
    auto[solution, lock] = get_solution(session);
    if (!update_solution(*solution, lock)) return false;

    *output = solution->current.phase_durations_.at(id)->IsContactPhase(time);
    return true;
}