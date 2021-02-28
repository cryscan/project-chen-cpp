//
// Created by cryscan on 2/26/21.
//

#include <iostream>
#include <vector>
#include <memory>
#include <future>
#include <mutex>
#include <exception>

#include <towr/nlp_formulation.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <ifopt/ipopt_solver.h>

#include "library.h"


using Lock = std::unique_lock<std::mutex>;

struct Formulation {
    towr::NlpFormulation formulation;
    std::mutex mutex;
};

struct Solution {
    std::future<towr::SplineHolder> solution;
    std::mutex mutex;
};

struct Session {
    Formulation formulation;
    Solution solution;
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

std::tuple<Formulation*, Lock> get_session_formulation(int session) {
    auto lock = std::unique_lock(sessions.mutex);
    try { check_session(session, lock); }
    catch (InvalidSessionError& error) { std::cerr << error.what() << std::endl; }

    auto lock_ = std::move(lock);
    auto formulation = &sessions.data[session]->formulation;
    lock = std::unique_lock(formulation->mutex);

    return std::make_tuple(formulation, std::move(lock));
}

std::tuple<Solution*, Lock> get_session_solution(int session) {
    auto lock = std::unique_lock(sessions.mutex);
    try { check_session(session, lock); }
    catch (InvalidSessionError& error) { std::cerr << error.what() << std::endl; }

    auto lock_ = std::move(lock);
    auto solution = &sessions.data[session]->solution;
    lock = std::unique_lock(solution->mutex);

    return std::make_tuple(solution, std::move(lock));
}

int create_session() {
    auto lock = std::unique_lock(sessions.mutex);
    auto iter = std::find(sessions.data.begin(), sessions.data.end(), nullptr);
    auto session = std::make_unique<Session>();

    // TODO: make this more flexible.
    auto& formulation = session->formulation.formulation;
    formulation.model_ = towr::RobotModel::Monoped;
    formulation.terrain_ = towr::HeightMap::MakeTerrain(towr::HeightMap::FlatID);

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
    auto[formulation, lock] = get_session_formulation(session);
    formulation->formulation.initial_base_.lin.at(towr::kPos) << x, y, z;
}

void set_initial_base_linear_velocity(int session, double x, double y, double z) {
    auto[formulation, lock] = get_session_formulation(session);
    formulation->formulation.initial_base_.lin.at(towr::kVel) << x, y, z;
}

void set_initial_base_angular_position(int session, double x, double y, double z) {
    auto[formulation, lock] = get_session_formulation(session);
    formulation->formulation.initial_base_.ang.at(towr::kPos) << x, y, z;
}

void set_initial_base_angular_velocity(int session, double x, double y, double z) {
    auto[formulation, lock] = get_session_formulation(session);
    formulation->formulation.initial_base_.ang.at(towr::kVel) << x, y, z;
}

void set_final_base_linear_position(int session, double x, double y, double z) {
    auto[formulation, lock] = get_session_formulation(session);
    formulation->formulation.final_base_.lin.at(towr::kPos) << x, y, z;
}

void set_final_base_linear_velocity(int session, double x, double y, double z) {
    auto[formulation, lock] = get_session_formulation(session);
    formulation->formulation.final_base_.lin.at(towr::kVel) << x, y, z;
}

void set_final_base_angular_position(int session, double x, double y, double z) {
    auto[formulation, lock] = get_session_formulation(session);
    formulation->formulation.final_base_.ang.at(towr::kPos) << x, y, z;
}

void set_final_base_angular_velocity(int session, double x, double y, double z) {
    auto[formulation, lock] = get_session_formulation(session);
    formulation->formulation.final_base_.ang.at(towr::kVel) << x, y, z;
}

void set_initial_end_effector_position(int session, int id, double x, double y) {
    auto[formulation, lock] = get_session_formulation(session);
    auto z = formulation->formulation.terrain_->GetHeight(x, y);
    formulation->formulation.initial_ee_W_[id] << x, y, z;
}

towr::SplineHolder async_optimize(int session) {
    auto[formulation, lock] = get_session_formulation(session);

    ifopt::Problem problem;
    towr::SplineHolder solution;

    for (auto& vars: formulation->formulation.GetVariableSets(solution))
        problem.AddVariableSet(vars);
    for (auto& constrains: formulation->formulation.GetConstraints(solution))
        problem.AddConstraintSet(constrains);
    for (auto& costs: formulation->formulation.GetCosts())
        problem.AddCostSet(costs);

    ifopt::IpoptSolver solver;
    solver.SetOption("jacobian_approximation", "exact");
    solver.Solve(problem);

    return std::move(solution);
}

void start_optimization(int session) {
    auto[solution, lock] = get_session_solution(session);
    solution->solution = std::async(async_optimize, session);
}