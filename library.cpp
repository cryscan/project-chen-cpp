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
#include "monoped_gait_generator.h"


using Lock = std::unique_lock<std::mutex>;

struct Session {
    towr::NlpFormulation formulation;
    std::mutex formulation_mutex;

    struct Solution {
        union {
            std::future<towr::SplineHolder> future;
            towr::SplineHolder current;
        };
        bool ready = false;

        Solution() : future() {}

        ~Solution() {
            if (ready) current.~SplineHolder();
            else future.~future();
        }
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

void init_gait(towr::NlpFormulation& formulation,
               towr::GaitGenerator::Combos combo,
               double duration,
               Lock&) {
    // auto ee_count = formulation.model_.dynamic_model_->GetEECount();

    // auto gait = towr::GaitGenerator::MakeGaitGenerator(0);
    std::unique_ptr<towr::GaitGenerator> gait = std::make_unique<MonopedGaitGenerator>();
    gait->SetCombo(combo);

    formulation.params_.ee_phase_durations_.clear();
    formulation.params_.ee_in_contact_at_start_.clear();

    // for (int i = 0; i < ee_count; ++i) {
    formulation.params_.ee_phase_durations_.push_back(gait->GetPhaseDurations(duration, 0));
    formulation.params_.ee_in_contact_at_start_.push_back(gait->IsInContactAtStart(0));
    // }
}

int create_session() {
    auto lock = std::unique_lock(sessions.mutex);
    auto iter = std::find(sessions.data.begin(), sessions.data.end(), nullptr);
    auto session = std::make_unique<Session>();

    // TODO: make this more flexible.
    auto& formulation = session->formulation;
    formulation.model_ = towr::RobotModel::Monoped;
    formulation.terrain_ = towr::HeightMap::MakeTerrain(towr::HeightMap::FlatID);

    // init_gait(formulation, towr::GaitGenerator::C0, duration, lock);

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

void set_bound(int session, const Bound* bound) {
    auto[formulation, lock] = get_formulation(session);

    using Eigen::Map;
    using Eigen::VectorXd;
    using towr::kPos;
    using towr::kVel;
    constexpr auto C0 = towr::GaitGenerator::Combos::C0;

    formulation->initial_base_.lin.at(kPos) = Map<const VectorXd>(bound->initial_base_linear_position, 3);
    formulation->initial_base_.lin.at(kVel) = Map<const VectorXd>(bound->initial_base_linear_velocity, 3);
    formulation->initial_base_.ang.at(kPos) = Map<const VectorXd>(bound->initial_base_angular_position, 3);
    formulation->initial_base_.ang.at(kVel) = Map<const VectorXd>(bound->initial_base_angular_velocity, 3);

    formulation->final_base_.lin.at(kPos) = Map<const VectorXd>(bound->final_base_linear_position, 3);
    formulation->final_base_.lin.at(kVel) = Map<const VectorXd>(bound->final_base_linear_velocity, 3);
    formulation->final_base_.ang.at(kPos) = Map<const VectorXd>(bound->final_base_angular_position, 3);
    formulation->final_base_.ang.at(kVel) = Map<const VectorXd>(bound->final_base_angular_velocity, 3);

    {
        formulation->initial_ee_W_.clear();
        VectorXd initial_ee_position = Map<const VectorXd>(bound->initial_ee_position, 3);
        formulation->initial_ee_W_.push_back(initial_ee_position);
    }

    init_gait(*formulation, C0, bound->duration, lock);
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
    solver.SetOption("max_iter", 20);
    solver.Solve(problem);

    return std::move(solution);
}

void start_optimization(int session) {
    auto[solution, lock] = get_solution(session);
    solution->future = std::async(std::launch::async, async_optimize, session);
    solution->ready = false;
}

bool update_solution(Session::Solution& solution, Lock&) {
    if (!solution.ready) {
        if (!solution.future.valid()) return false;
        if (solution.future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            solution.current = solution.future.get();
            solution.ready = true;
        } else return false;
    }
    return true;
}

bool get_solution(int session, double time, State* output) {
    auto[solution, lock] = get_solution(session);
    if (!update_solution(*solution, lock)) return false;

    using Eigen::Map;
    using Eigen::VectorXd;

    {
        auto point = solution->current.base_linear_->GetPoint(time);
        Map<VectorXd>(output->base_linear_position, 3) = point.p();
        Map<VectorXd>(output->base_linear_velocity, 3) = point.v();
    }
    {
        auto point = solution->current.base_angular_->GetPoint(time);
        Map<VectorXd>(output->base_angular_position, 3) = point.p();
        Map<VectorXd>(output->base_angular_velocity, 3) = point.v();
    }
    Map<VectorXd>(output->ee_motion, 3) = solution->current.ee_motion_[0]->GetPoint(time).p();
    Map<VectorXd>(output->ee_force, 3) = solution->current.ee_force_[0]->GetPoint(time).p();
    output->contact = solution->current.phase_durations_[0]->IsContactPhase(time);

    return true;
}

bool solution_ready(int session) {
    auto[solution, lock] = get_solution(session);
    return update_solution(*solution, lock);
}