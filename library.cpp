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

struct Session {
    struct Formulation {
        NlpFormulation nlp_formulation;

        // Other options.
        bool optimize_phase_durations = false;
        int max_iter = 0;
        double max_cpu_time = 0;
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

void init_gait(NlpFormulation& formulation, double duration, Lock&) {
    auto ee_count = formulation.model_.dynamic_model_->GetEECount();

    auto gait = GaitGenerator::MakeGaitGenerator(ee_count);
    gait->SetCombo(GaitGenerator::C0);

    formulation.params_.ee_phase_durations_.clear();
    formulation.params_.ee_in_contact_at_start_.clear();

    for (int id = 0; id < ee_count; ++id) {
        formulation.params_.ee_phase_durations_.push_back(gait->GetPhaseDurations(duration, id));
        formulation.params_.ee_in_contact_at_start_.push_back(gait->IsInContactAtStart(id));
    }
}

int create_session() {
    auto lock = std::unique_lock(sessions.mutex);
    auto iter = std::find(sessions.data.begin(), sessions.data.end(), nullptr);
    auto session = std::make_unique<Session>();

    // TODO: make this more flexible.
    auto& formulation = session->formulation;
    formulation.nlp_formulation.model_ = towr::RobotModel::Monoped;
    formulation.nlp_formulation.terrain_ = towr::HeightMap::MakeTerrain(towr::HeightMap::FlatID);

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

    auto& formulation_ = formulation->nlp_formulation;
    formulation_.initial_base_.lin.at(kPos) = Map<const VectorXd>(bound->initial_base_linear_position, 3);
    formulation_.initial_base_.lin.at(kVel) = Map<const VectorXd>(bound->initial_base_linear_velocity, 3);
    formulation_.initial_base_.ang.at(kPos) = Map<const VectorXd>(bound->initial_base_angular_position, 3);
    formulation_.initial_base_.ang.at(kVel) = Map<const VectorXd>(bound->initial_base_angular_velocity, 3);

    formulation_.final_base_.lin.at(kPos) = Map<const VectorXd>(bound->final_base_linear_position, 3);
    formulation_.final_base_.lin.at(kVel) = Map<const VectorXd>(bound->final_base_linear_velocity, 3);
    formulation_.final_base_.ang.at(kPos) = Map<const VectorXd>(bound->final_base_angular_position, 3);
    formulation_.final_base_.ang.at(kVel) = Map<const VectorXd>(bound->final_base_angular_velocity, 3);

    // TODO: multiple legs.
    formulation_.initial_ee_W_.clear();
    formulation_.initial_ee_W_.push_back(Map<const VectorXd>(bound->initial_ee_position, 3));

    init_gait(formulation_, bound->duration, lock);
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
    solver.Solve(problem);

    return std::move(solution);
}

void start_optimization(int session) {
    auto[solution, lock] = get_solution(session);
    *solution = std::async(std::launch::async, async_optimize, session);
}

bool update_solution(Session::Solution& solution, Lock&) {
    /*
    if (!solution.ready) {
        if (!solution.future.valid()) return false;
        if (solution.future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            solution.current = solution.future.get();
            solution.ready = true;
        } else return false;
    }
     */
    if (std::holds_alternative<std::future<SplineHolder>>(solution)) {
        auto& future = std::get<std::future<SplineHolder>>(solution);
        if (!future.valid()) return false;
        if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            solution = future.get();
        else return false;
    }
    return true;
}

bool get_solution(int session, double time, State* output) {
    auto[solution, lock] = get_solution(session);
    if (!update_solution(*solution, lock)) return false;

    using Eigen::Map;
    using Eigen::VectorXd;

    auto& current = std::get<SplineHolder>(*solution);
    {
        auto point = current.base_linear_->GetPoint(time);
        Map<VectorXd>(output->base_linear_position, 3) = point.p();
        Map<VectorXd>(output->base_linear_velocity, 3) = point.v();
    }
    {
        auto point = current.base_angular_->GetPoint(time);
        Map<VectorXd>(output->base_angular_position, 3) = point.p();
        Map<VectorXd>(output->base_angular_velocity, 3) = point.v();
    }
    Map<VectorXd>(output->ee_motion, 3) = current.ee_motion_[0]->GetPoint(time).p();
    Map<VectorXd>(output->ee_force, 3) = current.ee_force_[0]->GetPoint(time).p();
    output->contact = current.phase_durations_[0]->IsContactPhase(time);

    return true;
}

bool solution_ready(int session) {
    auto[solution, lock] = get_solution(session);
    return update_solution(*solution, lock);
}