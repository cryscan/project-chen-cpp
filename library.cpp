//
// Created by cryscan on 2/26/21.
//

#include <iostream>
#include <vector>
#include <memory>
#include <future>
#include <mutex>
#include <algorithm>
#include <stdexcept>

#include <towr/nlp_formulation.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <ifopt/ipopt_solver.h>

#include "library.h"


using Lock = std::unique_lock<std::mutex>;

struct SessionInfo {
    towr::NlpFormulation formulation;
    std::future<towr::SplineHolder> solution;

    std::mutex mutex;
};

struct SessionError : std::runtime_error {
    explicit SessionError(const std::string& str) : std::runtime_error(str) {}
};

struct {
    std::vector<std::unique_ptr<SessionInfo>> data;
    std::mutex mutex;
} static sessions;

void check_session(int session, Lock&) {
    if (session >= sessions.data.size() || sessions.data[session] == nullptr) {
        char error[32] = {0};
        sprintf(error, "Invalid session %d", session);
        throw SessionError(error);
    }
}

std::tuple<SessionInfo*, Lock> get_session_info(int session) {
    auto lock = std::unique_lock(sessions.mutex);
    try { check_session(session, lock); }
    catch (SessionError& error) { std::cerr << error.what() << std::endl; }

    auto lock_ = std::move(lock);
    auto info = sessions.data[session].get();
    lock = std::unique_lock(info->mutex);

    return std::make_tuple(info, std::move(lock));
}

int create_session() {
    auto lock = std::unique_lock(sessions.mutex);
    auto iter = std::find(sessions.data.begin(), sessions.data.end(), nullptr);
    auto session = std::make_unique<SessionInfo>();

    // TODO: make this more flexible.
    auto& formulation = session->formulation;
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
    catch (SessionError& error) {
        std::cerr << error.what() << std::endl;
        return;
    }
    sessions.data[session] = nullptr;
}

void set_initial_base_linear_position(int session, double x, double y, double z) {
    auto[info, lock] = get_session_info(session);
    info->formulation.initial_base_.lin.at(towr::kPos) << x, y, z;
}

void set_initial_base_linear_velocity(int session, double x, double y, double z) {
    auto[info, lock] = get_session_info(session);
    info->formulation.initial_base_.lin.at(towr::kVel) << x, y, z;
}

void set_initial_base_angular_position(int session, double x, double y, double z) {
    auto[info, lock] = get_session_info(session);
    info->formulation.initial_base_.ang.at(towr::kPos) << x, y, z;
}

void set_initial_base_angular_velocity(int session, double x, double y, double z) {
    auto[info, lock] = get_session_info(session);
    info->formulation.initial_base_.ang.at(towr::kVel) << x, y, z;
}

void set_final_base_linear_position(int session, double x, double y, double z) {
    auto[info, lock] = get_session_info(session);
    info->formulation.final_base_.lin.at(towr::kPos) << x, y, z;
}

void set_final_base_linear_velocity(int session, double x, double y, double z) {
    auto[info, lock] = get_session_info(session);
    info->formulation.final_base_.lin.at(towr::kVel) << x, y, z;
}

void set_final_base_angular_position(int session, double x, double y, double z) {
    auto[info, lock] = get_session_info(session);
    info->formulation.final_base_.ang.at(towr::kPos) << x, y, z;
}

void set_final_base_angular_velocity(int session, double x, double y, double z) {
    auto[info, lock] = get_session_info(session);
    info->formulation.final_base_.ang.at(towr::kVel) << x, y, z;
}

void set_initial_end_effector_position(int session, int id, double x, double y) {
    auto[info, lock] = get_session_info(session);
    auto& formulation = info->formulation;
    auto z = formulation.terrain_->GetHeight(x, y);
    formulation.initial_ee_W_[id] << x, y, z;
}