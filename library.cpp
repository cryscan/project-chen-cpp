//
// Created by cryscan on 2/26/21.
//

#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <algorithm>

#include <towr/nlp_formulation.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <ifopt/ipopt_solver.h>

#include "library.h"


struct SessionInfo {
    towr::NlpFormulation formulation;
    towr::SplineHolder solution;

    std::mutex mutex;
};

struct {
    std::vector<std::unique_ptr<SessionInfo>> data;
    std::mutex mutex;
} static sessions;

int create_session() {
    auto lock = std::unique_lock(sessions.mutex);
    auto iter = std::find(sessions.data.begin(), sessions.data.end(), nullptr);
    auto session = std::make_unique<SessionInfo>();

    // TODO: make this more flexible.
    auto& formation = session->formulation;
    formation.model_ = towr::RobotModel::Monoped;
    formation.terrain_ = towr::HeightMap::MakeTerrain(towr::HeightMap::FlatID);

    if (iter != sessions.data.end()) {
        *iter = std::move(session);
        return iter - sessions.data.begin();
    } else {
        auto index = sessions.data.size();
        sessions.data.push_back(std::move(session));
        return index;
    }
}

bool check_session(int session, std::unique_lock<std::mutex>&) {
    if (session >= sessions.data.size() || sessions.data[session] == nullptr) {
        std::cerr << "Error: invalid session id " << session << std::endl;
        return false;
    }
    return true;
}

void end_session(int session) {
    auto lock = std::unique_lock(sessions.mutex);
    if (!check_session(session, lock)) return;
    sessions.data[session] = nullptr;
}

void set_initial_base_linear_position(int session, double x, double y, double z) {
    auto lock = std::unique_lock(sessions.mutex);
    if (!check_session(session, lock)) return;

    auto& formation = sessions.data[session]->formulation;
    formation.initial_base_.lin.at(towr::kPos) << x, y, z;
}

void set_initial_base_linear_velocity(int session, double x, double y, double z) {
    auto lock = std::unique_lock(sessions.mutex);
    if (!check_session(session, lock)) return;

    auto& formation = sessions.data[session]->formulation;
    formation.initial_base_.lin.at(towr::kVel) << x, y, z;
}

void set_initial_base_angular_position(int session, double x, double y, double z) {
    auto lock = std::unique_lock(sessions.mutex);
    if (!check_session(session, lock)) return;

    auto& formation = sessions.data[session]->formulation;
    formation.initial_base_.ang.at(towr::kPos) << x, y, z;
}

void set_initial_base_angular_velocity(int session, double x, double y, double z) {
    auto lock = std::unique_lock(sessions.mutex);
    if (!check_session(session, lock)) return;

    auto& formation = sessions.data[session]->formulation;
    formation.initial_base_.ang.at(towr::kVel) << x, y, z;
}

void set_final_base_linear_position(int session, double x, double y, double z) {
    auto lock = std::unique_lock(sessions.mutex);
    if (!check_session(session, lock)) return;

    auto& formation = sessions.data[session]->formulation;
    formation.final_base_.lin.at(towr::kPos) << x, y, z;
}

void set_final_base_linear_velocity(int session, double x, double y, double z) {
    auto lock = std::unique_lock(sessions.mutex);
    if (!check_session(session, lock)) return;

    auto& formation = sessions.data[session]->formulation;
    formation.final_base_.lin.at(towr::kVel) << x, y, z;
}

void set_final_base_angular_position(int session, double x, double y, double z) {
    auto lock = std::unique_lock(sessions.mutex);
    if (!check_session(session, lock)) return;

    auto& formation = sessions.data[session]->formulation;
    formation.final_base_.ang.at(towr::kPos) << x, y, z;
}

void set_final_base_angular_velocity(int session, double x, double y, double z) {
    auto lock = std::unique_lock(sessions.mutex);
    if (!check_session(session, lock)) return;

    auto& formation = sessions.data[session]->formulation;
    formation.final_base_.ang.at(towr::kVel) << x, y, z;
}

void set_initial_end_effector_position(int session, int id, double x, double y) {
    auto lock = std::unique_lock(sessions.mutex);
    if (!check_session(session, lock)) return;

    auto& formation = sessions.data[session]->formulation;
    auto z = formation.terrain_->GetHeight(x, y);
    formation.initial_ee_W_[id] << x, y, z;
}