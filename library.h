//
// Created by cryscan on 2/26/21.
//

#ifndef PLAYGROUND_LIBRARY_H
#define PLAYGROUND_LIBRARY_H

extern "C" {
int create_session(int model);
void end_session(int session);

struct ModelInfo {
    double nominal_stance[4][3];
    double max_deviation[3];
};

void get_model_info(int session, ModelInfo* output);
int get_ee_count(int session);

struct Bound {
    double initial_base_linear_position[3];
    double initial_base_linear_velocity[3];
    double initial_base_angular_position[3];
    double initial_base_angular_velocity[3];

    double final_base_linear_position[3];
    double final_base_linear_velocity[3];
    double final_base_angular_position[3];
    double final_base_angular_velocity[3];

    double initial_ee_positions[4][3];

    double duration;

    double max_cpu_time;
    int max_iter;
    bool optimize_phase_durations;
};

void set_bound(int session, const Bound* bound);

void start_optimization(int session);

struct State {
    double base_linear_position[3];
    double base_linear_velocity[3];
    double base_angular_position[3];
    double base_angular_velocity[3];

    double ee_motions[4][3];
    double ee_forces[4][3];
    bool contacts[4];
};

bool get_solution(int session, double time, State* output);
bool solution_ready(int session);
}

#endif //PLAYGROUND_LIBRARY_H
