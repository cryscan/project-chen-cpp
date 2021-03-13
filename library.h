//
// Created by cryscan on 2/26/21.
//

#ifndef PLAYGROUND_LIBRARY_H
#define PLAYGROUND_LIBRARY_H

extern "C" {
int create_session(int model);
void end_session(int session);

struct RobotModel {
    double nominal_stance[4][3];
    double max_deviation[3];

    double mass;
    int ee_count;
};

void get_robot_model(int session, RobotModel* robot_model);
int get_ee_count(int session);

struct Bound {
    double duration;

    double initial_base_linear_position[3];
    double initial_base_linear_velocity[3];
    double initial_base_angular_position[3];
    double initial_base_angular_velocity[3];

    double final_base_linear_position[3];
    double final_base_linear_velocity[3];
    double final_base_angular_position[3];
    double final_base_angular_velocity[3];

    double initial_ee_positions[4][3];

    unsigned char bounds_final_linear_position;
    unsigned char bounds_final_linear_velocity;
    unsigned char bounds_final_angular_position;
    unsigned char bounds_final_angular_velocity;
};

struct Option {
    double max_cpu_time;
    int max_iter;
    bool optimize_phase_durations;
};

void set_bound(int session, const Bound* bound);
void set_option(int session, const Option* option);

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

bool solution_ready(int session);
bool get_solution_state(int session, double time, State* state);
}

#endif //PLAYGROUND_LIBRARY_H
