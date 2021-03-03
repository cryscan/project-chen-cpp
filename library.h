//
// Created by cryscan on 2/26/21.
//

#ifndef PLAYGROUND_LIBRARY_H
#define PLAYGROUND_LIBRARY_H

extern "C" {
int create_session();
void end_session(int session);

struct Boundary {
    double initial_base_linear_position[3];
    double initial_base_linear_velocity[3];
    double initial_base_angular_position[3];
    double initial_base_angular_velocity[3];

    double final_base_linear_position[3];
    double final_base_linear_velocity[3];
    double final_base_angular_position[3];
    double final_base_angular_velocity[3];

    double initial_ee_position[3];

    double duration;
};

/*
void set_initial_base_linear_position(int session, double x, double y, double z);
void set_initial_base_linear_velocity(int session, double x, double y, double z);
void set_initial_base_angular_position(int session, double x, double y, double z);
void set_initial_base_angular_velocity(int session, double x, double y, double z);

void set_final_base_linear_position(int session, double x, double y, double z);
void set_final_base_linear_velocity(int session, double x, double y, double z);
void set_final_base_angular_position(int session, double x, double y, double z);
void set_final_base_angular_velocity(int session, double x, double y, double z);

void set_initial_ee_position(int session, double x, double y);
 */

void set_boundary(int session, const Boundary* boundary);

void start_optimization(int session);

struct State {
    double base_linear_position[3];
    double base_linear_velocity[3];
    double base_angular_position[3];
    double base_angular_velocity[3];

    double ee_motion[3];
    double ee_force[3];
    bool contact;
};

bool get_solution(int session, double time, State* output);
bool solution_ready(int session);
}

#endif //PLAYGROUND_LIBRARY_H
