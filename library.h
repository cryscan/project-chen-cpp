//
// Created by cryscan on 2/26/21.
//

#ifndef PLAYGROUND_LIBRARY_H
#define PLAYGROUND_LIBRARY_H

extern "C" {
int create_session(int model);
void end_session(int session);

struct Model {
    double nominal_stance[4][3];
    double max_deviation[3];

    double mass;
    int ee_count;
};

void get_model(int session, Model* model);
int get_ee_count(int session);

struct Parameters {
    double initial_base_lin_pos[3];
    double initial_base_lin_vel[3];
    double initial_base_ang_pos[3];
    double initial_base_ang_vel[3];

    double final_base_lin_pos[3];
    double final_base_lin_vel[3];
    double final_base_ang_pos[3];
    double final_base_ang_vel[3];

    double initial_ee_pos[4][3];

    unsigned char bounds_final_lin_pos;
    unsigned char bounds_final_lin_vel;
    unsigned char bounds_final_ang_pos;
    unsigned char bounds_final_ang_vel;
};

struct Options {
    double max_cpu_time;
    int max_iter;
    bool optimize_phase_durations;
};

void set_params(int session, const Parameters* parameters);
void set_options(int session, const Options* options);
void set_duration(int session, double duration);

struct PathPoint {
    double time;
    double linear[3];
    double angular[3];
};

void push_path_point(int session, const PathPoint* path_point);
void push_gait(int session, int gait);

void start_optimization(int session);

struct State {
    double base_lin_pos[3];
    double base_lin_vel[3];
    double base_ang_pos[3];
    double base_ang_vel[3];

    double ee_motions[4][3];
    double ee_forces[4][3];
    bool contacts[4];
};

bool solution_ready(int session);
bool get_solution_state(int session, double time, State* state);

int create_terrain(double pos_x, double pos_y, double pos_z, uint x, uint y, double unit_size);
void end_terrain(int terrain);

void set_height(int terrain, uint x, uint y, double height);
double get_height(int terrain, double x, double y);
void get_height_derivatives(int terrain, double x, double y, double* dx, double* dy);
}

#endif //PLAYGROUND_LIBRARY_H
