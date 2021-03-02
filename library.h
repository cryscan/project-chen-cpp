//
// Created by cryscan on 2/26/21.
//

#ifndef PLAYGROUND_LIBRARY_H
#define PLAYGROUND_LIBRARY_H

extern "C" {
int create_session(double duration);
void end_session(int session);

void set_initial_base_linear_position(int session, double x, double y, double z);
void set_initial_base_linear_velocity(int session, double x, double y, double z);
void set_initial_base_angular_position(int session, double x, double y, double z);
void set_initial_base_angular_velocity(int session, double x, double y, double z);

void set_final_base_linear_position(int session, double x, double y, double z);
void set_final_base_linear_velocity(int session, double x, double y, double z);
void set_final_base_angular_position(int session, double x, double y, double z);
void set_final_base_angular_velocity(int session, double x, double y, double z);

void set_initial_ee_position(int session, double x, double y);

void start_optimization(int session);

bool
get_solution(int session,
             double time,
             double* base_linear,
             double* base_angular,
             double* ee_motion,
             double* ee_force,
             bool* contact);
}

#endif //PLAYGROUND_LIBRARY_H
