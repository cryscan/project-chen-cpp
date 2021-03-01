//
// Created by cryscan on 2/26/21.
//

#ifndef PLAYGROUND_LIBRARY_H
#define PLAYGROUND_LIBRARY_H

extern "C" {
int create_session();
void end_session(int session);

void set_initial_base_linear_position(int session, double x, double y, double z);
void set_initial_base_linear_velocity(int session, double x, double y, double z);
void set_initial_base_angular_position(int session, double x, double y, double z);
void set_initial_base_angular_velocity(int session, double x, double y, double z);

void set_final_base_linear_position(int session, double x, double y, double z);
void set_final_base_linear_velocity(int session, double x, double y, double z);
void set_final_base_angular_position(int session, double x, double y, double z);
void set_final_base_angular_velocity(int session, double x, double y, double z);

void set_initial_end_effector_position(int session, int id, double x, double y);

void start_optimization(int session);

bool get_base_linear_position(int session, double time, double* output);
bool get_base_angular_position(int session, double time, double* output);
bool get_end_effector_position(int session, int id, double time, double* output);
bool get_end_effector_force(int session, int id, double time, double* output);
bool get_end_effector_contact(int session, int id, double time, bool* output);
}

#endif //PLAYGROUND_LIBRARY_H
