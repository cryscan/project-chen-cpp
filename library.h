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
}

#endif //PLAYGROUND_LIBRARY_H
