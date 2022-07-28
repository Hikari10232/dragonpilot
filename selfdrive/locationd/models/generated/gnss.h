#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_4010717374297345647);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_5633106088223749202);
void gnss_H_mod_fun(double *state, double *out_7573960825995811843);
void gnss_f_fun(double *state, double dt, double *out_5885723575983756215);
void gnss_F_fun(double *state, double dt, double *out_2484021219078295684);
void gnss_h_6(double *state, double *sat_pos, double *out_32881397743364433);
void gnss_H_6(double *state, double *sat_pos, double *out_4275266657171808452);
void gnss_h_20(double *state, double *sat_pos, double *out_970729964152922164);
void gnss_H_20(double *state, double *sat_pos, double *out_4568390884189557169);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_6074491977418173187);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_805907699913486338);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_6074491977418173187);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_805907699913486338);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}