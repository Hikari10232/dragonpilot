#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_2252060213547807159);
void live_err_fun(double *nom_x, double *delta_x, double *out_7841525859904799454);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4658714646580996073);
void live_H_mod_fun(double *state, double *out_9058380023042709905);
void live_f_fun(double *state, double dt, double *out_1044564808547905457);
void live_F_fun(double *state, double dt, double *out_6112935945836886280);
void live_h_4(double *state, double *unused, double *out_6558505756783094796);
void live_H_4(double *state, double *unused, double *out_4393075056079565068);
void live_h_9(double *state, double *unused, double *out_7725128392923987525);
void live_H_9(double *state, double *unused, double *out_4634264702709155713);
void live_h_10(double *state, double *unused, double *out_6608025656990009536);
void live_H_10(double *state, double *unused, double *out_2080906802359322363);
void live_h_12(double *state, double *unused, double *out_8968777591833378658);
void live_H_12(double *state, double *unused, double *out_9034212609598024753);
void live_h_35(double *state, double *unused, double *out_1739278652694740466);
void live_H_35(double *state, double *unused, double *out_6288649577273011044);
void live_h_32(double *state, double *unused, double *out_4439112886558686093);
void live_H_32(double *state, double *unused, double *out_7817089720703433684);
void live_h_13(double *state, double *unused, double *out_33375021745986410);
void live_H_13(double *state, double *unused, double *out_1491437037948813424);
void live_h_14(double *state, double *unused, double *out_7725128392923987525);
void live_H_14(double *state, double *unused, double *out_4634264702709155713);
void live_h_33(double *state, double *unused, double *out_3596385653652256868);
void live_H_33(double *state, double *unused, double *out_3138092572634153440);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}