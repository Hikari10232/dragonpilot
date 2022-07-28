#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_3650554156799795983);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2584868814906915761);
void car_H_mod_fun(double *state, double *out_3427800612628027621);
void car_f_fun(double *state, double dt, double *out_5906797748358082084);
void car_F_fun(double *state, double dt, double *out_3663771206525014762);
void car_h_25(double *state, double *unused, double *out_9113208939091869473);
void car_H_25(double *state, double *unused, double *out_2338072232428484233);
void car_h_24(double *state, double *unused, double *out_1072197497808843752);
void car_H_24(double *state, double *unused, double *out_925043984390075396);
void car_h_30(double *state, double *unused, double *out_616548257828129481);
void car_H_30(double *state, double *unused, double *out_180260726078764394);
void car_h_26(double *state, double *unused, double *out_13232373151651964);
void car_H_26(double *state, double *unused, double *out_966453737332316368);
void car_h_27(double *state, double *unused, double *out_131239767085028708);
void car_H_27(double *state, double *unused, double *out_2403854797262707611);
void car_h_29(double *state, double *unused, double *out_5782493259585719665);
void car_H_29(double *state, double *unused, double *out_690492070393156578);
void car_h_28(double *state, double *unused, double *out_734555651761506225);
void car_H_28(double *state, double *unused, double *out_4391906946676373996);
void car_h_31(double *state, double *unused, double *out_8838014876807363584);
void car_H_31(double *state, double *unused, double *out_4738603018083333020);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}