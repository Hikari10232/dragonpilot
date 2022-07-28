#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3650554156799795983) {
   out_3650554156799795983[0] = delta_x[0] + nom_x[0];
   out_3650554156799795983[1] = delta_x[1] + nom_x[1];
   out_3650554156799795983[2] = delta_x[2] + nom_x[2];
   out_3650554156799795983[3] = delta_x[3] + nom_x[3];
   out_3650554156799795983[4] = delta_x[4] + nom_x[4];
   out_3650554156799795983[5] = delta_x[5] + nom_x[5];
   out_3650554156799795983[6] = delta_x[6] + nom_x[6];
   out_3650554156799795983[7] = delta_x[7] + nom_x[7];
   out_3650554156799795983[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2584868814906915761) {
   out_2584868814906915761[0] = -nom_x[0] + true_x[0];
   out_2584868814906915761[1] = -nom_x[1] + true_x[1];
   out_2584868814906915761[2] = -nom_x[2] + true_x[2];
   out_2584868814906915761[3] = -nom_x[3] + true_x[3];
   out_2584868814906915761[4] = -nom_x[4] + true_x[4];
   out_2584868814906915761[5] = -nom_x[5] + true_x[5];
   out_2584868814906915761[6] = -nom_x[6] + true_x[6];
   out_2584868814906915761[7] = -nom_x[7] + true_x[7];
   out_2584868814906915761[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3427800612628027621) {
   out_3427800612628027621[0] = 1.0;
   out_3427800612628027621[1] = 0;
   out_3427800612628027621[2] = 0;
   out_3427800612628027621[3] = 0;
   out_3427800612628027621[4] = 0;
   out_3427800612628027621[5] = 0;
   out_3427800612628027621[6] = 0;
   out_3427800612628027621[7] = 0;
   out_3427800612628027621[8] = 0;
   out_3427800612628027621[9] = 0;
   out_3427800612628027621[10] = 1.0;
   out_3427800612628027621[11] = 0;
   out_3427800612628027621[12] = 0;
   out_3427800612628027621[13] = 0;
   out_3427800612628027621[14] = 0;
   out_3427800612628027621[15] = 0;
   out_3427800612628027621[16] = 0;
   out_3427800612628027621[17] = 0;
   out_3427800612628027621[18] = 0;
   out_3427800612628027621[19] = 0;
   out_3427800612628027621[20] = 1.0;
   out_3427800612628027621[21] = 0;
   out_3427800612628027621[22] = 0;
   out_3427800612628027621[23] = 0;
   out_3427800612628027621[24] = 0;
   out_3427800612628027621[25] = 0;
   out_3427800612628027621[26] = 0;
   out_3427800612628027621[27] = 0;
   out_3427800612628027621[28] = 0;
   out_3427800612628027621[29] = 0;
   out_3427800612628027621[30] = 1.0;
   out_3427800612628027621[31] = 0;
   out_3427800612628027621[32] = 0;
   out_3427800612628027621[33] = 0;
   out_3427800612628027621[34] = 0;
   out_3427800612628027621[35] = 0;
   out_3427800612628027621[36] = 0;
   out_3427800612628027621[37] = 0;
   out_3427800612628027621[38] = 0;
   out_3427800612628027621[39] = 0;
   out_3427800612628027621[40] = 1.0;
   out_3427800612628027621[41] = 0;
   out_3427800612628027621[42] = 0;
   out_3427800612628027621[43] = 0;
   out_3427800612628027621[44] = 0;
   out_3427800612628027621[45] = 0;
   out_3427800612628027621[46] = 0;
   out_3427800612628027621[47] = 0;
   out_3427800612628027621[48] = 0;
   out_3427800612628027621[49] = 0;
   out_3427800612628027621[50] = 1.0;
   out_3427800612628027621[51] = 0;
   out_3427800612628027621[52] = 0;
   out_3427800612628027621[53] = 0;
   out_3427800612628027621[54] = 0;
   out_3427800612628027621[55] = 0;
   out_3427800612628027621[56] = 0;
   out_3427800612628027621[57] = 0;
   out_3427800612628027621[58] = 0;
   out_3427800612628027621[59] = 0;
   out_3427800612628027621[60] = 1.0;
   out_3427800612628027621[61] = 0;
   out_3427800612628027621[62] = 0;
   out_3427800612628027621[63] = 0;
   out_3427800612628027621[64] = 0;
   out_3427800612628027621[65] = 0;
   out_3427800612628027621[66] = 0;
   out_3427800612628027621[67] = 0;
   out_3427800612628027621[68] = 0;
   out_3427800612628027621[69] = 0;
   out_3427800612628027621[70] = 1.0;
   out_3427800612628027621[71] = 0;
   out_3427800612628027621[72] = 0;
   out_3427800612628027621[73] = 0;
   out_3427800612628027621[74] = 0;
   out_3427800612628027621[75] = 0;
   out_3427800612628027621[76] = 0;
   out_3427800612628027621[77] = 0;
   out_3427800612628027621[78] = 0;
   out_3427800612628027621[79] = 0;
   out_3427800612628027621[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5906797748358082084) {
   out_5906797748358082084[0] = state[0];
   out_5906797748358082084[1] = state[1];
   out_5906797748358082084[2] = state[2];
   out_5906797748358082084[3] = state[3];
   out_5906797748358082084[4] = state[4];
   out_5906797748358082084[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5906797748358082084[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5906797748358082084[7] = state[7];
   out_5906797748358082084[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3663771206525014762) {
   out_3663771206525014762[0] = 1;
   out_3663771206525014762[1] = 0;
   out_3663771206525014762[2] = 0;
   out_3663771206525014762[3] = 0;
   out_3663771206525014762[4] = 0;
   out_3663771206525014762[5] = 0;
   out_3663771206525014762[6] = 0;
   out_3663771206525014762[7] = 0;
   out_3663771206525014762[8] = 0;
   out_3663771206525014762[9] = 0;
   out_3663771206525014762[10] = 1;
   out_3663771206525014762[11] = 0;
   out_3663771206525014762[12] = 0;
   out_3663771206525014762[13] = 0;
   out_3663771206525014762[14] = 0;
   out_3663771206525014762[15] = 0;
   out_3663771206525014762[16] = 0;
   out_3663771206525014762[17] = 0;
   out_3663771206525014762[18] = 0;
   out_3663771206525014762[19] = 0;
   out_3663771206525014762[20] = 1;
   out_3663771206525014762[21] = 0;
   out_3663771206525014762[22] = 0;
   out_3663771206525014762[23] = 0;
   out_3663771206525014762[24] = 0;
   out_3663771206525014762[25] = 0;
   out_3663771206525014762[26] = 0;
   out_3663771206525014762[27] = 0;
   out_3663771206525014762[28] = 0;
   out_3663771206525014762[29] = 0;
   out_3663771206525014762[30] = 1;
   out_3663771206525014762[31] = 0;
   out_3663771206525014762[32] = 0;
   out_3663771206525014762[33] = 0;
   out_3663771206525014762[34] = 0;
   out_3663771206525014762[35] = 0;
   out_3663771206525014762[36] = 0;
   out_3663771206525014762[37] = 0;
   out_3663771206525014762[38] = 0;
   out_3663771206525014762[39] = 0;
   out_3663771206525014762[40] = 1;
   out_3663771206525014762[41] = 0;
   out_3663771206525014762[42] = 0;
   out_3663771206525014762[43] = 0;
   out_3663771206525014762[44] = 0;
   out_3663771206525014762[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3663771206525014762[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3663771206525014762[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3663771206525014762[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3663771206525014762[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3663771206525014762[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3663771206525014762[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3663771206525014762[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3663771206525014762[53] = -9.8000000000000007*dt;
   out_3663771206525014762[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3663771206525014762[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3663771206525014762[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3663771206525014762[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3663771206525014762[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3663771206525014762[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3663771206525014762[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3663771206525014762[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3663771206525014762[62] = 0;
   out_3663771206525014762[63] = 0;
   out_3663771206525014762[64] = 0;
   out_3663771206525014762[65] = 0;
   out_3663771206525014762[66] = 0;
   out_3663771206525014762[67] = 0;
   out_3663771206525014762[68] = 0;
   out_3663771206525014762[69] = 0;
   out_3663771206525014762[70] = 1;
   out_3663771206525014762[71] = 0;
   out_3663771206525014762[72] = 0;
   out_3663771206525014762[73] = 0;
   out_3663771206525014762[74] = 0;
   out_3663771206525014762[75] = 0;
   out_3663771206525014762[76] = 0;
   out_3663771206525014762[77] = 0;
   out_3663771206525014762[78] = 0;
   out_3663771206525014762[79] = 0;
   out_3663771206525014762[80] = 1;
}
void h_25(double *state, double *unused, double *out_9113208939091869473) {
   out_9113208939091869473[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2338072232428484233) {
   out_2338072232428484233[0] = 0;
   out_2338072232428484233[1] = 0;
   out_2338072232428484233[2] = 0;
   out_2338072232428484233[3] = 0;
   out_2338072232428484233[4] = 0;
   out_2338072232428484233[5] = 0;
   out_2338072232428484233[6] = 1;
   out_2338072232428484233[7] = 0;
   out_2338072232428484233[8] = 0;
}
void h_24(double *state, double *unused, double *out_1072197497808843752) {
   out_1072197497808843752[0] = state[4];
   out_1072197497808843752[1] = state[5];
}
void H_24(double *state, double *unused, double *out_925043984390075396) {
   out_925043984390075396[0] = 0;
   out_925043984390075396[1] = 0;
   out_925043984390075396[2] = 0;
   out_925043984390075396[3] = 0;
   out_925043984390075396[4] = 1;
   out_925043984390075396[5] = 0;
   out_925043984390075396[6] = 0;
   out_925043984390075396[7] = 0;
   out_925043984390075396[8] = 0;
   out_925043984390075396[9] = 0;
   out_925043984390075396[10] = 0;
   out_925043984390075396[11] = 0;
   out_925043984390075396[12] = 0;
   out_925043984390075396[13] = 0;
   out_925043984390075396[14] = 1;
   out_925043984390075396[15] = 0;
   out_925043984390075396[16] = 0;
   out_925043984390075396[17] = 0;
}
void h_30(double *state, double *unused, double *out_616548257828129481) {
   out_616548257828129481[0] = state[4];
}
void H_30(double *state, double *unused, double *out_180260726078764394) {
   out_180260726078764394[0] = 0;
   out_180260726078764394[1] = 0;
   out_180260726078764394[2] = 0;
   out_180260726078764394[3] = 0;
   out_180260726078764394[4] = 1;
   out_180260726078764394[5] = 0;
   out_180260726078764394[6] = 0;
   out_180260726078764394[7] = 0;
   out_180260726078764394[8] = 0;
}
void h_26(double *state, double *unused, double *out_13232373151651964) {
   out_13232373151651964[0] = state[7];
}
void H_26(double *state, double *unused, double *out_966453737332316368) {
   out_966453737332316368[0] = 0;
   out_966453737332316368[1] = 0;
   out_966453737332316368[2] = 0;
   out_966453737332316368[3] = 0;
   out_966453737332316368[4] = 0;
   out_966453737332316368[5] = 0;
   out_966453737332316368[6] = 0;
   out_966453737332316368[7] = 1;
   out_966453737332316368[8] = 0;
}
void h_27(double *state, double *unused, double *out_131239767085028708) {
   out_131239767085028708[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2403854797262707611) {
   out_2403854797262707611[0] = 0;
   out_2403854797262707611[1] = 0;
   out_2403854797262707611[2] = 0;
   out_2403854797262707611[3] = 1;
   out_2403854797262707611[4] = 0;
   out_2403854797262707611[5] = 0;
   out_2403854797262707611[6] = 0;
   out_2403854797262707611[7] = 0;
   out_2403854797262707611[8] = 0;
}
void h_29(double *state, double *unused, double *out_5782493259585719665) {
   out_5782493259585719665[0] = state[1];
}
void H_29(double *state, double *unused, double *out_690492070393156578) {
   out_690492070393156578[0] = 0;
   out_690492070393156578[1] = 1;
   out_690492070393156578[2] = 0;
   out_690492070393156578[3] = 0;
   out_690492070393156578[4] = 0;
   out_690492070393156578[5] = 0;
   out_690492070393156578[6] = 0;
   out_690492070393156578[7] = 0;
   out_690492070393156578[8] = 0;
}
void h_28(double *state, double *unused, double *out_734555651761506225) {
   out_734555651761506225[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4391906946676373996) {
   out_4391906946676373996[0] = 1;
   out_4391906946676373996[1] = 0;
   out_4391906946676373996[2] = 0;
   out_4391906946676373996[3] = 0;
   out_4391906946676373996[4] = 0;
   out_4391906946676373996[5] = 0;
   out_4391906946676373996[6] = 0;
   out_4391906946676373996[7] = 0;
   out_4391906946676373996[8] = 0;
}
void h_31(double *state, double *unused, double *out_8838014876807363584) {
   out_8838014876807363584[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4738603018083333020) {
   out_4738603018083333020[0] = 0;
   out_4738603018083333020[1] = 0;
   out_4738603018083333020[2] = 0;
   out_4738603018083333020[3] = 0;
   out_4738603018083333020[4] = 0;
   out_4738603018083333020[5] = 0;
   out_4738603018083333020[6] = 0;
   out_4738603018083333020[7] = 0;
   out_4738603018083333020[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_3650554156799795983) {
  err_fun(nom_x, delta_x, out_3650554156799795983);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2584868814906915761) {
  inv_err_fun(nom_x, true_x, out_2584868814906915761);
}
void car_H_mod_fun(double *state, double *out_3427800612628027621) {
  H_mod_fun(state, out_3427800612628027621);
}
void car_f_fun(double *state, double dt, double *out_5906797748358082084) {
  f_fun(state,  dt, out_5906797748358082084);
}
void car_F_fun(double *state, double dt, double *out_3663771206525014762) {
  F_fun(state,  dt, out_3663771206525014762);
}
void car_h_25(double *state, double *unused, double *out_9113208939091869473) {
  h_25(state, unused, out_9113208939091869473);
}
void car_H_25(double *state, double *unused, double *out_2338072232428484233) {
  H_25(state, unused, out_2338072232428484233);
}
void car_h_24(double *state, double *unused, double *out_1072197497808843752) {
  h_24(state, unused, out_1072197497808843752);
}
void car_H_24(double *state, double *unused, double *out_925043984390075396) {
  H_24(state, unused, out_925043984390075396);
}
void car_h_30(double *state, double *unused, double *out_616548257828129481) {
  h_30(state, unused, out_616548257828129481);
}
void car_H_30(double *state, double *unused, double *out_180260726078764394) {
  H_30(state, unused, out_180260726078764394);
}
void car_h_26(double *state, double *unused, double *out_13232373151651964) {
  h_26(state, unused, out_13232373151651964);
}
void car_H_26(double *state, double *unused, double *out_966453737332316368) {
  H_26(state, unused, out_966453737332316368);
}
void car_h_27(double *state, double *unused, double *out_131239767085028708) {
  h_27(state, unused, out_131239767085028708);
}
void car_H_27(double *state, double *unused, double *out_2403854797262707611) {
  H_27(state, unused, out_2403854797262707611);
}
void car_h_29(double *state, double *unused, double *out_5782493259585719665) {
  h_29(state, unused, out_5782493259585719665);
}
void car_H_29(double *state, double *unused, double *out_690492070393156578) {
  H_29(state, unused, out_690492070393156578);
}
void car_h_28(double *state, double *unused, double *out_734555651761506225) {
  h_28(state, unused, out_734555651761506225);
}
void car_H_28(double *state, double *unused, double *out_4391906946676373996) {
  H_28(state, unused, out_4391906946676373996);
}
void car_h_31(double *state, double *unused, double *out_8838014876807363584) {
  h_31(state, unused, out_8838014876807363584);
}
void car_H_31(double *state, double *unused, double *out_4738603018083333020) {
  H_31(state, unused, out_4738603018083333020);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
