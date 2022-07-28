#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4010717374297345647) {
   out_4010717374297345647[0] = delta_x[0] + nom_x[0];
   out_4010717374297345647[1] = delta_x[1] + nom_x[1];
   out_4010717374297345647[2] = delta_x[2] + nom_x[2];
   out_4010717374297345647[3] = delta_x[3] + nom_x[3];
   out_4010717374297345647[4] = delta_x[4] + nom_x[4];
   out_4010717374297345647[5] = delta_x[5] + nom_x[5];
   out_4010717374297345647[6] = delta_x[6] + nom_x[6];
   out_4010717374297345647[7] = delta_x[7] + nom_x[7];
   out_4010717374297345647[8] = delta_x[8] + nom_x[8];
   out_4010717374297345647[9] = delta_x[9] + nom_x[9];
   out_4010717374297345647[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5633106088223749202) {
   out_5633106088223749202[0] = -nom_x[0] + true_x[0];
   out_5633106088223749202[1] = -nom_x[1] + true_x[1];
   out_5633106088223749202[2] = -nom_x[2] + true_x[2];
   out_5633106088223749202[3] = -nom_x[3] + true_x[3];
   out_5633106088223749202[4] = -nom_x[4] + true_x[4];
   out_5633106088223749202[5] = -nom_x[5] + true_x[5];
   out_5633106088223749202[6] = -nom_x[6] + true_x[6];
   out_5633106088223749202[7] = -nom_x[7] + true_x[7];
   out_5633106088223749202[8] = -nom_x[8] + true_x[8];
   out_5633106088223749202[9] = -nom_x[9] + true_x[9];
   out_5633106088223749202[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_7573960825995811843) {
   out_7573960825995811843[0] = 1.0;
   out_7573960825995811843[1] = 0;
   out_7573960825995811843[2] = 0;
   out_7573960825995811843[3] = 0;
   out_7573960825995811843[4] = 0;
   out_7573960825995811843[5] = 0;
   out_7573960825995811843[6] = 0;
   out_7573960825995811843[7] = 0;
   out_7573960825995811843[8] = 0;
   out_7573960825995811843[9] = 0;
   out_7573960825995811843[10] = 0;
   out_7573960825995811843[11] = 0;
   out_7573960825995811843[12] = 1.0;
   out_7573960825995811843[13] = 0;
   out_7573960825995811843[14] = 0;
   out_7573960825995811843[15] = 0;
   out_7573960825995811843[16] = 0;
   out_7573960825995811843[17] = 0;
   out_7573960825995811843[18] = 0;
   out_7573960825995811843[19] = 0;
   out_7573960825995811843[20] = 0;
   out_7573960825995811843[21] = 0;
   out_7573960825995811843[22] = 0;
   out_7573960825995811843[23] = 0;
   out_7573960825995811843[24] = 1.0;
   out_7573960825995811843[25] = 0;
   out_7573960825995811843[26] = 0;
   out_7573960825995811843[27] = 0;
   out_7573960825995811843[28] = 0;
   out_7573960825995811843[29] = 0;
   out_7573960825995811843[30] = 0;
   out_7573960825995811843[31] = 0;
   out_7573960825995811843[32] = 0;
   out_7573960825995811843[33] = 0;
   out_7573960825995811843[34] = 0;
   out_7573960825995811843[35] = 0;
   out_7573960825995811843[36] = 1.0;
   out_7573960825995811843[37] = 0;
   out_7573960825995811843[38] = 0;
   out_7573960825995811843[39] = 0;
   out_7573960825995811843[40] = 0;
   out_7573960825995811843[41] = 0;
   out_7573960825995811843[42] = 0;
   out_7573960825995811843[43] = 0;
   out_7573960825995811843[44] = 0;
   out_7573960825995811843[45] = 0;
   out_7573960825995811843[46] = 0;
   out_7573960825995811843[47] = 0;
   out_7573960825995811843[48] = 1.0;
   out_7573960825995811843[49] = 0;
   out_7573960825995811843[50] = 0;
   out_7573960825995811843[51] = 0;
   out_7573960825995811843[52] = 0;
   out_7573960825995811843[53] = 0;
   out_7573960825995811843[54] = 0;
   out_7573960825995811843[55] = 0;
   out_7573960825995811843[56] = 0;
   out_7573960825995811843[57] = 0;
   out_7573960825995811843[58] = 0;
   out_7573960825995811843[59] = 0;
   out_7573960825995811843[60] = 1.0;
   out_7573960825995811843[61] = 0;
   out_7573960825995811843[62] = 0;
   out_7573960825995811843[63] = 0;
   out_7573960825995811843[64] = 0;
   out_7573960825995811843[65] = 0;
   out_7573960825995811843[66] = 0;
   out_7573960825995811843[67] = 0;
   out_7573960825995811843[68] = 0;
   out_7573960825995811843[69] = 0;
   out_7573960825995811843[70] = 0;
   out_7573960825995811843[71] = 0;
   out_7573960825995811843[72] = 1.0;
   out_7573960825995811843[73] = 0;
   out_7573960825995811843[74] = 0;
   out_7573960825995811843[75] = 0;
   out_7573960825995811843[76] = 0;
   out_7573960825995811843[77] = 0;
   out_7573960825995811843[78] = 0;
   out_7573960825995811843[79] = 0;
   out_7573960825995811843[80] = 0;
   out_7573960825995811843[81] = 0;
   out_7573960825995811843[82] = 0;
   out_7573960825995811843[83] = 0;
   out_7573960825995811843[84] = 1.0;
   out_7573960825995811843[85] = 0;
   out_7573960825995811843[86] = 0;
   out_7573960825995811843[87] = 0;
   out_7573960825995811843[88] = 0;
   out_7573960825995811843[89] = 0;
   out_7573960825995811843[90] = 0;
   out_7573960825995811843[91] = 0;
   out_7573960825995811843[92] = 0;
   out_7573960825995811843[93] = 0;
   out_7573960825995811843[94] = 0;
   out_7573960825995811843[95] = 0;
   out_7573960825995811843[96] = 1.0;
   out_7573960825995811843[97] = 0;
   out_7573960825995811843[98] = 0;
   out_7573960825995811843[99] = 0;
   out_7573960825995811843[100] = 0;
   out_7573960825995811843[101] = 0;
   out_7573960825995811843[102] = 0;
   out_7573960825995811843[103] = 0;
   out_7573960825995811843[104] = 0;
   out_7573960825995811843[105] = 0;
   out_7573960825995811843[106] = 0;
   out_7573960825995811843[107] = 0;
   out_7573960825995811843[108] = 1.0;
   out_7573960825995811843[109] = 0;
   out_7573960825995811843[110] = 0;
   out_7573960825995811843[111] = 0;
   out_7573960825995811843[112] = 0;
   out_7573960825995811843[113] = 0;
   out_7573960825995811843[114] = 0;
   out_7573960825995811843[115] = 0;
   out_7573960825995811843[116] = 0;
   out_7573960825995811843[117] = 0;
   out_7573960825995811843[118] = 0;
   out_7573960825995811843[119] = 0;
   out_7573960825995811843[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_5885723575983756215) {
   out_5885723575983756215[0] = dt*state[3] + state[0];
   out_5885723575983756215[1] = dt*state[4] + state[1];
   out_5885723575983756215[2] = dt*state[5] + state[2];
   out_5885723575983756215[3] = state[3];
   out_5885723575983756215[4] = state[4];
   out_5885723575983756215[5] = state[5];
   out_5885723575983756215[6] = dt*state[7] + state[6];
   out_5885723575983756215[7] = dt*state[8] + state[7];
   out_5885723575983756215[8] = state[8];
   out_5885723575983756215[9] = state[9];
   out_5885723575983756215[10] = state[10];
}
void F_fun(double *state, double dt, double *out_2484021219078295684) {
   out_2484021219078295684[0] = 1;
   out_2484021219078295684[1] = 0;
   out_2484021219078295684[2] = 0;
   out_2484021219078295684[3] = dt;
   out_2484021219078295684[4] = 0;
   out_2484021219078295684[5] = 0;
   out_2484021219078295684[6] = 0;
   out_2484021219078295684[7] = 0;
   out_2484021219078295684[8] = 0;
   out_2484021219078295684[9] = 0;
   out_2484021219078295684[10] = 0;
   out_2484021219078295684[11] = 0;
   out_2484021219078295684[12] = 1;
   out_2484021219078295684[13] = 0;
   out_2484021219078295684[14] = 0;
   out_2484021219078295684[15] = dt;
   out_2484021219078295684[16] = 0;
   out_2484021219078295684[17] = 0;
   out_2484021219078295684[18] = 0;
   out_2484021219078295684[19] = 0;
   out_2484021219078295684[20] = 0;
   out_2484021219078295684[21] = 0;
   out_2484021219078295684[22] = 0;
   out_2484021219078295684[23] = 0;
   out_2484021219078295684[24] = 1;
   out_2484021219078295684[25] = 0;
   out_2484021219078295684[26] = 0;
   out_2484021219078295684[27] = dt;
   out_2484021219078295684[28] = 0;
   out_2484021219078295684[29] = 0;
   out_2484021219078295684[30] = 0;
   out_2484021219078295684[31] = 0;
   out_2484021219078295684[32] = 0;
   out_2484021219078295684[33] = 0;
   out_2484021219078295684[34] = 0;
   out_2484021219078295684[35] = 0;
   out_2484021219078295684[36] = 1;
   out_2484021219078295684[37] = 0;
   out_2484021219078295684[38] = 0;
   out_2484021219078295684[39] = 0;
   out_2484021219078295684[40] = 0;
   out_2484021219078295684[41] = 0;
   out_2484021219078295684[42] = 0;
   out_2484021219078295684[43] = 0;
   out_2484021219078295684[44] = 0;
   out_2484021219078295684[45] = 0;
   out_2484021219078295684[46] = 0;
   out_2484021219078295684[47] = 0;
   out_2484021219078295684[48] = 1;
   out_2484021219078295684[49] = 0;
   out_2484021219078295684[50] = 0;
   out_2484021219078295684[51] = 0;
   out_2484021219078295684[52] = 0;
   out_2484021219078295684[53] = 0;
   out_2484021219078295684[54] = 0;
   out_2484021219078295684[55] = 0;
   out_2484021219078295684[56] = 0;
   out_2484021219078295684[57] = 0;
   out_2484021219078295684[58] = 0;
   out_2484021219078295684[59] = 0;
   out_2484021219078295684[60] = 1;
   out_2484021219078295684[61] = 0;
   out_2484021219078295684[62] = 0;
   out_2484021219078295684[63] = 0;
   out_2484021219078295684[64] = 0;
   out_2484021219078295684[65] = 0;
   out_2484021219078295684[66] = 0;
   out_2484021219078295684[67] = 0;
   out_2484021219078295684[68] = 0;
   out_2484021219078295684[69] = 0;
   out_2484021219078295684[70] = 0;
   out_2484021219078295684[71] = 0;
   out_2484021219078295684[72] = 1;
   out_2484021219078295684[73] = dt;
   out_2484021219078295684[74] = 0;
   out_2484021219078295684[75] = 0;
   out_2484021219078295684[76] = 0;
   out_2484021219078295684[77] = 0;
   out_2484021219078295684[78] = 0;
   out_2484021219078295684[79] = 0;
   out_2484021219078295684[80] = 0;
   out_2484021219078295684[81] = 0;
   out_2484021219078295684[82] = 0;
   out_2484021219078295684[83] = 0;
   out_2484021219078295684[84] = 1;
   out_2484021219078295684[85] = dt;
   out_2484021219078295684[86] = 0;
   out_2484021219078295684[87] = 0;
   out_2484021219078295684[88] = 0;
   out_2484021219078295684[89] = 0;
   out_2484021219078295684[90] = 0;
   out_2484021219078295684[91] = 0;
   out_2484021219078295684[92] = 0;
   out_2484021219078295684[93] = 0;
   out_2484021219078295684[94] = 0;
   out_2484021219078295684[95] = 0;
   out_2484021219078295684[96] = 1;
   out_2484021219078295684[97] = 0;
   out_2484021219078295684[98] = 0;
   out_2484021219078295684[99] = 0;
   out_2484021219078295684[100] = 0;
   out_2484021219078295684[101] = 0;
   out_2484021219078295684[102] = 0;
   out_2484021219078295684[103] = 0;
   out_2484021219078295684[104] = 0;
   out_2484021219078295684[105] = 0;
   out_2484021219078295684[106] = 0;
   out_2484021219078295684[107] = 0;
   out_2484021219078295684[108] = 1;
   out_2484021219078295684[109] = 0;
   out_2484021219078295684[110] = 0;
   out_2484021219078295684[111] = 0;
   out_2484021219078295684[112] = 0;
   out_2484021219078295684[113] = 0;
   out_2484021219078295684[114] = 0;
   out_2484021219078295684[115] = 0;
   out_2484021219078295684[116] = 0;
   out_2484021219078295684[117] = 0;
   out_2484021219078295684[118] = 0;
   out_2484021219078295684[119] = 0;
   out_2484021219078295684[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_32881397743364433) {
   out_32881397743364433[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_4275266657171808452) {
   out_4275266657171808452[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4275266657171808452[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4275266657171808452[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4275266657171808452[3] = 0;
   out_4275266657171808452[4] = 0;
   out_4275266657171808452[5] = 0;
   out_4275266657171808452[6] = 1;
   out_4275266657171808452[7] = 0;
   out_4275266657171808452[8] = 0;
   out_4275266657171808452[9] = 0;
   out_4275266657171808452[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_970729964152922164) {
   out_970729964152922164[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_4568390884189557169) {
   out_4568390884189557169[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4568390884189557169[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4568390884189557169[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4568390884189557169[3] = 0;
   out_4568390884189557169[4] = 0;
   out_4568390884189557169[5] = 0;
   out_4568390884189557169[6] = 1;
   out_4568390884189557169[7] = 0;
   out_4568390884189557169[8] = 0;
   out_4568390884189557169[9] = 1;
   out_4568390884189557169[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_6074491977418173187) {
   out_6074491977418173187[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_805907699913486338) {
   out_805907699913486338[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_805907699913486338[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_805907699913486338[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_805907699913486338[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_805907699913486338[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_805907699913486338[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_805907699913486338[6] = 0;
   out_805907699913486338[7] = 1;
   out_805907699913486338[8] = 0;
   out_805907699913486338[9] = 0;
   out_805907699913486338[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_6074491977418173187) {
   out_6074491977418173187[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_805907699913486338) {
   out_805907699913486338[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_805907699913486338[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_805907699913486338[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_805907699913486338[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_805907699913486338[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_805907699913486338[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_805907699913486338[6] = 0;
   out_805907699913486338[7] = 1;
   out_805907699913486338[8] = 0;
   out_805907699913486338[9] = 0;
   out_805907699913486338[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_4010717374297345647) {
  err_fun(nom_x, delta_x, out_4010717374297345647);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_5633106088223749202) {
  inv_err_fun(nom_x, true_x, out_5633106088223749202);
}
void gnss_H_mod_fun(double *state, double *out_7573960825995811843) {
  H_mod_fun(state, out_7573960825995811843);
}
void gnss_f_fun(double *state, double dt, double *out_5885723575983756215) {
  f_fun(state,  dt, out_5885723575983756215);
}
void gnss_F_fun(double *state, double dt, double *out_2484021219078295684) {
  F_fun(state,  dt, out_2484021219078295684);
}
void gnss_h_6(double *state, double *sat_pos, double *out_32881397743364433) {
  h_6(state, sat_pos, out_32881397743364433);
}
void gnss_H_6(double *state, double *sat_pos, double *out_4275266657171808452) {
  H_6(state, sat_pos, out_4275266657171808452);
}
void gnss_h_20(double *state, double *sat_pos, double *out_970729964152922164) {
  h_20(state, sat_pos, out_970729964152922164);
}
void gnss_H_20(double *state, double *sat_pos, double *out_4568390884189557169) {
  H_20(state, sat_pos, out_4568390884189557169);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_6074491977418173187) {
  h_7(state, sat_pos_vel, out_6074491977418173187);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_805907699913486338) {
  H_7(state, sat_pos_vel, out_805907699913486338);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_6074491977418173187) {
  h_21(state, sat_pos_vel, out_6074491977418173187);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_805907699913486338) {
  H_21(state, sat_pos_vel, out_805907699913486338);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
