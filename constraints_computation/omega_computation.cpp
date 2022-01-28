//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// omega_computation.cpp
//
// Code generation for function 'omega_computation'
//

// Include files
#include "omega_computation.h"
#include <cmath>

// Function Declarations
static double rt_powd_snf(double u0, double u1);

// Function Definitions
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = std::numeric_limits<double>::quiet_NaN();
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = std::numeric_limits<int>::max();
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = std::numeric_limits<int>::max();
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = std::numeric_limits<double>::quiet_NaN();
    } else {
      y = std::pow(u0, u1);
    }
  }
  return y;
}

void Test_omegaComputation(double* in1, double* omega)
{
  double a;
  double a_tmp;
  double b_a;
  double b_t142_tmp;
  double c_a;
  double t100;
  double t100_tmp;
  double t102;
  double t103;
  double t11;
  double t110;
  double t111;
  double t112;
  double t128;
  double t128_tmp;
  double t131;
  double t131_tmp;
  double t132;
  double t132_tmp;
  double t134;
  double t14;
  double t142_tmp;
  double t17;
  double t18;
  double t26;
  double t49;
  double t49_tmp;
  double t5;
  double t50;
  double t51;
  double t52;
  double t52_tmp;
  double t53;
  double t54;
  double t60;
  double t61;
  double t62;
  double t64;
  double t70;
  double t71;
  double t77;
  double t8;
  double t80;
  double t85;
  double t86;
  double t96;
  double t97;
  // OMEGA_COMPUTATION
  //     OMEGA = OMEGA_COMPUTATION(IN1)
  //     This function was generated by the Symbolic Math Toolbox version 9.0.
  //     27-Jan-2022 10:56:34
  a = std::abs(in1[15]);
  t5 = in1[14] * in1[15];
  t8 = in1[11] * in1[16];
  t11 = in1[5] * in1[16];
  t14 = in1[2] * in1[17];
  t17 = in1[8] * in1[17];
  t18 = 1.0 / in1[15];
  t26 = 1.0 / (a * a);
  t49_tmp = in1[17] * t18;
  t49 = t49_tmp * (in1[0] + -in1[6]);
  t50 = t49_tmp * (in1[1] + -in1[7]);
  t51 = t49_tmp * (in1[2] + -in1[8]);
  t52_tmp = in1[16] * t18;
  t52 = t52_tmp * (-in1[3] + in1[9]);
  t53 = t52_tmp * (-in1[4] + in1[10]);
  t54 = t52_tmp * (-in1[5] + in1[11]);
  t60 = (((in1[12] * in1[15] + in1[3] * in1[16]) + in1[0] * in1[17]) +
         -(in1[9] * in1[16])) +
        -(in1[6] * in1[17]);
  t61 = (((in1[13] * in1[15] + in1[4] * in1[16]) + in1[1] * in1[17]) +
         -(in1[10] * in1[16])) +
        -(in1[7] * in1[17]);
  t77 = ((((t8 + t17) + -t5) + -t11) + -t14) + in1[15] * 9.81;
  a_tmp = ((((t5 * 100.0 - t8 * 100.0) + t11 * 100.0) + t14 * 100.0) -
           t17 * 100.0) -
          in1[15] * 981.0;
  t62 = std::abs(t60);
  t11 = std::abs(t61);
  t64 = t60;
  if (t60 < 0.0) {
    t64 = -1.0;
  } else if (t60 > 0.0) {
    t64 = 1.0;
  } else if (t60 == 0.0) {
    t64 = 0.0;
  }
  t14 = t61;
  if (t61 < 0.0) {
    t14 = -1.0;
  } else if (t61 > 0.0) {
    t14 = 1.0;
  } else if (t61 == 0.0) {
    t14 = 0.0;
  }
  t80 = std::abs(t77);
  if (t77 < 0.0) {
    t77 = -1.0;
  } else if (t77 > 0.0) {
    t77 = 1.0;
  } else if (t77 == 0.0) {
    t77 = 0.0;
  }
  t70 = (in1[12] + t49) + -t52;
  t71 = (in1[13] + t50) + -t53;
  t5 = ((in1[14] + t51) + -t54) - 9.81;
  t85 = 1.0 / t5;
  t8 = (t62 * t62 + t11 * t11) + t80 * t80;
  a = std::abs(in1[15] * t70);
  b_a = std::abs(in1[15] * t71);
  t86 = t85 * t85;
  c_a = std::abs(in1[15] * t5);
  t97 = 1.0 / std::sqrt(t60 * t60 * (1.0 / (a_tmp * a_tmp)) * 10000.0 + 1.0);
  t102 = 1.0 / std::sqrt(t8);
  t5 = t70 * t70 * t86;
  t103 = rt_powd_snf(t102, 3.0);
  t96 = 1.0 / (t5 + 1.0);
  t5 = 1.0 / std::sqrt(t5 + 1.0);
  t110 = 1.0 / std::sqrt((a * a + b_a * b_a) + c_a * c_a);
  t100_tmp = in1[18] * t18;
  t100 = t100_tmp * t5;
  t111 = std::sqrt(-(t61 * t61 * (1.0 / t8)) + 1.0);
  t17 = in1[19] * t71 * t5 * t110;
  t131_tmp = in1[16] * t61;
  t131 = in1[16] * t102 + -(t131_tmp * t11 * t14 * t103);
  t132_tmp = in1[17] * t61;
  t132 = in1[17] * t102 + -(t132_tmp * t11 * t14 * t103);
  t112 = 1.0 / t111;
  t128_tmp = in1[20] * t18;
  t128 = t128_tmp * t97 * t111;
  t134 = t60 * t128 * 100.0 / a_tmp;
  t5 = t49_tmp * t70 * t86 * t96;
  t8 = t70 * t85;
  t142_tmp = in1[12] * in1[16];
  b_t142_tmp = in1[14] * in1[16];
  t52 = (in1[12] + t52) + -t49;
  t49 = (in1[14] + t54) + -t51;
  t5 = ((((((t142_tmp * t18 * t85 * t96 + t49_tmp * t85 * t96 * in1[9]) +
            t49_tmp * -in1[3] * t85 * t96) +
           t5 * in1[5]) +
          -(b_t142_tmp * t18 * t70 * t86 * t96)) +
         -(t5 * in1[11])) +
        t52_tmp * t70 * t86 * t96 * (((t49 + -(t8 * t100)) + t128) + -t17)) +
       -(t52_tmp * t85 * t96 * (((t52 + t100) + t134) + t8 * -t17));
  a = std::abs(in1[13]);
  b_a = std::abs(in1[16]);
  c_a = std::abs(in1[10]);
  t8 = std::abs(in1[4]);
  t11 = std::abs(in1[17]);
  t14 = std::abs(in1[1]);
  t17 = std::abs(in1[7]);
  t100 = in1[19] * t18;
  t96 = t132_tmp * t62 * t64 * t103 * t112;
  omega[0] = ((((((((((-t112 * t131 *
                           ((((in1[13] - t50) + t53) + t100 * t111) +
                            t128_tmp * t61 * t102) +
                       in1[13] * t112 * t131) +
                      -in1[4] * t112 * t132) +
                     t112 * t132 * in1[10]) -
                    t142_tmp * t61 * t62 * t64 * t103 * t112) +
                   b_t142_tmp * t61 * t80 * t77 * t103 * t112) +
                  t131_tmp * t62 * t64 * t103 * t112 *
                      (((t52 + t134) + t100_tmp * t97) -
                       t100 * t60 * t61 * t97 * t102 * 100.0 / a_tmp)) -
                 t131_tmp * t80 * t77 * t103 * t112 *
                     (((t49 + t128) - t100 * t61 * t97 * t102) -
                      t100_tmp * t60 * t97 * 100.0 / a_tmp)) +
                in1[17] * -in1[5] * t61 * t80 * t77 * t103 * t112) -
               t96 * in1[9]) +
              t132_tmp * t80 * t77 * t103 * t112 * in1[11]) +
             t96 * in1[3];
  omega[1] =
      -t5 * std::sqrt(-1.0 / (t18 * t18) * (t71 * t71) * (t110 * t110) + 1.0);
  t100 = in1[15] * t26;
}

void omegaComputation(double* in1, double* omega, void* user_data)
{
  double a;
  double a_tmp;
  double b_a;
  double b_t142_tmp;
  double c_a;
  double t100;
  double t100_tmp;
  double t102;
  double t103;
  double t11;
  double t110;
  double t111;
  double t112;
  double t128;
  double t128_tmp;
  double t131;
  double t131_tmp;
  double t132;
  double t132_tmp;
  double t134;
  double t14;
  double t142_tmp;
  double t17;
  double t18;
  double t26;
  double t49;
  double t49_tmp;
  double t5;
  double t50;
  double t51;
  double t52;
  double t52_tmp;
  double t53;
  double t54;
  double t60;
  double t61;
  double t62;
  double t64;
  double t70;
  double t71;
  double t77;
  double t8;
  double t80;
  double t85;
  double t86;
  double t96;
  double t97;
  // OMEGA_COMPUTATION
  //     OMEGA = OMEGA_COMPUTATION(IN1)
  //     This function was generated by the Symbolic Math Toolbox version 9.0.
  //     27-Jan-2022 10:56:34
  a = std::abs(in1[15]);
  t5 = in1[14] * in1[15];
  t8 = in1[11] * in1[16];
  t11 = in1[5] * in1[16];
  t14 = in1[2] * in1[17];
  t17 = in1[8] * in1[17];
  t18 = 1.0 / in1[15];
  t26 = 1.0 / (a * a);
  t49_tmp = in1[17] * t18;
  t49 = t49_tmp * (in1[0] + -in1[6]);
  t50 = t49_tmp * (in1[1] + -in1[7]);
  t51 = t49_tmp * (in1[2] + -in1[8]);
  t52_tmp = in1[16] * t18;
  t52 = t52_tmp * (-in1[3] + in1[9]);
  t53 = t52_tmp * (-in1[4] + in1[10]);
  t54 = t52_tmp * (-in1[5] + in1[11]);
  t60 = (((in1[12] * in1[15] + in1[3] * in1[16]) + in1[0] * in1[17]) +
         -(in1[9] * in1[16])) +
        -(in1[6] * in1[17]);
  t61 = (((in1[13] * in1[15] + in1[4] * in1[16]) + in1[1] * in1[17]) +
         -(in1[10] * in1[16])) +
        -(in1[7] * in1[17]);
  t77 = ((((t8 + t17) + -t5) + -t11) + -t14) + in1[15] * 9.81;
  a_tmp = ((((t5 * 100.0 - t8 * 100.0) + t11 * 100.0) + t14 * 100.0) -
           t17 * 100.0) -
          in1[15] * 981.0;
  t62 = std::abs(t60);
  t11 = std::abs(t61);
  t64 = t60;
  if (t60 < 0.0) {
    t64 = -1.0;
  } else if (t60 > 0.0) {
    t64 = 1.0;
  } else if (t60 == 0.0) {
    t64 = 0.0;
  }
  t14 = t61;
  if (t61 < 0.0) {
    t14 = -1.0;
  } else if (t61 > 0.0) {
    t14 = 1.0;
  } else if (t61 == 0.0) {
    t14 = 0.0;
  }
  t80 = std::abs(t77);
  if (t77 < 0.0) {
    t77 = -1.0;
  } else if (t77 > 0.0) {
    t77 = 1.0;
  } else if (t77 == 0.0) {
    t77 = 0.0;
  }
  t70 = (in1[12] + t49) + -t52;
  t71 = (in1[13] + t50) + -t53;
  t5 = ((in1[14] + t51) + -t54) - 9.81;
  t85 = 1.0 / t5;
  t8 = (t62 * t62 + t11 * t11) + t80 * t80;
  a = std::abs(in1[15] * t70);
  b_a = std::abs(in1[15] * t71);
  t86 = t85 * t85;
  c_a = std::abs(in1[15] * t5);
  t97 = 1.0 / std::sqrt(t60 * t60 * (1.0 / (a_tmp * a_tmp)) * 10000.0 + 1.0);
  t102 = 1.0 / std::sqrt(t8);
  t5 = t70 * t70 * t86;
  t103 = rt_powd_snf(t102, 3.0);
  t96 = 1.0 / (t5 + 1.0);
  t5 = 1.0 / std::sqrt(t5 + 1.0);
  t110 = 1.0 / std::sqrt((a * a + b_a * b_a) + c_a * c_a);
  t100_tmp = in1[18] * t18;
  t100 = t100_tmp * t5;
  t111 = std::sqrt(-(t61 * t61 * (1.0 / t8)) + 1.0);
  t17 = in1[19] * t71 * t5 * t110;
  t131_tmp = in1[16] * t61;
  t131 = in1[16] * t102 + -(t131_tmp * t11 * t14 * t103);
  t132_tmp = in1[17] * t61;
  t132 = in1[17] * t102 + -(t132_tmp * t11 * t14 * t103);
  t112 = 1.0 / t111;
  t128_tmp = in1[20] * t18;
  t128 = t128_tmp * t97 * t111;
  t134 = t60 * t128 * 100.0 / a_tmp;
  t5 = t49_tmp * t70 * t86 * t96;
  t8 = t70 * t85;
  t142_tmp = in1[12] * in1[16];
  b_t142_tmp = in1[14] * in1[16];
  t52 = (in1[12] + t52) + -t49;
  t49 = (in1[14] + t54) + -t51;
  t5 = ((((((t142_tmp * t18 * t85 * t96 + t49_tmp * t85 * t96 * in1[9]) +
            t49_tmp * -in1[3] * t85 * t96) +
           t5 * in1[5]) +
          -(b_t142_tmp * t18 * t70 * t86 * t96)) +
         -(t5 * in1[11])) +
        t52_tmp * t70 * t86 * t96 * (((t49 + -(t8 * t100)) + t128) + -t17)) +
       -(t52_tmp * t85 * t96 * (((t52 + t100) + t134) + t8 * -t17));
  a = std::abs(in1[13]);
  b_a = std::abs(in1[16]);
  c_a = std::abs(in1[10]);
  t8 = std::abs(in1[4]);
  t11 = std::abs(in1[17]);
  t14 = std::abs(in1[1]);
  t17 = std::abs(in1[7]);
  t100 = in1[19] * t18;
  t96 = t132_tmp * t62 * t64 * t103 * t112;
  omega[0] = ((((((((((-t112 * t131 *
                           ((((in1[13] - t50) + t53) + t100 * t111) +
                            t128_tmp * t61 * t102) +
                       in1[13] * t112 * t131) +
                      -in1[4] * t112 * t132) +
                     t112 * t132 * in1[10]) -
                    t142_tmp * t61 * t62 * t64 * t103 * t112) +
                   b_t142_tmp * t61 * t80 * t77 * t103 * t112) +
                  t131_tmp * t62 * t64 * t103 * t112 *
                      (((t52 + t134) + t100_tmp * t97) -
                       t100 * t60 * t61 * t97 * t102 * 100.0 / a_tmp)) -
                 t131_tmp * t80 * t77 * t103 * t112 *
                     (((t49 + t128) - t100 * t61 * t97 * t102) -
                      t100_tmp * t60 * t97 * 100.0 / a_tmp)) +
                in1[17] * -in1[5] * t61 * t80 * t77 * t103 * t112) -
               t96 * in1[9]) +
              t132_tmp * t80 * t77 * t103 * t112 * in1[11]) +
             t96 * in1[3];
  omega[1] =
      -t5 * std::sqrt(-1.0 / (t18 * t18) * (t71 * t71) * (t110 * t110) + 1.0);
  t100 = in1[15] * t26;
}

// End of code generation (omega_computation.cpp)