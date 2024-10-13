/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: PID_MODEL_data.c
 *
 * Code generated for Simulink model 'PID_MODEL'.
 *
 * Model version                  : 2.18
 * Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
 * C/C++ source code generated on : Mon Apr 22 19:38:11 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "PID_MODEL.h"

/* Block parameters (default storage) */
P rtP = {
  /* Variable: ANG_A_D_CH1_1
   * Referenced by: '<S142>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_A_D_CH1_2
   * Referenced by: '<S292>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_A_D_CH1_3
   * Referenced by: '<S442>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_A_D_CH1_4
   * Referenced by: '<S592>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_A_D_CH1_5
   * Referenced by: '<S742>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_A_D_CH1_6
   * Referenced by: '<S892>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_A_D_CH1_7
   * Referenced by: '<S1042>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_A_D_CH2_1
   * Referenced by: '<S1192>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_A_D_CH2_2
   * Referenced by: '<S1342>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_A_D_CH2_3
   * Referenced by: '<S1492>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_A_D_CH2_4
   * Referenced by: '<S1642>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_A_D_CH2_5
   * Referenced by: '<S1792>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_A_D_CH2_6
   * Referenced by: '<S1942>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_A_D_CH2_7
   * Referenced by: '<S2092>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_A_I_CH1_1
   * Referenced by: '<S145>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_A_I_CH1_2
   * Referenced by: '<S295>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_A_I_CH1_3
   * Referenced by: '<S445>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_A_I_CH1_4
   * Referenced by: '<S595>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_A_I_CH1_5
   * Referenced by: '<S745>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_A_I_CH1_6
   * Referenced by: '<S895>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_A_I_CH1_7
   * Referenced by: '<S1045>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_A_I_CH2_1
   * Referenced by: '<S1195>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_A_I_CH2_2
   * Referenced by: '<S1345>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_A_I_CH2_3
   * Referenced by: '<S1495>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_A_I_CH2_4
   * Referenced by: '<S1645>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_A_I_CH2_5
   * Referenced by: '<S1795>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_A_I_CH2_6
   * Referenced by: '<S1945>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_A_I_CH2_7
   * Referenced by: '<S2095>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_A_P_CH1_1
   * Referenced by: '<S153>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_A_P_CH1_2
   * Referenced by: '<S303>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_A_P_CH1_3
   * Referenced by: '<S453>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_A_P_CH1_4
   * Referenced by: '<S603>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_A_P_CH1_5
   * Referenced by: '<S753>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_A_P_CH1_6
   * Referenced by: '<S903>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_A_P_CH1_7
   * Referenced by: '<S1053>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_A_P_CH2_1
   * Referenced by: '<S1203>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_A_P_CH2_2
   * Referenced by: '<S1353>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_A_P_CH2_3
   * Referenced by: '<S1503>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_A_P_CH2_4
   * Referenced by: '<S1653>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_A_P_CH2_5
   * Referenced by: '<S1803>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_A_P_CH2_6
   * Referenced by: '<S1953>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_A_P_CH2_7
   * Referenced by: '<S2103>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_S_D_CH1_1
   * Referenced by: '<S94>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_S_D_CH1_2
   * Referenced by: '<S244>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_S_D_CH1_3
   * Referenced by: '<S394>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_S_D_CH1_4
   * Referenced by: '<S544>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_S_D_CH1_5
   * Referenced by: '<S694>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_S_D_CH1_6
   * Referenced by: '<S844>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_S_D_CH1_7
   * Referenced by: '<S994>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_S_D_CH2_1
   * Referenced by: '<S1144>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_S_D_CH2_2
   * Referenced by: '<S1294>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_S_D_CH2_3
   * Referenced by: '<S1444>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_S_D_CH2_4
   * Referenced by: '<S1594>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_S_D_CH2_5
   * Referenced by: '<S1744>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_S_D_CH2_6
   * Referenced by: '<S1894>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_S_D_CH2_7
   * Referenced by: '<S2044>/Derivative Gain'
   */
  0.0F,

  /* Variable: ANG_S_I_CH1_1
   * Referenced by: '<S97>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_S_I_CH1_2
   * Referenced by: '<S247>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_S_I_CH1_3
   * Referenced by: '<S397>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_S_I_CH1_4
   * Referenced by: '<S547>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_S_I_CH1_5
   * Referenced by: '<S697>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_S_I_CH1_6
   * Referenced by: '<S847>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_S_I_CH1_7
   * Referenced by: '<S997>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_S_I_CH2_1
   * Referenced by: '<S1147>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_S_I_CH2_2
   * Referenced by: '<S1297>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_S_I_CH2_3
   * Referenced by: '<S1447>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_S_I_CH2_4
   * Referenced by: '<S1597>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_S_I_CH2_5
   * Referenced by: '<S1747>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_S_I_CH2_6
   * Referenced by: '<S1897>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_S_I_CH2_7
   * Referenced by: '<S2047>/Integral Gain'
   */
  0.0F,

  /* Variable: ANG_S_P_CH1_1
   * Referenced by: '<S105>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_S_P_CH1_2
   * Referenced by: '<S255>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_S_P_CH1_3
   * Referenced by: '<S405>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_S_P_CH1_4
   * Referenced by: '<S555>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_S_P_CH1_5
   * Referenced by: '<S705>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_S_P_CH1_6
   * Referenced by: '<S855>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_S_P_CH1_7
   * Referenced by: '<S1005>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_S_P_CH2_1
   * Referenced by: '<S1155>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_S_P_CH2_2
   * Referenced by: '<S1305>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_S_P_CH2_3
   * Referenced by: '<S1455>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_S_P_CH2_4
   * Referenced by: '<S1605>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_S_P_CH2_5
   * Referenced by: '<S1755>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_S_P_CH2_6
   * Referenced by: '<S1905>/Proportional Gain'
   */
  0.0F,

  /* Variable: ANG_S_P_CH2_7
   * Referenced by: '<S2055>/Proportional Gain'
   */
  0.0F,

  /* Variable: DEADBAND_CH1_1
   * Referenced by:
   *   '<S16>/Switch1'
   *   '<S16>/Switch2'
   */
  500.0F,

  /* Variable: DEADBAND_CH1_2
   * Referenced by:
   *   '<S166>/Switch1'
   *   '<S166>/Switch2'
   */
  500.0F,

  /* Variable: DEADBAND_CH1_3
   * Referenced by:
   *   '<S316>/Switch1'
   *   '<S316>/Switch2'
   */
  500.0F,

  /* Variable: DEADBAND_CH1_4
   * Referenced by:
   *   '<S466>/Switch1'
   *   '<S466>/Switch2'
   */
  500.0F,

  /* Variable: DEADBAND_CH1_5
   * Referenced by:
   *   '<S616>/Switch1'
   *   '<S616>/Switch2'
   */
  500.0F,

  /* Variable: DEADBAND_CH1_6
   * Referenced by:
   *   '<S766>/Switch1'
   *   '<S766>/Switch2'
   */
  500.0F,

  /* Variable: DEADBAND_CH1_7
   * Referenced by:
   *   '<S916>/Switch1'
   *   '<S916>/Switch2'
   */
  500.0F,

  /* Variable: DEADBAND_CH2_1
   * Referenced by:
   *   '<S1066>/Switch1'
   *   '<S1066>/Switch2'
   */
  500.0F,

  /* Variable: DEADBAND_CH2_2
   * Referenced by:
   *   '<S1216>/Switch1'
   *   '<S1216>/Switch2'
   */
  500.0F,

  /* Variable: DEADBAND_CH2_3
   * Referenced by:
   *   '<S1366>/Switch1'
   *   '<S1366>/Switch2'
   */
  500.0F,

  /* Variable: DEADBAND_CH2_4
   * Referenced by:
   *   '<S1516>/Switch1'
   *   '<S1516>/Switch2'
   */
  500.0F,

  /* Variable: DEADBAND_CH2_5
   * Referenced by:
   *   '<S1666>/Switch1'
   *   '<S1666>/Switch2'
   */
  500.0F,

  /* Variable: DEADBAND_CH2_6
   * Referenced by:
   *   '<S1816>/Switch1'
   *   '<S1816>/Switch2'
   */
  500.0F,

  /* Variable: DEADBAND_CH2_7
   * Referenced by:
   *   '<S1966>/Switch1'
   *   '<S1966>/Switch2'
   */
  500.0F,

  /* Variable: SPD_D_CH1_1
   * Referenced by: '<S43>/Derivative Gain'
   */
  0.0F,

  /* Variable: SPD_D_CH1_2
   * Referenced by: '<S193>/Derivative Gain'
   */
  0.0F,

  /* Variable: SPD_D_CH1_3
   * Referenced by: '<S343>/Derivative Gain'
   */
  0.0F,

  /* Variable: SPD_D_CH1_4
   * Referenced by: '<S493>/Derivative Gain'
   */
  0.0F,

  /* Variable: SPD_D_CH1_5
   * Referenced by: '<S643>/Derivative Gain'
   */
  0.0F,

  /* Variable: SPD_D_CH1_6
   * Referenced by: '<S793>/Derivative Gain'
   */
  0.0F,

  /* Variable: SPD_D_CH1_7
   * Referenced by: '<S943>/Derivative Gain'
   */
  0.0F,

  /* Variable: SPD_D_CH2_1
   * Referenced by: '<S1093>/Derivative Gain'
   */
  0.0F,

  /* Variable: SPD_D_CH2_2
   * Referenced by: '<S1243>/Derivative Gain'
   */
  0.0F,

  /* Variable: SPD_D_CH2_3
   * Referenced by: '<S1393>/Derivative Gain'
   */
  0.0F,

  /* Variable: SPD_D_CH2_4
   * Referenced by: '<S1543>/Derivative Gain'
   */
  0.0F,

  /* Variable: SPD_D_CH2_5
   * Referenced by: '<S1693>/Derivative Gain'
   */
  0.0F,

  /* Variable: SPD_D_CH2_6
   * Referenced by: '<S1843>/Derivative Gain'
   */
  0.0F,

  /* Variable: SPD_D_CH2_7
   * Referenced by: '<S1993>/Derivative Gain'
   */
  0.0F,

  /* Variable: SPD_I_CH1_1
   * Referenced by: '<S46>/Integral Gain'
   */
  0.0F,

  /* Variable: SPD_I_CH1_2
   * Referenced by: '<S196>/Integral Gain'
   */
  0.0F,

  /* Variable: SPD_I_CH1_3
   * Referenced by: '<S346>/Integral Gain'
   */
  0.0F,

  /* Variable: SPD_I_CH1_4
   * Referenced by: '<S496>/Integral Gain'
   */
  0.0F,

  /* Variable: SPD_I_CH1_5
   * Referenced by: '<S646>/Integral Gain'
   */
  0.0F,

  /* Variable: SPD_I_CH1_6
   * Referenced by: '<S796>/Integral Gain'
   */
  0.0F,

  /* Variable: SPD_I_CH1_7
   * Referenced by: '<S946>/Integral Gain'
   */
  0.0F,

  /* Variable: SPD_I_CH2_1
   * Referenced by: '<S1096>/Integral Gain'
   */
  0.0F,

  /* Variable: SPD_I_CH2_2
   * Referenced by: '<S1246>/Integral Gain'
   */
  0.0F,

  /* Variable: SPD_I_CH2_3
   * Referenced by: '<S1396>/Integral Gain'
   */
  0.0F,

  /* Variable: SPD_I_CH2_4
   * Referenced by: '<S1546>/Integral Gain'
   */
  0.0F,

  /* Variable: SPD_I_CH2_5
   * Referenced by: '<S1696>/Integral Gain'
   */
  0.0F,

  /* Variable: SPD_I_CH2_6
   * Referenced by: '<S1846>/Integral Gain'
   */
  0.0F,

  /* Variable: SPD_I_CH2_7
   * Referenced by: '<S1996>/Integral Gain'
   */
  0.0F,

  /* Variable: SPD_P_CH1_1
   * Referenced by: '<S54>/Proportional Gain'
   */
  0.0F,

  /* Variable: SPD_P_CH1_2
   * Referenced by: '<S204>/Proportional Gain'
   */
  0.0F,

  /* Variable: SPD_P_CH1_3
   * Referenced by: '<S354>/Proportional Gain'
   */
  0.0F,

  /* Variable: SPD_P_CH1_4
   * Referenced by: '<S504>/Proportional Gain'
   */
  0.0F,

  /* Variable: SPD_P_CH1_5
   * Referenced by: '<S654>/Proportional Gain'
   */
  0.0F,

  /* Variable: SPD_P_CH1_6
   * Referenced by: '<S804>/Proportional Gain'
   */
  0.0F,

  /* Variable: SPD_P_CH1_7
   * Referenced by: '<S954>/Proportional Gain'
   */
  0.0F,

  /* Variable: SPD_P_CH2_1
   * Referenced by: '<S1104>/Proportional Gain'
   */
  0.0F,

  /* Variable: SPD_P_CH2_2
   * Referenced by: '<S1254>/Proportional Gain'
   */
  0.0F,

  /* Variable: SPD_P_CH2_3
   * Referenced by: '<S1404>/Proportional Gain'
   */
  0.0F,

  /* Variable: SPD_P_CH2_4
   * Referenced by: '<S1554>/Proportional Gain'
   */
  0.0F,

  /* Variable: SPD_P_CH2_5
   * Referenced by: '<S1704>/Proportional Gain'
   */
  0.0F,

  /* Variable: SPD_P_CH2_6
   * Referenced by: '<S1854>/Proportional Gain'
   */
  0.0F,

  /* Variable: SPD_P_CH2_7
   * Referenced by: '<S2004>/Proportional Gain'
   */
  0.0F,

  /* Variable: TRANS_CH1_1
   * Referenced by: '<S16>/Multiply'
   */
  0.5F,

  /* Variable: TRANS_CH1_2
   * Referenced by: '<S166>/Multiply'
   */
  0.5F,

  /* Variable: TRANS_CH1_3
   * Referenced by: '<S316>/Multiply'
   */
  0.5F,

  /* Variable: TRANS_CH1_4
   * Referenced by: '<S466>/Multiply'
   */
  0.5F,

  /* Variable: TRANS_CH1_5
   * Referenced by: '<S616>/Multiply'
   */
  0.5F,

  /* Variable: TRANS_CH1_6
   * Referenced by: '<S766>/Multiply'
   */
  0.5F,

  /* Variable: TRANS_CH1_7
   * Referenced by: '<S916>/Multiply'
   */
  0.5F,

  /* Variable: TRANS_CH2_1
   * Referenced by: '<S1066>/Multiply'
   */
  0.5F,

  /* Variable: TRANS_CH2_2
   * Referenced by: '<S1216>/Multiply'
   */
  0.5F,

  /* Variable: TRANS_CH2_3
   * Referenced by: '<S1366>/Multiply'
   */
  0.5F,

  /* Variable: TRANS_CH2_4
   * Referenced by: '<S1516>/Multiply'
   */
  0.5F,

  /* Variable: TRANS_CH2_5
   * Referenced by: '<S1666>/Multiply'
   */
  0.5F,

  /* Variable: TRANS_CH2_6
   * Referenced by: '<S1816>/Multiply'
   */
  0.5F,

  /* Variable: TRANS_CH2_7
   * Referenced by: '<S1966>/Multiply'
   */
  0.5F
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
