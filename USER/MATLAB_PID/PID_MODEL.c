/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: PID_MODEL.c
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
#include "rtwtypes.h"
#include <math.h>

/* Named constants for Chart: '<S16>/Chart1' */
#define IN_ADD                         ((uint8_T)1U)
#define IN_IDLE                        ((uint8_T)2U)
#define IN_SUB                         ((uint8_T)3U)

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
static void Chart1_Init(real32_T *rty_u2);
static void Chart1(real32_T rtu_u1, real32_T *rty_u2, DW_Chart1 *localDW);

/*
 * System initialize for atomic system:
 *    '<S16>/Chart1'
 *    '<S166>/Chart1'
 *    '<S316>/Chart1'
 *    '<S466>/Chart1'
 *    '<S616>/Chart1'
 *    '<S766>/Chart1'
 *    '<S916>/Chart1'
 *    '<S1066>/Chart1'
 *    '<S1216>/Chart1'
 *    '<S1366>/Chart1'
 *    ...
 */
static void Chart1_Init(real32_T *rty_u2)
{
  *rty_u2 = 0.0F;
}

/*
 * Output and update for atomic system:
 *    '<S16>/Chart1'
 *    '<S166>/Chart1'
 *    '<S316>/Chart1'
 *    '<S466>/Chart1'
 *    '<S616>/Chart1'
 *    '<S766>/Chart1'
 *    '<S916>/Chart1'
 *    '<S1066>/Chart1'
 *    '<S1216>/Chart1'
 *    '<S1366>/Chart1'
 *    ...
 */
static void Chart1(real32_T rtu_u1, real32_T *rty_u2, DW_Chart1 *localDW)
{
  /* Chart: '<S16>/Chart1' */
  if (localDW->is_active_c1_PID_MODEL == 0U) {
    localDW->is_active_c1_PID_MODEL = 1U;
    localDW->is_c1_PID_MODEL = IN_IDLE;
  } else {
    switch (localDW->is_c1_PID_MODEL) {
     case IN_ADD:
      if (rtu_u1 <= -5000.0F) {
        localDW->is_c1_PID_MODEL = IN_SUB;
        *rty_u2 = -1.0F;
      } else if ((rtu_u1 < 5000.0F) && (rtu_u1 > -5000.0F)) {
        localDW->is_c1_PID_MODEL = IN_IDLE;
      }
      break;

     case IN_IDLE:
      if (rtu_u1 >= 5000.0F) {
        localDW->is_c1_PID_MODEL = IN_ADD;
        *rty_u2 = 1.0F;
      } else if (rtu_u1 <= -5000.0F) {
        localDW->is_c1_PID_MODEL = IN_SUB;
        *rty_u2 = -1.0F;
      } else {
        *rty_u2 = 0.0F;
      }
      break;

     default:
      /* case IN_SUB: */
      if (rtu_u1 >= 5000.0F) {
        localDW->is_c1_PID_MODEL = IN_ADD;
        *rty_u2 = 1.0F;
      } else if ((rtu_u1 < 5000.0F) && (rtu_u1 > -5000.0F)) {
        localDW->is_c1_PID_MODEL = IN_IDLE;
      }
      break;
    }
  }

  /* End of Chart: '<S16>/Chart1' */
}

/* Model step function */
void PID_MODEL_step(void)
{
  real32_T rtb_FilterCoefficient_dy;
  real32_T rtb_FilterCoefficient_e4;
  real32_T rtb_FilterCoefficient_k;
  real32_T rtb_Sum_pp;
  real32_T u0;

  /* SwitchCase: '<S8>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_status_CH2_1'
   */
  switch ((int32_T)rtU.yaw_status_CH2_1) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S8>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S1065>/Action Port'
     */
    /* Sum: '<S1065>/Sum' incorporates:
     *  Inport: '<Root>/yaw_speed_rpm_CH2_1'
     *  Inport: '<Root>/yaw_target_CH2_1'
     */
    rtb_Sum_pp = rtU.yaw_target_CH2_1 - rtU.yaw_speed_rpm_CH2_1;

    /* Gain: '<S1102>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1094>/Filter'
     *  Gain: '<S1093>/Derivative Gain'
     *  Sum: '<S1094>/SumD'
     */
    rtb_FilterCoefficient_k = (rtP.SPD_D_CH2_1 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_k4) * 100.0F;

    /* Sum: '<S1108>/Sum' incorporates:
     *  DiscreteIntegrator: '<S1099>/Integrator'
     *  Gain: '<S1104>/Proportional Gain'
     */
    u0 = (rtP.SPD_P_CH2_1 * rtb_Sum_pp + rtDW.Integrator_DSTATE_ee) +
      rtb_FilterCoefficient_k;

    /* Saturate: '<S1106>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_1' */
      rtY.yaw_SPD_OUT_CH2_1 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_1' */
      rtY.yaw_SPD_OUT_CH2_1 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_1' */
      rtY.yaw_SPD_OUT_CH2_1 = u0;
    }

    /* End of Saturate: '<S1106>/Saturation' */

    /* Update for DiscreteIntegrator: '<S1094>/Filter' */
    rtDW.Filter_DSTATE_k4 += 0.001F * rtb_FilterCoefficient_k;

    /* Update for DiscreteIntegrator: '<S1099>/Integrator' incorporates:
     *  Gain: '<S1096>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_ee += rtP.SPD_I_CH2_1 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S8>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S8>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S1066>/Action Port'
     */
    /* Sum: '<S1066>/Sum' incorporates:
     *  Gain: '<S1066>/Gain'
     *  Inport: '<Root>/yaw_circle_CH2_1'
     *  Inport: '<Root>/yaw_ecd_CH2_1'
     *  Inport: '<Root>/yaw_target_CH2_1'
     *  Sum: '<S1066>/Sum2'
     */
    rtb_FilterCoefficient_k = rtU.yaw_target_CH2_1 - (8191.0F *
      rtU.yaw_circle_CH2_1 + rtU.yaw_ecd_CH2_1);

    /* Abs: '<S1066>/Abs' */
    rtb_Sum_pp = (real32_T)fabs(rtb_FilterCoefficient_k);

    /* Switch: '<S1066>/Switch2' incorporates:
     *  Constant: '<S1066>/Constant'
     */
    if (!(rtb_Sum_pp > rtP.DEADBAND_CH2_1)) {
      rtb_FilterCoefficient_k = 0.0F;
    }

    /* End of Switch: '<S1066>/Switch2' */

    /* Gain: '<S1201>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1193>/Filter'
     *  Gain: '<S1192>/Derivative Gain'
     *  Sum: '<S1193>/SumD'
     */
    rtb_FilterCoefficient_dy = (rtP.ANG_A_D_CH2_1 * rtb_FilterCoefficient_k -
      rtDW.Filter_DSTATE_o2) * 100.0F;

    /* Switch: '<S1066>/Switch1' incorporates:
     *  Constant: '<S1066>/Constant'
     *  Gain: '<S1066>/Multiply'
     *  Inport: '<Root>/yaw_speed_rpm_CH2_1'
     *  Saturate: '<S1205>/Saturation'
     *  Sum: '<S1066>/Sum1'
     */
    if (rtb_Sum_pp > rtP.DEADBAND_CH2_1) {
      /* Sum: '<S1207>/Sum' incorporates:
       *  DiscreteIntegrator: '<S1198>/Integrator'
       *  Gain: '<S1203>/Proportional Gain'
       */
      u0 = (rtP.ANG_A_P_CH2_1 * rtb_FilterCoefficient_k +
            rtDW.Integrator_DSTATE_c) + rtb_FilterCoefficient_dy;

      /* Saturate: '<S1205>/Saturation' */
      if (u0 > 16384.0F) {
        u0 = 16384.0F;
      } else if (u0 < -16384.0F) {
        u0 = -16384.0F;
      }

      rtb_Sum_pp = rtP.TRANS_CH2_1 * u0 - rtU.yaw_speed_rpm_CH2_1;
    } else {
      rtb_Sum_pp = 0.0F;
    }

    /* End of Switch: '<S1066>/Switch1' */

    /* Gain: '<S1153>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1145>/Filter'
     *  Gain: '<S1144>/Derivative Gain'
     *  Sum: '<S1145>/SumD'
     */
    rtb_FilterCoefficient_e4 = (rtP.ANG_S_D_CH2_1 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_nk) * 100.0F;

    /* Sum: '<S1159>/Sum' incorporates:
     *  DiscreteIntegrator: '<S1150>/Integrator'
     *  Gain: '<S1155>/Proportional Gain'
     */
    u0 = (rtP.ANG_S_P_CH2_1 * rtb_Sum_pp + rtDW.Integrator_DSTATE_if) +
      rtb_FilterCoefficient_e4;

    /* Saturate: '<S1157>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_1' */
      rtY.yaw_ANG_OUT_CH2_1 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_1' */
      rtY.yaw_ANG_OUT_CH2_1 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_1' */
      rtY.yaw_ANG_OUT_CH2_1 = u0;
    }

    /* End of Saturate: '<S1157>/Saturation' */

    /* Chart: '<S1066>/Chart1' incorporates:
     *  Inport: '<Root>/yaw_ecd_CH2_1'
     *  Inport: '<Root>/yaw_last_ecd_CH2_1'
     *  Sum: '<S1066>/Sum3'
     */
    Chart1(rtU.yaw_ecd_CH2_1 - rtU.yaw_last_ecd_CH2_1, &rtDW.u2_o,
           &rtDW.sf_Chart1_d);

    /* Outport: '<Root>/yaw_circle_out_CH2_1' incorporates:
     *  Inport: '<Root>/yaw_circle_CH2_1'
     *  Sum: '<S1066>/Sum4'
     */
    rtY.yaw_circle_out_CH2_1 = rtDW.u2_o + rtU.yaw_circle_CH2_1;

    /* Update for DiscreteIntegrator: '<S1198>/Integrator' incorporates:
     *  Gain: '<S1195>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_c += rtP.ANG_A_I_CH2_1 * rtb_FilterCoefficient_k *
      0.001F;

    /* Update for DiscreteIntegrator: '<S1193>/Filter' */
    rtDW.Filter_DSTATE_o2 += 0.001F * rtb_FilterCoefficient_dy;

    /* Update for DiscreteIntegrator: '<S1145>/Filter' */
    rtDW.Filter_DSTATE_nk += 0.001F * rtb_FilterCoefficient_e4;

    /* Update for DiscreteIntegrator: '<S1150>/Integrator' incorporates:
     *  Gain: '<S1147>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_if += rtP.ANG_S_I_CH2_1 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S8>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S8>/Switch Case' */

  /* SwitchCase: '<S1>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_status_CH1_1'
   */
  switch ((int32_T)rtU.yaw_status_CH1_1) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S1>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S15>/Action Port'
     */
    /* Sum: '<S15>/Sum' incorporates:
     *  Inport: '<Root>/yaw_speed_rpm_CH1_1'
     *  Inport: '<Root>/yaw_target_CH1_1'
     */
    rtb_Sum_pp = rtU.yaw_target_CH1_1 - rtU.yaw_speed_rpm_CH1_1;

    /* Gain: '<S52>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S44>/Filter'
     *  Gain: '<S43>/Derivative Gain'
     *  Sum: '<S44>/SumD'
     */
    rtb_FilterCoefficient_k = (rtP.SPD_D_CH1_1 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_ks) * 100.0F;

    /* Sum: '<S58>/Sum' incorporates:
     *  DiscreteIntegrator: '<S49>/Integrator'
     *  Gain: '<S54>/Proportional Gain'
     */
    u0 = (rtP.SPD_P_CH1_1 * rtb_Sum_pp + rtDW.Integrator_DSTATE_lm) +
      rtb_FilterCoefficient_k;

    /* Saturate: '<S56>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_1' */
      rtY.yaw_SPD_OUT_CH1_1 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_1' */
      rtY.yaw_SPD_OUT_CH1_1 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_1' */
      rtY.yaw_SPD_OUT_CH1_1 = u0;
    }

    /* End of Saturate: '<S56>/Saturation' */

    /* Update for DiscreteIntegrator: '<S44>/Filter' */
    rtDW.Filter_DSTATE_ks += 0.001F * rtb_FilterCoefficient_k;

    /* Update for DiscreteIntegrator: '<S49>/Integrator' incorporates:
     *  Gain: '<S46>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_lm += rtP.SPD_I_CH1_1 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S1>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S1>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S16>/Action Port'
     */
    /* Sum: '<S16>/Sum' incorporates:
     *  Gain: '<S16>/Gain'
     *  Inport: '<Root>/yaw_circle_CH1_1'
     *  Inport: '<Root>/yaw_ecd_CH1_1'
     *  Inport: '<Root>/yaw_target_CH1_1'
     *  Sum: '<S16>/Sum2'
     */
    rtb_FilterCoefficient_k = rtU.yaw_target_CH1_1 - (8191.0F *
      rtU.yaw_circle_CH1_1 + rtU.yaw_ecd_CH1_1);

    /* Abs: '<S16>/Abs' */
    rtb_Sum_pp = (real32_T)fabs(rtb_FilterCoefficient_k);

    /* Switch: '<S16>/Switch2' incorporates:
     *  Constant: '<S16>/Constant'
     */
    if (!(rtb_Sum_pp > rtP.DEADBAND_CH1_1)) {
      rtb_FilterCoefficient_k = 0.0F;
    }

    /* End of Switch: '<S16>/Switch2' */

    /* Gain: '<S151>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S143>/Filter'
     *  Gain: '<S142>/Derivative Gain'
     *  Sum: '<S143>/SumD'
     */
    rtb_FilterCoefficient_dy = (rtP.ANG_A_D_CH1_1 * rtb_FilterCoefficient_k -
      rtDW.Filter_DSTATE_hs) * 100.0F;

    /* Switch: '<S16>/Switch1' incorporates:
     *  Constant: '<S16>/Constant'
     *  Gain: '<S16>/Multiply'
     *  Inport: '<Root>/yaw_speed_rpm_CH1_1'
     *  Saturate: '<S155>/Saturation'
     *  Sum: '<S16>/Sum1'
     */
    if (rtb_Sum_pp > rtP.DEADBAND_CH1_1) {
      /* Sum: '<S157>/Sum' incorporates:
       *  DiscreteIntegrator: '<S148>/Integrator'
       *  Gain: '<S153>/Proportional Gain'
       */
      u0 = (rtP.ANG_A_P_CH1_1 * rtb_FilterCoefficient_k +
            rtDW.Integrator_DSTATE_ag) + rtb_FilterCoefficient_dy;

      /* Saturate: '<S155>/Saturation' */
      if (u0 > 16384.0F) {
        u0 = 16384.0F;
      } else if (u0 < -16384.0F) {
        u0 = -16384.0F;
      }

      rtb_Sum_pp = rtP.TRANS_CH1_1 * u0 - rtU.yaw_speed_rpm_CH1_1;
    } else {
      rtb_Sum_pp = 0.0F;
    }

    /* End of Switch: '<S16>/Switch1' */

    /* Gain: '<S103>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S95>/Filter'
     *  Gain: '<S94>/Derivative Gain'
     *  Sum: '<S95>/SumD'
     */
    rtb_FilterCoefficient_e4 = (rtP.ANG_S_D_CH1_1 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_jw) * 100.0F;

    /* Sum: '<S109>/Sum' incorporates:
     *  DiscreteIntegrator: '<S100>/Integrator'
     *  Gain: '<S105>/Proportional Gain'
     */
    u0 = (rtP.ANG_S_P_CH1_1 * rtb_Sum_pp + rtDW.Integrator_DSTATE_l4) +
      rtb_FilterCoefficient_e4;

    /* Saturate: '<S107>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_1' */
      rtY.yaw_ANG_OUT_CH1_1 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_1' */
      rtY.yaw_ANG_OUT_CH1_1 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_1' */
      rtY.yaw_ANG_OUT_CH1_1 = u0;
    }

    /* End of Saturate: '<S107>/Saturation' */

    /* Chart: '<S16>/Chart1' incorporates:
     *  Inport: '<Root>/yaw_ecd_CH1_1'
     *  Inport: '<Root>/yaw_last_ecd_CH1_1'
     *  Sum: '<S16>/Sum3'
     */
    Chart1(rtU.yaw_ecd_CH1_1 - rtU.yaw_last_ecd_CH1_1, &rtDW.u2_ae,
           &rtDW.sf_Chart1);

    /* Outport: '<Root>/yaw_circle_out_CH1_1' incorporates:
     *  Inport: '<Root>/yaw_circle_CH1_1'
     *  Sum: '<S16>/Sum4'
     */
    rtY.yaw_circle_out_CH1_1 = rtDW.u2_ae + rtU.yaw_circle_CH1_1;

    /* Update for DiscreteIntegrator: '<S148>/Integrator' incorporates:
     *  Gain: '<S145>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_ag += rtP.ANG_A_I_CH1_1 * rtb_FilterCoefficient_k *
      0.001F;

    /* Update for DiscreteIntegrator: '<S143>/Filter' */
    rtDW.Filter_DSTATE_hs += 0.001F * rtb_FilterCoefficient_dy;

    /* Update for DiscreteIntegrator: '<S95>/Filter' */
    rtDW.Filter_DSTATE_jw += 0.001F * rtb_FilterCoefficient_e4;

    /* Update for DiscreteIntegrator: '<S100>/Integrator' incorporates:
     *  Gain: '<S97>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_l4 += rtP.ANG_S_I_CH1_1 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S1>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S1>/Switch Case' */

  /* SwitchCase: '<S2>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_status_CH1_2'
   */
  switch ((int32_T)rtU.yaw_status_CH1_2) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S2>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S165>/Action Port'
     */
    /* Sum: '<S165>/Sum' incorporates:
     *  Inport: '<Root>/yaw_speed_rpm_CH1_2'
     *  Inport: '<Root>/yaw_target_CH1_2'
     */
    rtb_Sum_pp = rtU.yaw_target_CH1_2 - rtU.yaw_speed_rpm_CH1_2;

    /* Gain: '<S202>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S194>/Filter'
     *  Gain: '<S193>/Derivative Gain'
     *  Sum: '<S194>/SumD'
     */
    rtb_FilterCoefficient_k = (rtP.SPD_D_CH1_2 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_gi) * 100.0F;

    /* Sum: '<S208>/Sum' incorporates:
     *  DiscreteIntegrator: '<S199>/Integrator'
     *  Gain: '<S204>/Proportional Gain'
     */
    u0 = (rtP.SPD_P_CH1_2 * rtb_Sum_pp + rtDW.Integrator_DSTATE_mwe) +
      rtb_FilterCoefficient_k;

    /* Saturate: '<S206>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_2' */
      rtY.yaw_SPD_OUT_CH1_2 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_2' */
      rtY.yaw_SPD_OUT_CH1_2 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_2' */
      rtY.yaw_SPD_OUT_CH1_2 = u0;
    }

    /* End of Saturate: '<S206>/Saturation' */

    /* Update for DiscreteIntegrator: '<S194>/Filter' */
    rtDW.Filter_DSTATE_gi += 0.001F * rtb_FilterCoefficient_k;

    /* Update for DiscreteIntegrator: '<S199>/Integrator' incorporates:
     *  Gain: '<S196>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_mwe += rtP.SPD_I_CH1_2 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S2>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S2>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S166>/Action Port'
     */
    /* Sum: '<S166>/Sum' incorporates:
     *  Gain: '<S166>/Gain'
     *  Inport: '<Root>/yaw_circle_CH1_2'
     *  Inport: '<Root>/yaw_ecd_CH1_2'
     *  Inport: '<Root>/yaw_target_CH1_2'
     *  Sum: '<S166>/Sum2'
     */
    rtb_FilterCoefficient_k = rtU.yaw_target_CH1_2 - (8191.0F *
      rtU.yaw_circle_CH1_2 + rtU.yaw_ecd_CH1_2);

    /* Abs: '<S166>/Abs' */
    rtb_Sum_pp = (real32_T)fabs(rtb_FilterCoefficient_k);

    /* Switch: '<S166>/Switch2' incorporates:
     *  Constant: '<S166>/Constant'
     */
    if (!(rtb_Sum_pp > rtP.DEADBAND_CH1_2)) {
      rtb_FilterCoefficient_k = 0.0F;
    }

    /* End of Switch: '<S166>/Switch2' */

    /* Gain: '<S301>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S293>/Filter'
     *  Gain: '<S292>/Derivative Gain'
     *  Sum: '<S293>/SumD'
     */
    rtb_FilterCoefficient_dy = (rtP.ANG_A_D_CH1_2 * rtb_FilterCoefficient_k -
      rtDW.Filter_DSTATE_co) * 100.0F;

    /* Switch: '<S166>/Switch1' incorporates:
     *  Constant: '<S166>/Constant'
     *  Gain: '<S166>/Multiply'
     *  Inport: '<Root>/yaw_speed_rpm_CH1_2'
     *  Saturate: '<S305>/Saturation'
     *  Sum: '<S166>/Sum1'
     */
    if (rtb_Sum_pp > rtP.DEADBAND_CH1_2) {
      /* Sum: '<S307>/Sum' incorporates:
       *  DiscreteIntegrator: '<S298>/Integrator'
       *  Gain: '<S303>/Proportional Gain'
       */
      u0 = (rtP.ANG_A_P_CH1_2 * rtb_FilterCoefficient_k +
            rtDW.Integrator_DSTATE_hk) + rtb_FilterCoefficient_dy;

      /* Saturate: '<S305>/Saturation' */
      if (u0 > 16384.0F) {
        u0 = 16384.0F;
      } else if (u0 < -16384.0F) {
        u0 = -16384.0F;
      }

      rtb_Sum_pp = rtP.TRANS_CH1_2 * u0 - rtU.yaw_speed_rpm_CH1_2;
    } else {
      rtb_Sum_pp = 0.0F;
    }

    /* End of Switch: '<S166>/Switch1' */

    /* Gain: '<S253>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S245>/Filter'
     *  Gain: '<S244>/Derivative Gain'
     *  Sum: '<S245>/SumD'
     */
    rtb_FilterCoefficient_e4 = (rtP.ANG_S_D_CH1_2 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_eg) * 100.0F;

    /* Sum: '<S259>/Sum' incorporates:
     *  DiscreteIntegrator: '<S250>/Integrator'
     *  Gain: '<S255>/Proportional Gain'
     */
    u0 = (rtP.ANG_S_P_CH1_2 * rtb_Sum_pp + rtDW.Integrator_DSTATE_ac) +
      rtb_FilterCoefficient_e4;

    /* Saturate: '<S257>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_2' */
      rtY.yaw_ANG_OUT_CH1_2 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_2' */
      rtY.yaw_ANG_OUT_CH1_2 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_2' */
      rtY.yaw_ANG_OUT_CH1_2 = u0;
    }

    /* End of Saturate: '<S257>/Saturation' */

    /* Chart: '<S166>/Chart1' incorporates:
     *  Inport: '<Root>/yaw_ecd_CH1_2'
     *  Inport: '<Root>/yaw_last_ecd_CH1_2'
     *  Sum: '<S166>/Sum3'
     */
    Chart1(rtU.yaw_ecd_CH1_2 - rtU.yaw_last_ecd_CH1_2, &rtDW.u2_m,
           &rtDW.sf_Chart1_l);

    /* Outport: '<Root>/yaw_circle_out_CH1_2' incorporates:
     *  Inport: '<Root>/yaw_circle_CH1_2'
     *  Sum: '<S166>/Sum4'
     */
    rtY.yaw_circle_out_CH1_2 = rtDW.u2_m + rtU.yaw_circle_CH1_2;

    /* Update for DiscreteIntegrator: '<S298>/Integrator' incorporates:
     *  Gain: '<S295>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_hk += rtP.ANG_A_I_CH1_2 * rtb_FilterCoefficient_k *
      0.001F;

    /* Update for DiscreteIntegrator: '<S293>/Filter' */
    rtDW.Filter_DSTATE_co += 0.001F * rtb_FilterCoefficient_dy;

    /* Update for DiscreteIntegrator: '<S245>/Filter' */
    rtDW.Filter_DSTATE_eg += 0.001F * rtb_FilterCoefficient_e4;

    /* Update for DiscreteIntegrator: '<S250>/Integrator' incorporates:
     *  Gain: '<S247>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_ac += rtP.ANG_S_I_CH1_2 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S2>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S2>/Switch Case' */

  /* SwitchCase: '<S3>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_status_CH1_3'
   */
  switch ((int32_T)rtU.yaw_status_CH1_3) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S3>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S315>/Action Port'
     */
    /* Sum: '<S315>/Sum' incorporates:
     *  Inport: '<Root>/yaw_speed_rpm_CH1_3'
     *  Inport: '<Root>/yaw_target_CH1_3'
     */
    rtb_Sum_pp = rtU.yaw_target_CH1_3 - rtU.yaw_speed_rpm_CH1_3;

    /* Gain: '<S352>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S344>/Filter'
     *  Gain: '<S343>/Derivative Gain'
     *  Sum: '<S344>/SumD'
     */
    rtb_FilterCoefficient_k = (rtP.SPD_D_CH1_3 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_oa) * 100.0F;

    /* Sum: '<S358>/Sum' incorporates:
     *  DiscreteIntegrator: '<S349>/Integrator'
     *  Gain: '<S354>/Proportional Gain'
     */
    u0 = (rtP.SPD_P_CH1_3 * rtb_Sum_pp + rtDW.Integrator_DSTATE_cf) +
      rtb_FilterCoefficient_k;

    /* Saturate: '<S356>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_3' */
      rtY.yaw_SPD_OUT_CH1_3 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_3' */
      rtY.yaw_SPD_OUT_CH1_3 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_3' */
      rtY.yaw_SPD_OUT_CH1_3 = u0;
    }

    /* End of Saturate: '<S356>/Saturation' */

    /* Update for DiscreteIntegrator: '<S344>/Filter' */
    rtDW.Filter_DSTATE_oa += 0.001F * rtb_FilterCoefficient_k;

    /* Update for DiscreteIntegrator: '<S349>/Integrator' incorporates:
     *  Gain: '<S346>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_cf += rtP.SPD_I_CH1_3 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S3>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S3>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S316>/Action Port'
     */
    /* Sum: '<S316>/Sum' incorporates:
     *  Gain: '<S316>/Gain'
     *  Inport: '<Root>/yaw_circle_CH1_3'
     *  Inport: '<Root>/yaw_ecd_CH1_3'
     *  Inport: '<Root>/yaw_target_CH1_3'
     *  Sum: '<S316>/Sum2'
     */
    rtb_FilterCoefficient_k = rtU.yaw_target_CH1_3 - (8191.0F *
      rtU.yaw_circle_CH1_3 + rtU.yaw_ecd_CH1_3);

    /* Abs: '<S316>/Abs' */
    rtb_Sum_pp = (real32_T)fabs(rtb_FilterCoefficient_k);

    /* Switch: '<S316>/Switch2' incorporates:
     *  Constant: '<S316>/Constant'
     */
    if (!(rtb_Sum_pp > rtP.DEADBAND_CH1_3)) {
      rtb_FilterCoefficient_k = 0.0F;
    }

    /* End of Switch: '<S316>/Switch2' */

    /* Gain: '<S451>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S443>/Filter'
     *  Gain: '<S442>/Derivative Gain'
     *  Sum: '<S443>/SumD'
     */
    rtb_FilterCoefficient_dy = (rtP.ANG_A_D_CH1_3 * rtb_FilterCoefficient_k -
      rtDW.Filter_DSTATE_as) * 100.0F;

    /* Switch: '<S316>/Switch1' incorporates:
     *  Constant: '<S316>/Constant'
     *  Gain: '<S316>/Multiply'
     *  Inport: '<Root>/yaw_speed_rpm_CH1_3'
     *  Saturate: '<S455>/Saturation'
     *  Sum: '<S316>/Sum1'
     */
    if (rtb_Sum_pp > rtP.DEADBAND_CH1_3) {
      /* Sum: '<S457>/Sum' incorporates:
       *  DiscreteIntegrator: '<S448>/Integrator'
       *  Gain: '<S453>/Proportional Gain'
       */
      u0 = (rtP.ANG_A_P_CH1_3 * rtb_FilterCoefficient_k +
            rtDW.Integrator_DSTATE_ie) + rtb_FilterCoefficient_dy;

      /* Saturate: '<S455>/Saturation' */
      if (u0 > 16384.0F) {
        u0 = 16384.0F;
      } else if (u0 < -16384.0F) {
        u0 = -16384.0F;
      }

      rtb_Sum_pp = rtP.TRANS_CH1_3 * u0 - rtU.yaw_speed_rpm_CH1_3;
    } else {
      rtb_Sum_pp = 0.0F;
    }

    /* End of Switch: '<S316>/Switch1' */

    /* Gain: '<S403>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S395>/Filter'
     *  Gain: '<S394>/Derivative Gain'
     *  Sum: '<S395>/SumD'
     */
    rtb_FilterCoefficient_e4 = (rtP.ANG_S_D_CH1_3 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_kc) * 100.0F;

    /* Sum: '<S409>/Sum' incorporates:
     *  DiscreteIntegrator: '<S400>/Integrator'
     *  Gain: '<S405>/Proportional Gain'
     */
    u0 = (rtP.ANG_S_P_CH1_3 * rtb_Sum_pp + rtDW.Integrator_DSTATE_cw) +
      rtb_FilterCoefficient_e4;

    /* Saturate: '<S407>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_3' */
      rtY.yaw_ANG_OUT_CH1_3 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_3' */
      rtY.yaw_ANG_OUT_CH1_3 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_3' */
      rtY.yaw_ANG_OUT_CH1_3 = u0;
    }

    /* End of Saturate: '<S407>/Saturation' */

    /* Chart: '<S316>/Chart1' incorporates:
     *  Inport: '<Root>/yaw_ecd_CH1_3'
     *  Inport: '<Root>/yaw_last_ecd_CH1_3'
     *  Sum: '<S316>/Sum3'
     */
    Chart1(rtU.yaw_ecd_CH1_3 - rtU.yaw_last_ecd_CH1_3, &rtDW.u2_nv,
           &rtDW.sf_Chart1_i);

    /* Outport: '<Root>/yaw_circle_out_CH1_3' incorporates:
     *  Inport: '<Root>/yaw_circle_CH1_3'
     *  Sum: '<S316>/Sum4'
     */
    rtY.yaw_circle_out_CH1_3 = rtDW.u2_nv + rtU.yaw_circle_CH1_3;

    /* Update for DiscreteIntegrator: '<S448>/Integrator' incorporates:
     *  Gain: '<S445>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_ie += rtP.ANG_A_I_CH1_3 * rtb_FilterCoefficient_k *
      0.001F;

    /* Update for DiscreteIntegrator: '<S443>/Filter' */
    rtDW.Filter_DSTATE_as += 0.001F * rtb_FilterCoefficient_dy;

    /* Update for DiscreteIntegrator: '<S395>/Filter' */
    rtDW.Filter_DSTATE_kc += 0.001F * rtb_FilterCoefficient_e4;

    /* Update for DiscreteIntegrator: '<S400>/Integrator' incorporates:
     *  Gain: '<S397>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_cw += rtP.ANG_S_I_CH1_3 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S3>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S3>/Switch Case' */

  /* SwitchCase: '<S4>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_status_CH1_4'
   */
  switch ((int32_T)rtU.yaw_status_CH1_4) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S4>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S465>/Action Port'
     */
    /* Sum: '<S465>/Sum' incorporates:
     *  Inport: '<Root>/yaw_speed_rpm_CH1_4'
     *  Inport: '<Root>/yaw_target_CH1_4'
     */
    rtb_Sum_pp = rtU.yaw_target_CH1_4 - rtU.yaw_speed_rpm_CH1_4;

    /* Gain: '<S502>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S494>/Filter'
     *  Gain: '<S493>/Derivative Gain'
     *  Sum: '<S494>/SumD'
     */
    rtb_FilterCoefficient_k = (rtP.SPD_D_CH1_4 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_de) * 100.0F;

    /* Sum: '<S508>/Sum' incorporates:
     *  DiscreteIntegrator: '<S499>/Integrator'
     *  Gain: '<S504>/Proportional Gain'
     */
    u0 = (rtP.SPD_P_CH1_4 * rtb_Sum_pp + rtDW.Integrator_DSTATE_mw) +
      rtb_FilterCoefficient_k;

    /* Saturate: '<S506>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_4' */
      rtY.yaw_SPD_OUT_CH1_4 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_4' */
      rtY.yaw_SPD_OUT_CH1_4 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_4' */
      rtY.yaw_SPD_OUT_CH1_4 = u0;
    }

    /* End of Saturate: '<S506>/Saturation' */

    /* Update for DiscreteIntegrator: '<S494>/Filter' */
    rtDW.Filter_DSTATE_de += 0.001F * rtb_FilterCoefficient_k;

    /* Update for DiscreteIntegrator: '<S499>/Integrator' incorporates:
     *  Gain: '<S496>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_mw += rtP.SPD_I_CH1_4 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S4>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S4>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S466>/Action Port'
     */
    /* Sum: '<S466>/Sum' incorporates:
     *  Gain: '<S466>/Gain'
     *  Inport: '<Root>/yaw_circle_CH1_4'
     *  Inport: '<Root>/yaw_ecd_CH1_4'
     *  Inport: '<Root>/yaw_target_CH1_4'
     *  Sum: '<S466>/Sum2'
     */
    rtb_FilterCoefficient_k = rtU.yaw_target_CH1_4 - (8191.0F *
      rtU.yaw_circle_CH1_4 + rtU.yaw_ecd_CH1_4);

    /* Abs: '<S466>/Abs' */
    rtb_Sum_pp = (real32_T)fabs(rtb_FilterCoefficient_k);

    /* Switch: '<S466>/Switch2' incorporates:
     *  Constant: '<S466>/Constant'
     */
    if (!(rtb_Sum_pp > rtP.DEADBAND_CH1_4)) {
      rtb_FilterCoefficient_k = 0.0F;
    }

    /* End of Switch: '<S466>/Switch2' */

    /* Gain: '<S601>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S593>/Filter'
     *  Gain: '<S592>/Derivative Gain'
     *  Sum: '<S593>/SumD'
     */
    rtb_FilterCoefficient_dy = (rtP.ANG_A_D_CH1_4 * rtb_FilterCoefficient_k -
      rtDW.Filter_DSTATE_pb) * 100.0F;

    /* Switch: '<S466>/Switch1' incorporates:
     *  Constant: '<S466>/Constant'
     *  Gain: '<S466>/Multiply'
     *  Inport: '<Root>/yaw_speed_rpm_CH1_4'
     *  Saturate: '<S605>/Saturation'
     *  Sum: '<S466>/Sum1'
     */
    if (rtb_Sum_pp > rtP.DEADBAND_CH1_4) {
      /* Sum: '<S607>/Sum' incorporates:
       *  DiscreteIntegrator: '<S598>/Integrator'
       *  Gain: '<S603>/Proportional Gain'
       */
      u0 = (rtP.ANG_A_P_CH1_4 * rtb_FilterCoefficient_k +
            rtDW.Integrator_DSTATE_jj) + rtb_FilterCoefficient_dy;

      /* Saturate: '<S605>/Saturation' */
      if (u0 > 16384.0F) {
        u0 = 16384.0F;
      } else if (u0 < -16384.0F) {
        u0 = -16384.0F;
      }

      rtb_Sum_pp = rtP.TRANS_CH1_4 * u0 - rtU.yaw_speed_rpm_CH1_4;
    } else {
      rtb_Sum_pp = 0.0F;
    }

    /* End of Switch: '<S466>/Switch1' */

    /* Gain: '<S553>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S545>/Filter'
     *  Gain: '<S544>/Derivative Gain'
     *  Sum: '<S545>/SumD'
     */
    rtb_FilterCoefficient_e4 = (rtP.ANG_S_D_CH1_4 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_mq) * 100.0F;

    /* Sum: '<S559>/Sum' incorporates:
     *  DiscreteIntegrator: '<S550>/Integrator'
     *  Gain: '<S555>/Proportional Gain'
     */
    u0 = (rtP.ANG_S_P_CH1_4 * rtb_Sum_pp + rtDW.Integrator_DSTATE_bh) +
      rtb_FilterCoefficient_e4;

    /* Saturate: '<S557>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_4' */
      rtY.yaw_ANG_OUT_CH1_4 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_4' */
      rtY.yaw_ANG_OUT_CH1_4 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_4' */
      rtY.yaw_ANG_OUT_CH1_4 = u0;
    }

    /* End of Saturate: '<S557>/Saturation' */

    /* Chart: '<S466>/Chart1' incorporates:
     *  Inport: '<Root>/yaw_ecd_CH1_4'
     *  Inport: '<Root>/yaw_last_ecd_CH1_4'
     *  Sum: '<S466>/Sum3'
     */
    Chart1(rtU.yaw_ecd_CH1_4 - rtU.yaw_last_ecd_CH1_4, &rtDW.u2_f,
           &rtDW.sf_Chart1_j);

    /* Outport: '<Root>/yaw_circle_out_CH1_4' incorporates:
     *  Inport: '<Root>/yaw_circle_CH1_4'
     *  Sum: '<S466>/Sum4'
     */
    rtY.yaw_circle_out_CH1_4 = rtDW.u2_f + rtU.yaw_circle_CH1_4;

    /* Update for DiscreteIntegrator: '<S598>/Integrator' incorporates:
     *  Gain: '<S595>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_jj += rtP.ANG_A_I_CH1_4 * rtb_FilterCoefficient_k *
      0.001F;

    /* Update for DiscreteIntegrator: '<S593>/Filter' */
    rtDW.Filter_DSTATE_pb += 0.001F * rtb_FilterCoefficient_dy;

    /* Update for DiscreteIntegrator: '<S545>/Filter' */
    rtDW.Filter_DSTATE_mq += 0.001F * rtb_FilterCoefficient_e4;

    /* Update for DiscreteIntegrator: '<S550>/Integrator' incorporates:
     *  Gain: '<S547>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_bh += rtP.ANG_S_I_CH1_4 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S4>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S4>/Switch Case' */

  /* SwitchCase: '<S5>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_status_CH1_5'
   */
  switch ((int32_T)rtU.yaw_status_CH1_5) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S5>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S615>/Action Port'
     */
    /* Sum: '<S615>/Sum' incorporates:
     *  Inport: '<Root>/yaw_speed_rpm_CH1_5'
     *  Inport: '<Root>/yaw_target_CH1_5'
     */
    rtb_Sum_pp = rtU.yaw_target_CH1_5 - rtU.yaw_speed_rpm_CH1_5;

    /* Gain: '<S652>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S644>/Filter'
     *  Gain: '<S643>/Derivative Gain'
     *  Sum: '<S644>/SumD'
     */
    rtb_FilterCoefficient_k = (rtP.SPD_D_CH1_5 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_ca) * 100.0F;

    /* Sum: '<S658>/Sum' incorporates:
     *  DiscreteIntegrator: '<S649>/Integrator'
     *  Gain: '<S654>/Proportional Gain'
     */
    u0 = (rtP.SPD_P_CH1_5 * rtb_Sum_pp + rtDW.Integrator_DSTATE_o5) +
      rtb_FilterCoefficient_k;

    /* Saturate: '<S656>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_5' */
      rtY.yaw_SPD_OUT_CH1_5 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_5' */
      rtY.yaw_SPD_OUT_CH1_5 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_5' */
      rtY.yaw_SPD_OUT_CH1_5 = u0;
    }

    /* End of Saturate: '<S656>/Saturation' */

    /* Update for DiscreteIntegrator: '<S644>/Filter' */
    rtDW.Filter_DSTATE_ca += 0.001F * rtb_FilterCoefficient_k;

    /* Update for DiscreteIntegrator: '<S649>/Integrator' incorporates:
     *  Gain: '<S646>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_o5 += rtP.SPD_I_CH1_5 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S5>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S5>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S616>/Action Port'
     */
    /* Sum: '<S616>/Sum' incorporates:
     *  Gain: '<S616>/Gain'
     *  Inport: '<Root>/yaw_circle_CH1_5'
     *  Inport: '<Root>/yaw_ecd_CH1_5'
     *  Inport: '<Root>/yaw_target_CH1_5'
     *  Sum: '<S616>/Sum2'
     */
    rtb_FilterCoefficient_k = rtU.yaw_target_CH1_5 - (8191.0F *
      rtU.yaw_circle_CH1_5 + rtU.yaw_ecd_CH1_5);

    /* Abs: '<S616>/Abs' */
    rtb_Sum_pp = (real32_T)fabs(rtb_FilterCoefficient_k);

    /* Switch: '<S616>/Switch2' incorporates:
     *  Constant: '<S616>/Constant'
     */
    if (!(rtb_Sum_pp > rtP.DEADBAND_CH1_5)) {
      rtb_FilterCoefficient_k = 0.0F;
    }

    /* End of Switch: '<S616>/Switch2' */

    /* Gain: '<S751>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S743>/Filter'
     *  Gain: '<S742>/Derivative Gain'
     *  Sum: '<S743>/SumD'
     */
    rtb_FilterCoefficient_dy = (rtP.ANG_A_D_CH1_5 * rtb_FilterCoefficient_k -
      rtDW.Filter_DSTATE_p) * 100.0F;

    /* Switch: '<S616>/Switch1' incorporates:
     *  Constant: '<S616>/Constant'
     *  Gain: '<S616>/Multiply'
     *  Inport: '<Root>/yaw_speed_rpm_CH1_5'
     *  Saturate: '<S755>/Saturation'
     *  Sum: '<S616>/Sum1'
     */
    if (rtb_Sum_pp > rtP.DEADBAND_CH1_5) {
      /* Sum: '<S757>/Sum' incorporates:
       *  DiscreteIntegrator: '<S748>/Integrator'
       *  Gain: '<S753>/Proportional Gain'
       */
      u0 = (rtP.ANG_A_P_CH1_5 * rtb_FilterCoefficient_k +
            rtDW.Integrator_DSTATE_p) + rtb_FilterCoefficient_dy;

      /* Saturate: '<S755>/Saturation' */
      if (u0 > 16384.0F) {
        u0 = 16384.0F;
      } else if (u0 < -16384.0F) {
        u0 = -16384.0F;
      }

      rtb_Sum_pp = rtP.TRANS_CH1_5 * u0 - rtU.yaw_speed_rpm_CH1_5;
    } else {
      rtb_Sum_pp = 0.0F;
    }

    /* End of Switch: '<S616>/Switch1' */

    /* Gain: '<S703>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S695>/Filter'
     *  Gain: '<S694>/Derivative Gain'
     *  Sum: '<S695>/SumD'
     */
    rtb_FilterCoefficient_e4 = (rtP.ANG_S_D_CH1_5 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_bm) * 100.0F;

    /* Sum: '<S709>/Sum' incorporates:
     *  DiscreteIntegrator: '<S700>/Integrator'
     *  Gain: '<S705>/Proportional Gain'
     */
    u0 = (rtP.ANG_S_P_CH1_5 * rtb_Sum_pp + rtDW.Integrator_DSTATE_ep) +
      rtb_FilterCoefficient_e4;

    /* Saturate: '<S707>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_5' */
      rtY.yaw_ANG_OUT_CH1_5 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_5' */
      rtY.yaw_ANG_OUT_CH1_5 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_5' */
      rtY.yaw_ANG_OUT_CH1_5 = u0;
    }

    /* End of Saturate: '<S707>/Saturation' */

    /* Chart: '<S616>/Chart1' incorporates:
     *  Inport: '<Root>/yaw_ecd_CH1_5'
     *  Inport: '<Root>/yaw_last_ecd_CH1_5'
     *  Sum: '<S616>/Sum3'
     */
    Chart1(rtU.yaw_ecd_CH1_5 - rtU.yaw_last_ecd_CH1_5, &rtDW.u2_c,
           &rtDW.sf_Chart1_n);

    /* Outport: '<Root>/yaw_circle_out_CH1_5' incorporates:
     *  Inport: '<Root>/yaw_circle_CH1_5'
     *  Sum: '<S616>/Sum4'
     */
    rtY.yaw_circle_out_CH1_5 = rtDW.u2_c + rtU.yaw_circle_CH1_5;

    /* Update for DiscreteIntegrator: '<S748>/Integrator' incorporates:
     *  Gain: '<S745>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_p += rtP.ANG_A_I_CH1_5 * rtb_FilterCoefficient_k *
      0.001F;

    /* Update for DiscreteIntegrator: '<S743>/Filter' */
    rtDW.Filter_DSTATE_p += 0.001F * rtb_FilterCoefficient_dy;

    /* Update for DiscreteIntegrator: '<S695>/Filter' */
    rtDW.Filter_DSTATE_bm += 0.001F * rtb_FilterCoefficient_e4;

    /* Update for DiscreteIntegrator: '<S700>/Integrator' incorporates:
     *  Gain: '<S697>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_ep += rtP.ANG_S_I_CH1_5 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S5>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S5>/Switch Case' */

  /* SwitchCase: '<S6>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_status_CH1_6'
   */
  switch ((int32_T)rtU.yaw_status_CH1_6) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S6>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S765>/Action Port'
     */
    /* Sum: '<S765>/Sum' incorporates:
     *  Inport: '<Root>/yaw_speed_rpm_CH1_6'
     *  Inport: '<Root>/yaw_target_CH1_6'
     */
    rtb_Sum_pp = rtU.yaw_target_CH1_6 - rtU.yaw_speed_rpm_CH1_6;

    /* Gain: '<S802>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S794>/Filter'
     *  Gain: '<S793>/Derivative Gain'
     *  Sum: '<S794>/SumD'
     */
    rtb_FilterCoefficient_k = (rtP.SPD_D_CH1_6 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_ke) * 100.0F;

    /* Sum: '<S808>/Sum' incorporates:
     *  DiscreteIntegrator: '<S799>/Integrator'
     *  Gain: '<S804>/Proportional Gain'
     */
    u0 = (rtP.SPD_P_CH1_6 * rtb_Sum_pp + rtDW.Integrator_DSTATE_jz) +
      rtb_FilterCoefficient_k;

    /* Saturate: '<S806>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_6' */
      rtY.yaw_SPD_OUT_CH1_6 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_6' */
      rtY.yaw_SPD_OUT_CH1_6 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_6' */
      rtY.yaw_SPD_OUT_CH1_6 = u0;
    }

    /* End of Saturate: '<S806>/Saturation' */

    /* Update for DiscreteIntegrator: '<S794>/Filter' */
    rtDW.Filter_DSTATE_ke += 0.001F * rtb_FilterCoefficient_k;

    /* Update for DiscreteIntegrator: '<S799>/Integrator' incorporates:
     *  Gain: '<S796>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_jz += rtP.SPD_I_CH1_6 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S6>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S6>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S766>/Action Port'
     */
    /* Sum: '<S766>/Sum' incorporates:
     *  Gain: '<S766>/Gain'
     *  Inport: '<Root>/yaw_circle_CH1_6'
     *  Inport: '<Root>/yaw_ecd_CH1_6'
     *  Inport: '<Root>/yaw_target_CH1_6'
     *  Sum: '<S766>/Sum2'
     */
    rtb_FilterCoefficient_k = rtU.yaw_target_CH1_6 - (8191.0F *
      rtU.yaw_circle_CH1_6 + rtU.yaw_ecd_CH1_6);

    /* Abs: '<S766>/Abs' */
    rtb_Sum_pp = (real32_T)fabs(rtb_FilterCoefficient_k);

    /* Switch: '<S766>/Switch2' incorporates:
     *  Constant: '<S766>/Constant'
     */
    if (!(rtb_Sum_pp > rtP.DEADBAND_CH1_6)) {
      rtb_FilterCoefficient_k = 0.0F;
    }

    /* End of Switch: '<S766>/Switch2' */

    /* Gain: '<S901>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S893>/Filter'
     *  Gain: '<S892>/Derivative Gain'
     *  Sum: '<S893>/SumD'
     */
    rtb_FilterCoefficient_dy = (rtP.ANG_A_D_CH1_6 * rtb_FilterCoefficient_k -
      rtDW.Filter_DSTATE_b3) * 100.0F;

    /* Switch: '<S766>/Switch1' incorporates:
     *  Constant: '<S766>/Constant'
     *  Gain: '<S766>/Multiply'
     *  Inport: '<Root>/yaw_speed_rpm_CH1_6'
     *  Saturate: '<S905>/Saturation'
     *  Sum: '<S766>/Sum1'
     */
    if (rtb_Sum_pp > rtP.DEADBAND_CH1_6) {
      /* Sum: '<S907>/Sum' incorporates:
       *  DiscreteIntegrator: '<S898>/Integrator'
       *  Gain: '<S903>/Proportional Gain'
       */
      u0 = (rtP.ANG_A_P_CH1_6 * rtb_FilterCoefficient_k +
            rtDW.Integrator_DSTATE_en) + rtb_FilterCoefficient_dy;

      /* Saturate: '<S905>/Saturation' */
      if (u0 > 16384.0F) {
        u0 = 16384.0F;
      } else if (u0 < -16384.0F) {
        u0 = -16384.0F;
      }

      rtb_Sum_pp = rtP.TRANS_CH1_6 * u0 - rtU.yaw_speed_rpm_CH1_6;
    } else {
      rtb_Sum_pp = 0.0F;
    }

    /* End of Switch: '<S766>/Switch1' */

    /* Gain: '<S853>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S845>/Filter'
     *  Gain: '<S844>/Derivative Gain'
     *  Sum: '<S845>/SumD'
     */
    rtb_FilterCoefficient_e4 = (rtP.ANG_S_D_CH1_6 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_e) * 100.0F;

    /* Sum: '<S859>/Sum' incorporates:
     *  DiscreteIntegrator: '<S850>/Integrator'
     *  Gain: '<S855>/Proportional Gain'
     */
    u0 = (rtP.ANG_S_P_CH1_6 * rtb_Sum_pp + rtDW.Integrator_DSTATE_j) +
      rtb_FilterCoefficient_e4;

    /* Saturate: '<S857>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_6' */
      rtY.yaw_ANG_OUT_CH1_6 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_6' */
      rtY.yaw_ANG_OUT_CH1_6 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_6' */
      rtY.yaw_ANG_OUT_CH1_6 = u0;
    }

    /* End of Saturate: '<S857>/Saturation' */

    /* Chart: '<S766>/Chart1' incorporates:
     *  Inport: '<Root>/yaw_ecd_CH1_6'
     *  Inport: '<Root>/yaw_last_ecd_CH1_6'
     *  Sum: '<S766>/Sum3'
     */
    Chart1(rtU.yaw_ecd_CH1_6 - rtU.yaw_last_ecd_CH1_6, &rtDW.u2_jm,
           &rtDW.sf_Chart1_f);

    /* Outport: '<Root>/yaw_circle_out_CH1_6' incorporates:
     *  Inport: '<Root>/yaw_circle_CH1_6'
     *  Sum: '<S766>/Sum4'
     */
    rtY.yaw_circle_out_CH1_6 = rtDW.u2_jm + rtU.yaw_circle_CH1_6;

    /* Update for DiscreteIntegrator: '<S898>/Integrator' incorporates:
     *  Gain: '<S895>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_en += rtP.ANG_A_I_CH1_6 * rtb_FilterCoefficient_k *
      0.001F;

    /* Update for DiscreteIntegrator: '<S893>/Filter' */
    rtDW.Filter_DSTATE_b3 += 0.001F * rtb_FilterCoefficient_dy;

    /* Update for DiscreteIntegrator: '<S845>/Filter' */
    rtDW.Filter_DSTATE_e += 0.001F * rtb_FilterCoefficient_e4;

    /* Update for DiscreteIntegrator: '<S850>/Integrator' incorporates:
     *  Gain: '<S847>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_j += rtP.ANG_S_I_CH1_6 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S6>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S6>/Switch Case' */

  /* SwitchCase: '<S7>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_status_CH1_7'
   */
  switch ((int32_T)rtU.yaw_status_CH1_7) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S7>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S915>/Action Port'
     */
    /* Sum: '<S915>/Sum' incorporates:
     *  Inport: '<Root>/yaw_speed_rpm_CH1_7'
     *  Inport: '<Root>/yaw_target_CH1_7'
     */
    rtb_Sum_pp = rtU.yaw_target_CH1_7 - rtU.yaw_speed_rpm_CH1_7;

    /* Gain: '<S952>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S944>/Filter'
     *  Gain: '<S943>/Derivative Gain'
     *  Sum: '<S944>/SumD'
     */
    rtb_FilterCoefficient_k = (rtP.SPD_D_CH1_7 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_nv) * 100.0F;

    /* Sum: '<S958>/Sum' incorporates:
     *  DiscreteIntegrator: '<S949>/Integrator'
     *  Gain: '<S954>/Proportional Gain'
     */
    u0 = (rtP.SPD_P_CH1_7 * rtb_Sum_pp + rtDW.Integrator_DSTATE_mp) +
      rtb_FilterCoefficient_k;

    /* Saturate: '<S956>/Saturation' */
    if (u0 > 3000.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_7' */
      rtY.yaw_SPD_OUT_CH1_7 = 3000.0F;
    } else if (u0 < -3000.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_7' */
      rtY.yaw_SPD_OUT_CH1_7 = -3000.0F;
    } else {
      /* Outport: '<Root>/yaw_SPD_OUT_CH1_7' */
      rtY.yaw_SPD_OUT_CH1_7 = u0;
    }

    /* End of Saturate: '<S956>/Saturation' */

    /* Update for DiscreteIntegrator: '<S944>/Filter' */
    rtDW.Filter_DSTATE_nv += 0.001F * rtb_FilterCoefficient_k;

    /* Update for DiscreteIntegrator: '<S949>/Integrator' incorporates:
     *  Gain: '<S946>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_mp += rtP.SPD_I_CH1_7 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S7>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S7>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S916>/Action Port'
     */
    /* Sum: '<S916>/Sum' incorporates:
     *  Gain: '<S916>/Gain'
     *  Inport: '<Root>/yaw_circle_CH1_7'
     *  Inport: '<Root>/yaw_ecd_CH1_7'
     *  Inport: '<Root>/yaw_target_CH1_7'
     *  Sum: '<S916>/Sum2'
     */
    rtb_FilterCoefficient_k = rtU.yaw_target_CH1_7 - (8191.0F *
      rtU.yaw_circle_CH1_7 + rtU.yaw_ecd_CH1_7);

    /* Abs: '<S916>/Abs' */
    rtb_Sum_pp = (real32_T)fabs(rtb_FilterCoefficient_k);

    /* Switch: '<S916>/Switch2' incorporates:
     *  Constant: '<S916>/Constant'
     */
    if (!(rtb_Sum_pp > rtP.DEADBAND_CH1_7)) {
      rtb_FilterCoefficient_k = 0.0F;
    }

    /* End of Switch: '<S916>/Switch2' */

    /* Gain: '<S1051>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1043>/Filter'
     *  Gain: '<S1042>/Derivative Gain'
     *  Sum: '<S1043>/SumD'
     */
    rtb_FilterCoefficient_dy = (rtP.ANG_A_D_CH1_7 * rtb_FilterCoefficient_k -
      rtDW.Filter_DSTATE_m0) * 100.0F;

    /* Switch: '<S916>/Switch1' incorporates:
     *  Constant: '<S916>/Constant'
     *  Gain: '<S916>/Multiply'
     *  Inport: '<Root>/yaw_speed_rpm_CH1_7'
     *  Saturate: '<S1055>/Saturation'
     *  Sum: '<S916>/Sum1'
     */
    if (rtb_Sum_pp > rtP.DEADBAND_CH1_7) {
      /* Sum: '<S1057>/Sum' incorporates:
       *  DiscreteIntegrator: '<S1048>/Integrator'
       *  Gain: '<S1053>/Proportional Gain'
       */
      u0 = (rtP.ANG_A_P_CH1_7 * rtb_FilterCoefficient_k +
            rtDW.Integrator_DSTATE_gf) + rtb_FilterCoefficient_dy;

      /* Saturate: '<S1055>/Saturation' */
      if (u0 > 3000.0F) {
        u0 = 3000.0F;
      } else if (u0 < -3000.0F) {
        u0 = -3000.0F;
      }

      rtb_Sum_pp = rtP.TRANS_CH1_7 * u0 - rtU.yaw_speed_rpm_CH1_7;
    } else {
      rtb_Sum_pp = 0.0F;
    }

    /* End of Switch: '<S916>/Switch1' */

    /* Gain: '<S1003>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S995>/Filter'
     *  Gain: '<S994>/Derivative Gain'
     *  Sum: '<S995>/SumD'
     */
    rtb_FilterCoefficient_e4 = (rtP.ANG_S_D_CH1_7 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_hb) * 100.0F;

    /* Sum: '<S1009>/Sum' incorporates:
     *  DiscreteIntegrator: '<S1000>/Integrator'
     *  Gain: '<S1005>/Proportional Gain'
     */
    u0 = (rtP.ANG_S_P_CH1_7 * rtb_Sum_pp + rtDW.Integrator_DSTATE_li) +
      rtb_FilterCoefficient_e4;

    /* Saturate: '<S1007>/Saturation' */
    if (u0 > 3000.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_7' */
      rtY.yaw_ANG_OUT_CH1_7 = 3000.0F;
    } else if (u0 < -3000.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_7' */
      rtY.yaw_ANG_OUT_CH1_7 = -3000.0F;
    } else {
      /* Outport: '<Root>/yaw_ANG_OUT_CH1_7' */
      rtY.yaw_ANG_OUT_CH1_7 = u0;
    }

    /* End of Saturate: '<S1007>/Saturation' */

    /* Chart: '<S916>/Chart1' incorporates:
     *  Inport: '<Root>/yaw_ecd_CH1_7'
     *  Inport: '<Root>/yaw_last_ecd_CH1_7'
     *  Sum: '<S916>/Sum3'
     */
    Chart1(rtU.yaw_ecd_CH1_7 - rtU.yaw_last_ecd_CH1_7, &rtDW.u2_nr,
           &rtDW.sf_Chart1_in);

    /* Outport: '<Root>/yaw_circle_out_CH1_7' incorporates:
     *  Inport: '<Root>/yaw_circle_CH1_7'
     *  Sum: '<S916>/Sum4'
     */
    rtY.yaw_circle_out_CH1_7 = rtDW.u2_nr + rtU.yaw_circle_CH1_7;

    /* Update for DiscreteIntegrator: '<S1048>/Integrator' incorporates:
     *  Gain: '<S1045>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_gf += rtP.ANG_A_I_CH1_7 * rtb_FilterCoefficient_k *
      0.001F;

    /* Update for DiscreteIntegrator: '<S1043>/Filter' */
    rtDW.Filter_DSTATE_m0 += 0.001F * rtb_FilterCoefficient_dy;

    /* Update for DiscreteIntegrator: '<S995>/Filter' */
    rtDW.Filter_DSTATE_hb += 0.001F * rtb_FilterCoefficient_e4;

    /* Update for DiscreteIntegrator: '<S1000>/Integrator' incorporates:
     *  Gain: '<S997>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_li += rtP.ANG_S_I_CH1_7 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S7>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S7>/Switch Case' */

  /* SwitchCase: '<S9>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_status_CH2_2'
   */
  switch ((int32_T)rtU.yaw_status_CH2_2) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S9>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S1215>/Action Port'
     */
    /* Sum: '<S1215>/Sum' incorporates:
     *  Inport: '<Root>/yaw_speed_rpm_CH2_2'
     *  Inport: '<Root>/yaw_target_CH2_2'
     */
    rtb_Sum_pp = rtU.yaw_target_CH2_2 - rtU.yaw_speed_rpm_CH2_2;

    /* Gain: '<S1252>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1244>/Filter'
     *  Gain: '<S1243>/Derivative Gain'
     *  Sum: '<S1244>/SumD'
     */
    rtb_FilterCoefficient_k = (rtP.SPD_D_CH2_2 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_l) * 100.0F;

    /* Sum: '<S1258>/Sum' incorporates:
     *  DiscreteIntegrator: '<S1249>/Integrator'
     *  Gain: '<S1254>/Proportional Gain'
     */
    u0 = (rtP.SPD_P_CH2_2 * rtb_Sum_pp + rtDW.Integrator_DSTATE_m) +
      rtb_FilterCoefficient_k;

    /* Saturate: '<S1256>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_2' */
      rtY.yaw_SPD_OUT_CH2_2 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_2' */
      rtY.yaw_SPD_OUT_CH2_2 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_2' */
      rtY.yaw_SPD_OUT_CH2_2 = u0;
    }

    /* End of Saturate: '<S1256>/Saturation' */

    /* Update for DiscreteIntegrator: '<S1244>/Filter' */
    rtDW.Filter_DSTATE_l += 0.001F * rtb_FilterCoefficient_k;

    /* Update for DiscreteIntegrator: '<S1249>/Integrator' incorporates:
     *  Gain: '<S1246>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_m += rtP.SPD_I_CH2_2 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S9>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S9>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S1216>/Action Port'
     */
    /* Sum: '<S1216>/Sum' incorporates:
     *  Gain: '<S1216>/Gain'
     *  Inport: '<Root>/yaw_circle_CH2_2'
     *  Inport: '<Root>/yaw_ecd_CH2_2'
     *  Inport: '<Root>/yaw_target_CH2_2'
     *  Sum: '<S1216>/Sum2'
     */
    rtb_FilterCoefficient_k = rtU.yaw_target_CH2_2 - (8191.0F *
      rtU.yaw_circle_CH2_2 + rtU.yaw_ecd_CH2_2);

    /* Abs: '<S1216>/Abs' */
    rtb_Sum_pp = (real32_T)fabs(rtb_FilterCoefficient_k);

    /* Switch: '<S1216>/Switch2' incorporates:
     *  Constant: '<S1216>/Constant'
     */
    if (!(rtb_Sum_pp > rtP.DEADBAND_CH2_2)) {
      rtb_FilterCoefficient_k = 0.0F;
    }

    /* End of Switch: '<S1216>/Switch2' */

    /* Gain: '<S1351>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1343>/Filter'
     *  Gain: '<S1342>/Derivative Gain'
     *  Sum: '<S1343>/SumD'
     */
    rtb_FilterCoefficient_dy = (rtP.ANG_A_D_CH2_2 * rtb_FilterCoefficient_k -
      rtDW.Filter_DSTATE_b) * 100.0F;

    /* Switch: '<S1216>/Switch1' incorporates:
     *  Constant: '<S1216>/Constant'
     *  Gain: '<S1216>/Multiply'
     *  Inport: '<Root>/yaw_speed_rpm_CH2_2'
     *  Saturate: '<S1355>/Saturation'
     *  Sum: '<S1216>/Sum1'
     */
    if (rtb_Sum_pp > rtP.DEADBAND_CH2_2) {
      /* Sum: '<S1357>/Sum' incorporates:
       *  DiscreteIntegrator: '<S1348>/Integrator'
       *  Gain: '<S1353>/Proportional Gain'
       */
      u0 = (rtP.ANG_A_P_CH2_2 * rtb_FilterCoefficient_k +
            rtDW.Integrator_DSTATE_e) + rtb_FilterCoefficient_dy;

      /* Saturate: '<S1355>/Saturation' */
      if (u0 > 16384.0F) {
        u0 = 16384.0F;
      } else if (u0 < -16384.0F) {
        u0 = -16384.0F;
      }

      rtb_Sum_pp = rtP.TRANS_CH2_2 * u0 - rtU.yaw_speed_rpm_CH2_2;
    } else {
      rtb_Sum_pp = 0.0F;
    }

    /* End of Switch: '<S1216>/Switch1' */

    /* Gain: '<S1303>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1295>/Filter'
     *  Gain: '<S1294>/Derivative Gain'
     *  Sum: '<S1295>/SumD'
     */
    rtb_FilterCoefficient_e4 = (rtP.ANG_S_D_CH2_2 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_gy) * 100.0F;

    /* Sum: '<S1309>/Sum' incorporates:
     *  DiscreteIntegrator: '<S1300>/Integrator'
     *  Gain: '<S1305>/Proportional Gain'
     */
    u0 = (rtP.ANG_S_P_CH2_2 * rtb_Sum_pp + rtDW.Integrator_DSTATE_ds) +
      rtb_FilterCoefficient_e4;

    /* Saturate: '<S1307>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_2' */
      rtY.yaw_ANG_OUT_CH2_2 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_2' */
      rtY.yaw_ANG_OUT_CH2_2 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_2' */
      rtY.yaw_ANG_OUT_CH2_2 = u0;
    }

    /* End of Saturate: '<S1307>/Saturation' */

    /* Chart: '<S1216>/Chart1' incorporates:
     *  Inport: '<Root>/yaw_ecd_CH2_2'
     *  Inport: '<Root>/yaw_last_ecd_CH2_2'
     *  Sum: '<S1216>/Sum3'
     */
    Chart1(rtU.yaw_ecd_CH2_2 - rtU.yaw_last_ecd_CH2_2, &rtDW.u2_a,
           &rtDW.sf_Chart1_b);

    /* Outport: '<Root>/yaw_circle_out_CH2_2' incorporates:
     *  Inport: '<Root>/yaw_circle_CH2_2'
     *  Sum: '<S1216>/Sum4'
     */
    rtY.yaw_circle_out_CH2_2 = rtDW.u2_a + rtU.yaw_circle_CH2_2;

    /* Update for DiscreteIntegrator: '<S1348>/Integrator' incorporates:
     *  Gain: '<S1345>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_e += rtP.ANG_A_I_CH2_2 * rtb_FilterCoefficient_k *
      0.001F;

    /* Update for DiscreteIntegrator: '<S1343>/Filter' */
    rtDW.Filter_DSTATE_b += 0.001F * rtb_FilterCoefficient_dy;

    /* Update for DiscreteIntegrator: '<S1295>/Filter' */
    rtDW.Filter_DSTATE_gy += 0.001F * rtb_FilterCoefficient_e4;

    /* Update for DiscreteIntegrator: '<S1300>/Integrator' incorporates:
     *  Gain: '<S1297>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_ds += rtP.ANG_S_I_CH2_2 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S9>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S9>/Switch Case' */

  /* SwitchCase: '<S10>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_status_CH2_3'
   */
  switch ((int32_T)rtU.yaw_status_CH2_3) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S10>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S1365>/Action Port'
     */
    /* Sum: '<S1365>/Sum' incorporates:
     *  Inport: '<Root>/yaw_speed_rpm_CH2_3'
     *  Inport: '<Root>/yaw_target_CH2_3'
     */
    rtb_Sum_pp = rtU.yaw_target_CH2_3 - rtU.yaw_speed_rpm_CH2_3;

    /* Gain: '<S1402>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1394>/Filter'
     *  Gain: '<S1393>/Derivative Gain'
     *  Sum: '<S1394>/SumD'
     */
    rtb_FilterCoefficient_k = (rtP.SPD_D_CH2_3 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_n) * 100.0F;

    /* Sum: '<S1408>/Sum' incorporates:
     *  DiscreteIntegrator: '<S1399>/Integrator'
     *  Gain: '<S1404>/Proportional Gain'
     */
    u0 = (rtP.SPD_P_CH2_3 * rtb_Sum_pp + rtDW.Integrator_DSTATE_i) +
      rtb_FilterCoefficient_k;

    /* Saturate: '<S1406>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_3' */
      rtY.yaw_SPD_OUT_CH2_3 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_3' */
      rtY.yaw_SPD_OUT_CH2_3 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_3' */
      rtY.yaw_SPD_OUT_CH2_3 = u0;
    }

    /* End of Saturate: '<S1406>/Saturation' */

    /* Update for DiscreteIntegrator: '<S1394>/Filter' */
    rtDW.Filter_DSTATE_n += 0.001F * rtb_FilterCoefficient_k;

    /* Update for DiscreteIntegrator: '<S1399>/Integrator' incorporates:
     *  Gain: '<S1396>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_i += rtP.SPD_I_CH2_3 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S10>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S10>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S1366>/Action Port'
     */
    /* Sum: '<S1366>/Sum' incorporates:
     *  Gain: '<S1366>/Gain'
     *  Inport: '<Root>/yaw_circle_CH2_3'
     *  Inport: '<Root>/yaw_ecd_CH2_3'
     *  Inport: '<Root>/yaw_target_CH2_3'
     *  Sum: '<S1366>/Sum2'
     */
    rtb_FilterCoefficient_k = rtU.yaw_target_CH2_3 - (8191.0F *
      rtU.yaw_circle_CH2_3 + rtU.yaw_ecd_CH2_3);

    /* Abs: '<S1366>/Abs' */
    rtb_Sum_pp = (real32_T)fabs(rtb_FilterCoefficient_k);

    /* Switch: '<S1366>/Switch2' incorporates:
     *  Constant: '<S1366>/Constant'
     */
    if (!(rtb_Sum_pp > rtP.DEADBAND_CH2_3)) {
      rtb_FilterCoefficient_k = 0.0F;
    }

    /* End of Switch: '<S1366>/Switch2' */

    /* Gain: '<S1501>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1493>/Filter'
     *  Gain: '<S1492>/Derivative Gain'
     *  Sum: '<S1493>/SumD'
     */
    rtb_FilterCoefficient_dy = (rtP.ANG_A_D_CH2_3 * rtb_FilterCoefficient_k -
      rtDW.Filter_DSTATE_a) * 100.0F;

    /* Switch: '<S1366>/Switch1' incorporates:
     *  Constant: '<S1366>/Constant'
     *  Gain: '<S1366>/Multiply'
     *  Inport: '<Root>/yaw_speed_rpm_CH2_3'
     *  Saturate: '<S1505>/Saturation'
     *  Sum: '<S1366>/Sum1'
     */
    if (rtb_Sum_pp > rtP.DEADBAND_CH2_3) {
      /* Sum: '<S1507>/Sum' incorporates:
       *  DiscreteIntegrator: '<S1498>/Integrator'
       *  Gain: '<S1503>/Proportional Gain'
       */
      u0 = (rtP.ANG_A_P_CH2_3 * rtb_FilterCoefficient_k +
            rtDW.Integrator_DSTATE_hc) + rtb_FilterCoefficient_dy;

      /* Saturate: '<S1505>/Saturation' */
      if (u0 > 16384.0F) {
        u0 = 16384.0F;
      } else if (u0 < -16384.0F) {
        u0 = -16384.0F;
      }

      rtb_Sum_pp = rtP.TRANS_CH2_3 * u0 - rtU.yaw_speed_rpm_CH2_3;
    } else {
      rtb_Sum_pp = 0.0F;
    }

    /* End of Switch: '<S1366>/Switch1' */

    /* Gain: '<S1453>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1445>/Filter'
     *  Gain: '<S1444>/Derivative Gain'
     *  Sum: '<S1445>/SumD'
     */
    rtb_FilterCoefficient_e4 = (rtP.ANG_S_D_CH2_3 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_k) * 100.0F;

    /* Sum: '<S1459>/Sum' incorporates:
     *  DiscreteIntegrator: '<S1450>/Integrator'
     *  Gain: '<S1455>/Proportional Gain'
     */
    u0 = (rtP.ANG_S_P_CH2_3 * rtb_Sum_pp + rtDW.Integrator_DSTATE_o) +
      rtb_FilterCoefficient_e4;

    /* Saturate: '<S1457>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_3' */
      rtY.yaw_ANG_OUT_CH2_3 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_3' */
      rtY.yaw_ANG_OUT_CH2_3 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_3' */
      rtY.yaw_ANG_OUT_CH2_3 = u0;
    }

    /* End of Saturate: '<S1457>/Saturation' */

    /* Chart: '<S1366>/Chart1' incorporates:
     *  Inport: '<Root>/yaw_ecd_CH2_3'
     *  Inport: '<Root>/yaw_last_ecd_CH2_3'
     *  Sum: '<S1366>/Sum3'
     */
    Chart1(rtU.yaw_ecd_CH2_3 - rtU.yaw_last_ecd_CH2_3, &rtDW.u2_ge,
           &rtDW.sf_Chart1_nh);

    /* Outport: '<Root>/yaw_circle_out_CH2_3' incorporates:
     *  Inport: '<Root>/yaw_circle_CH2_3'
     *  Sum: '<S1366>/Sum4'
     */
    rtY.yaw_circle_out_CH2_3 = rtDW.u2_ge + rtU.yaw_circle_CH2_3;

    /* Update for DiscreteIntegrator: '<S1498>/Integrator' incorporates:
     *  Gain: '<S1495>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_hc += rtP.ANG_A_I_CH2_3 * rtb_FilterCoefficient_k *
      0.001F;

    /* Update for DiscreteIntegrator: '<S1493>/Filter' */
    rtDW.Filter_DSTATE_a += 0.001F * rtb_FilterCoefficient_dy;

    /* Update for DiscreteIntegrator: '<S1445>/Filter' */
    rtDW.Filter_DSTATE_k += 0.001F * rtb_FilterCoefficient_e4;

    /* Update for DiscreteIntegrator: '<S1450>/Integrator' incorporates:
     *  Gain: '<S1447>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_o += rtP.ANG_S_I_CH2_3 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S10>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S10>/Switch Case' */

  /* SwitchCase: '<S11>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_status_CH2_4'
   */
  switch ((int32_T)rtU.yaw_status_CH2_4) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S11>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S1515>/Action Port'
     */
    /* Sum: '<S1515>/Sum' incorporates:
     *  Inport: '<Root>/yaw_speed_rpm_CH2_4'
     *  Inport: '<Root>/yaw_target_CH2_4'
     */
    rtb_Sum_pp = rtU.yaw_target_CH2_4 - rtU.yaw_speed_rpm_CH2_4;

    /* Gain: '<S1552>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1544>/Filter'
     *  Gain: '<S1543>/Derivative Gain'
     *  Sum: '<S1544>/SumD'
     */
    rtb_FilterCoefficient_k = (rtP.SPD_D_CH2_4 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_g) * 100.0F;

    /* Sum: '<S1558>/Sum' incorporates:
     *  DiscreteIntegrator: '<S1549>/Integrator'
     *  Gain: '<S1554>/Proportional Gain'
     */
    u0 = (rtP.SPD_P_CH2_4 * rtb_Sum_pp + rtDW.Integrator_DSTATE_a1) +
      rtb_FilterCoefficient_k;

    /* Saturate: '<S1556>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_4' */
      rtY.yaw_SPD_OUT_CH2_4 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_4' */
      rtY.yaw_SPD_OUT_CH2_4 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_4' */
      rtY.yaw_SPD_OUT_CH2_4 = u0;
    }

    /* End of Saturate: '<S1556>/Saturation' */

    /* Update for DiscreteIntegrator: '<S1544>/Filter' */
    rtDW.Filter_DSTATE_g += 0.001F * rtb_FilterCoefficient_k;

    /* Update for DiscreteIntegrator: '<S1549>/Integrator' incorporates:
     *  Gain: '<S1546>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_a1 += rtP.SPD_I_CH2_4 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S11>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S11>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S1516>/Action Port'
     */
    /* Sum: '<S1516>/Sum' incorporates:
     *  Gain: '<S1516>/Gain'
     *  Inport: '<Root>/yaw_circle_CH2_4'
     *  Inport: '<Root>/yaw_ecd_CH2_4'
     *  Inport: '<Root>/yaw_target_CH2_4'
     *  Sum: '<S1516>/Sum2'
     */
    rtb_FilterCoefficient_k = rtU.yaw_target_CH2_4 - (8191.0F *
      rtU.yaw_circle_CH2_4 + rtU.yaw_ecd_CH2_4);

    /* Abs: '<S1516>/Abs' */
    rtb_Sum_pp = (real32_T)fabs(rtb_FilterCoefficient_k);

    /* Switch: '<S1516>/Switch2' incorporates:
     *  Constant: '<S1516>/Constant'
     */
    if (!(rtb_Sum_pp > rtP.DEADBAND_CH2_4)) {
      rtb_FilterCoefficient_k = 0.0F;
    }

    /* End of Switch: '<S1516>/Switch2' */

    /* Gain: '<S1651>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1643>/Filter'
     *  Gain: '<S1642>/Derivative Gain'
     *  Sum: '<S1643>/SumD'
     */
    rtb_FilterCoefficient_dy = (rtP.ANG_A_D_CH2_4 * rtb_FilterCoefficient_k -
      rtDW.Filter_DSTATE_m) * 100.0F;

    /* Switch: '<S1516>/Switch1' incorporates:
     *  Constant: '<S1516>/Constant'
     *  Gain: '<S1516>/Multiply'
     *  Inport: '<Root>/yaw_speed_rpm_CH2_4'
     *  Saturate: '<S1655>/Saturation'
     *  Sum: '<S1516>/Sum1'
     */
    if (rtb_Sum_pp > rtP.DEADBAND_CH2_4) {
      /* Sum: '<S1657>/Sum' incorporates:
       *  DiscreteIntegrator: '<S1648>/Integrator'
       *  Gain: '<S1653>/Proportional Gain'
       */
      u0 = (rtP.ANG_A_P_CH2_4 * rtb_FilterCoefficient_k +
            rtDW.Integrator_DSTATE_g) + rtb_FilterCoefficient_dy;

      /* Saturate: '<S1655>/Saturation' */
      if (u0 > 16384.0F) {
        u0 = 16384.0F;
      } else if (u0 < -16384.0F) {
        u0 = -16384.0F;
      }

      rtb_Sum_pp = rtP.TRANS_CH2_4 * u0 - rtU.yaw_speed_rpm_CH2_4;
    } else {
      rtb_Sum_pp = 0.0F;
    }

    /* End of Switch: '<S1516>/Switch1' */

    /* Gain: '<S1603>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1595>/Filter'
     *  Gain: '<S1594>/Derivative Gain'
     *  Sum: '<S1595>/SumD'
     */
    rtb_FilterCoefficient_e4 = (rtP.ANG_S_D_CH2_4 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_dx) * 100.0F;

    /* Sum: '<S1609>/Sum' incorporates:
     *  DiscreteIntegrator: '<S1600>/Integrator'
     *  Gain: '<S1605>/Proportional Gain'
     */
    u0 = (rtP.ANG_S_P_CH2_4 * rtb_Sum_pp + rtDW.Integrator_DSTATE_k) +
      rtb_FilterCoefficient_e4;

    /* Saturate: '<S1607>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_4' */
      rtY.yaw_ANG_OUT_CH2_4 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_4' */
      rtY.yaw_ANG_OUT_CH2_4 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_4' */
      rtY.yaw_ANG_OUT_CH2_4 = u0;
    }

    /* End of Saturate: '<S1607>/Saturation' */

    /* Chart: '<S1516>/Chart1' incorporates:
     *  Inport: '<Root>/yaw_ecd_CH2_4'
     *  Inport: '<Root>/yaw_last_ecd_CH2_4'
     *  Sum: '<S1516>/Sum3'
     */
    Chart1(rtU.yaw_ecd_CH2_4 - rtU.yaw_last_ecd_CH2_4, &rtDW.u2_j,
           &rtDW.sf_Chart1_k);

    /* Outport: '<Root>/yaw_circle_out_CH2_4' incorporates:
     *  Inport: '<Root>/yaw_circle_CH2_4'
     *  Sum: '<S1516>/Sum4'
     */
    rtY.yaw_circle_out_CH2_4 = rtDW.u2_j + rtU.yaw_circle_CH2_4;

    /* Update for DiscreteIntegrator: '<S1648>/Integrator' incorporates:
     *  Gain: '<S1645>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_g += rtP.ANG_A_I_CH2_4 * rtb_FilterCoefficient_k *
      0.001F;

    /* Update for DiscreteIntegrator: '<S1643>/Filter' */
    rtDW.Filter_DSTATE_m += 0.001F * rtb_FilterCoefficient_dy;

    /* Update for DiscreteIntegrator: '<S1595>/Filter' */
    rtDW.Filter_DSTATE_dx += 0.001F * rtb_FilterCoefficient_e4;

    /* Update for DiscreteIntegrator: '<S1600>/Integrator' incorporates:
     *  Gain: '<S1597>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_k += rtP.ANG_S_I_CH2_4 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S11>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S11>/Switch Case' */

  /* SwitchCase: '<S12>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_status_CH2_5'
   */
  switch ((int32_T)rtU.yaw_status_CH2_5) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S12>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S1665>/Action Port'
     */
    /* Sum: '<S1665>/Sum' incorporates:
     *  Inport: '<Root>/yaw_speed_rpm_CH2_5'
     *  Inport: '<Root>/yaw_target_CH2_5'
     */
    rtb_Sum_pp = rtU.yaw_target_CH2_5 - rtU.yaw_speed_rpm_CH2_5;

    /* Gain: '<S1702>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1694>/Filter'
     *  Gain: '<S1693>/Derivative Gain'
     *  Sum: '<S1694>/SumD'
     */
    rtb_FilterCoefficient_k = (rtP.SPD_D_CH2_5 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_om) * 100.0F;

    /* Sum: '<S1708>/Sum' incorporates:
     *  DiscreteIntegrator: '<S1699>/Integrator'
     *  Gain: '<S1704>/Proportional Gain'
     */
    u0 = (rtP.SPD_P_CH2_5 * rtb_Sum_pp + rtDW.Integrator_DSTATE_a) +
      rtb_FilterCoefficient_k;

    /* Saturate: '<S1706>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_5' */
      rtY.yaw_SPD_OUT_CH2_5 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_5' */
      rtY.yaw_SPD_OUT_CH2_5 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_5' */
      rtY.yaw_SPD_OUT_CH2_5 = u0;
    }

    /* End of Saturate: '<S1706>/Saturation' */

    /* Update for DiscreteIntegrator: '<S1694>/Filter' */
    rtDW.Filter_DSTATE_om += 0.001F * rtb_FilterCoefficient_k;

    /* Update for DiscreteIntegrator: '<S1699>/Integrator' incorporates:
     *  Gain: '<S1696>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_a += rtP.SPD_I_CH2_5 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S12>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S12>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S1666>/Action Port'
     */
    /* Sum: '<S1666>/Sum' incorporates:
     *  Gain: '<S1666>/Gain'
     *  Inport: '<Root>/yaw_circle_CH2_5'
     *  Inport: '<Root>/yaw_ecd_CH2_5'
     *  Inport: '<Root>/yaw_target_CH2_5'
     *  Sum: '<S1666>/Sum2'
     */
    rtb_FilterCoefficient_k = rtU.yaw_target_CH2_5 - (8191.0F *
      rtU.yaw_circle_CH2_5 + rtU.yaw_ecd_CH2_5);

    /* Abs: '<S1666>/Abs' */
    rtb_Sum_pp = (real32_T)fabs(rtb_FilterCoefficient_k);

    /* Switch: '<S1666>/Switch2' incorporates:
     *  Constant: '<S1666>/Constant'
     */
    if (!(rtb_Sum_pp > rtP.DEADBAND_CH2_5)) {
      rtb_FilterCoefficient_k = 0.0F;
    }

    /* End of Switch: '<S1666>/Switch2' */

    /* Gain: '<S1801>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1793>/Filter'
     *  Gain: '<S1792>/Derivative Gain'
     *  Sum: '<S1793>/SumD'
     */
    rtb_FilterCoefficient_dy = (rtP.ANG_A_D_CH2_5 * rtb_FilterCoefficient_k -
      rtDW.Filter_DSTATE_fg) * 100.0F;

    /* Switch: '<S1666>/Switch1' incorporates:
     *  Constant: '<S1666>/Constant'
     *  Gain: '<S1666>/Multiply'
     *  Inport: '<Root>/yaw_speed_rpm_CH2_5'
     *  Saturate: '<S1805>/Saturation'
     *  Sum: '<S1666>/Sum1'
     */
    if (rtb_Sum_pp > rtP.DEADBAND_CH2_5) {
      /* Sum: '<S1807>/Sum' incorporates:
       *  DiscreteIntegrator: '<S1798>/Integrator'
       *  Gain: '<S1803>/Proportional Gain'
       */
      u0 = (rtP.ANG_A_P_CH2_5 * rtb_FilterCoefficient_k +
            rtDW.Integrator_DSTATE_l) + rtb_FilterCoefficient_dy;

      /* Saturate: '<S1805>/Saturation' */
      if (u0 > 16384.0F) {
        u0 = 16384.0F;
      } else if (u0 < -16384.0F) {
        u0 = -16384.0F;
      }

      rtb_Sum_pp = rtP.TRANS_CH2_5 * u0 - rtU.yaw_speed_rpm_CH2_5;
    } else {
      rtb_Sum_pp = 0.0F;
    }

    /* End of Switch: '<S1666>/Switch1' */

    /* Gain: '<S1753>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1745>/Filter'
     *  Gain: '<S1744>/Derivative Gain'
     *  Sum: '<S1745>/SumD'
     */
    rtb_FilterCoefficient_e4 = (rtP.ANG_S_D_CH2_5 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_d) * 100.0F;

    /* Sum: '<S1759>/Sum' incorporates:
     *  DiscreteIntegrator: '<S1750>/Integrator'
     *  Gain: '<S1755>/Proportional Gain'
     */
    u0 = (rtP.ANG_S_P_CH2_5 * rtb_Sum_pp + rtDW.Integrator_DSTATE_d0) +
      rtb_FilterCoefficient_e4;

    /* Saturate: '<S1757>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_5' */
      rtY.yaw_ANG_OUT_CH2_5 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_5' */
      rtY.yaw_ANG_OUT_CH2_5 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_5' */
      rtY.yaw_ANG_OUT_CH2_5 = u0;
    }

    /* End of Saturate: '<S1757>/Saturation' */

    /* Chart: '<S1666>/Chart1' incorporates:
     *  Inport: '<Root>/yaw_ecd_CH2_5'
     *  Inport: '<Root>/yaw_last_ecd_CH2_5'
     *  Sum: '<S1666>/Sum3'
     */
    Chart1(rtU.yaw_ecd_CH2_5 - rtU.yaw_last_ecd_CH2_5, &rtDW.u2_n,
           &rtDW.sf_Chart1_m);

    /* Outport: '<Root>/yaw_circle_out_CH2_5' incorporates:
     *  Inport: '<Root>/yaw_circle_CH2_5'
     *  Sum: '<S1666>/Sum4'
     */
    rtY.yaw_circle_out_CH2_5 = rtDW.u2_n + rtU.yaw_circle_CH2_5;

    /* Update for DiscreteIntegrator: '<S1798>/Integrator' incorporates:
     *  Gain: '<S1795>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_l += rtP.ANG_A_I_CH2_5 * rtb_FilterCoefficient_k *
      0.001F;

    /* Update for DiscreteIntegrator: '<S1793>/Filter' */
    rtDW.Filter_DSTATE_fg += 0.001F * rtb_FilterCoefficient_dy;

    /* Update for DiscreteIntegrator: '<S1745>/Filter' */
    rtDW.Filter_DSTATE_d += 0.001F * rtb_FilterCoefficient_e4;

    /* Update for DiscreteIntegrator: '<S1750>/Integrator' incorporates:
     *  Gain: '<S1747>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_d0 += rtP.ANG_S_I_CH2_5 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S12>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S12>/Switch Case' */

  /* SwitchCase: '<S13>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_status_CH2_6'
   */
  switch ((int32_T)rtU.yaw_status_CH2_6) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S13>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S1815>/Action Port'
     */
    /* Sum: '<S1815>/Sum' incorporates:
     *  Inport: '<Root>/yaw_speed_rpm_CH2_6'
     *  Inport: '<Root>/yaw_target_CH2_6'
     */
    rtb_Sum_pp = rtU.yaw_target_CH2_6 - rtU.yaw_speed_rpm_CH2_6;

    /* Gain: '<S1852>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1844>/Filter'
     *  Gain: '<S1843>/Derivative Gain'
     *  Sum: '<S1844>/SumD'
     */
    rtb_FilterCoefficient_k = (rtP.SPD_D_CH2_6 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_h) * 100.0F;

    /* Sum: '<S1858>/Sum' incorporates:
     *  DiscreteIntegrator: '<S1849>/Integrator'
     *  Gain: '<S1854>/Proportional Gain'
     */
    u0 = (rtP.SPD_P_CH2_6 * rtb_Sum_pp + rtDW.Integrator_DSTATE_d) +
      rtb_FilterCoefficient_k;

    /* Saturate: '<S1856>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_6' */
      rtY.yaw_SPD_OUT_CH2_6 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_6' */
      rtY.yaw_SPD_OUT_CH2_6 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_6' */
      rtY.yaw_SPD_OUT_CH2_6 = u0;
    }

    /* End of Saturate: '<S1856>/Saturation' */

    /* Update for DiscreteIntegrator: '<S1844>/Filter' */
    rtDW.Filter_DSTATE_h += 0.001F * rtb_FilterCoefficient_k;

    /* Update for DiscreteIntegrator: '<S1849>/Integrator' incorporates:
     *  Gain: '<S1846>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_d += rtP.SPD_I_CH2_6 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S13>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S13>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S1816>/Action Port'
     */
    /* Sum: '<S1816>/Sum' incorporates:
     *  Gain: '<S1816>/Gain'
     *  Inport: '<Root>/yaw_circle_CH2_6'
     *  Inport: '<Root>/yaw_ecd_CH2_6'
     *  Inport: '<Root>/yaw_target_CH2_6'
     *  Sum: '<S1816>/Sum2'
     */
    rtb_FilterCoefficient_k = rtU.yaw_target_CH2_6 - (8191.0F *
      rtU.yaw_circle_CH2_6 + rtU.yaw_ecd_CH2_6);

    /* Abs: '<S1816>/Abs' */
    rtb_Sum_pp = (real32_T)fabs(rtb_FilterCoefficient_k);

    /* Switch: '<S1816>/Switch2' incorporates:
     *  Constant: '<S1816>/Constant'
     */
    if (!(rtb_Sum_pp > rtP.DEADBAND_CH2_6)) {
      rtb_FilterCoefficient_k = 0.0F;
    }

    /* End of Switch: '<S1816>/Switch2' */

    /* Gain: '<S1951>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1943>/Filter'
     *  Gain: '<S1942>/Derivative Gain'
     *  Sum: '<S1943>/SumD'
     */
    rtb_FilterCoefficient_dy = (rtP.ANG_A_D_CH2_6 * rtb_FilterCoefficient_k -
      rtDW.Filter_DSTATE_o) * 100.0F;

    /* Switch: '<S1816>/Switch1' incorporates:
     *  Constant: '<S1816>/Constant'
     *  Gain: '<S1816>/Multiply'
     *  Inport: '<Root>/yaw_speed_rpm_CH2_6'
     *  Saturate: '<S1955>/Saturation'
     *  Sum: '<S1816>/Sum1'
     */
    if (rtb_Sum_pp > rtP.DEADBAND_CH2_6) {
      /* Sum: '<S1957>/Sum' incorporates:
       *  DiscreteIntegrator: '<S1948>/Integrator'
       *  Gain: '<S1953>/Proportional Gain'
       */
      u0 = (rtP.ANG_A_P_CH2_6 * rtb_FilterCoefficient_k +
            rtDW.Integrator_DSTATE_h4) + rtb_FilterCoefficient_dy;

      /* Saturate: '<S1955>/Saturation' */
      if (u0 > 16384.0F) {
        u0 = 16384.0F;
      } else if (u0 < -16384.0F) {
        u0 = -16384.0F;
      }

      rtb_Sum_pp = rtP.TRANS_CH2_6 * u0 - rtU.yaw_speed_rpm_CH2_6;
    } else {
      rtb_Sum_pp = 0.0F;
    }

    /* End of Switch: '<S1816>/Switch1' */

    /* Gain: '<S1903>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1895>/Filter'
     *  Gain: '<S1894>/Derivative Gain'
     *  Sum: '<S1895>/SumD'
     */
    rtb_FilterCoefficient_e4 = (rtP.ANG_S_D_CH2_6 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_f) * 100.0F;

    /* Sum: '<S1909>/Sum' incorporates:
     *  DiscreteIntegrator: '<S1900>/Integrator'
     *  Gain: '<S1905>/Proportional Gain'
     */
    u0 = (rtP.ANG_S_P_CH2_6 * rtb_Sum_pp + rtDW.Integrator_DSTATE_bg) +
      rtb_FilterCoefficient_e4;

    /* Saturate: '<S1907>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_6' */
      rtY.yaw_ANG_OUT_CH2_6 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_6' */
      rtY.yaw_ANG_OUT_CH2_6 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_6' */
      rtY.yaw_ANG_OUT_CH2_6 = u0;
    }

    /* End of Saturate: '<S1907>/Saturation' */

    /* Chart: '<S1816>/Chart1' incorporates:
     *  Inport: '<Root>/yaw_ecd_CH2_6'
     *  Inport: '<Root>/yaw_last_ecd_CH2_6'
     *  Sum: '<S1816>/Sum3'
     */
    Chart1(rtU.yaw_ecd_CH2_6 - rtU.yaw_last_ecd_CH2_6, &rtDW.u2_g,
           &rtDW.sf_Chart1_nb);

    /* Outport: '<Root>/yaw_circle_out_CH2_6' incorporates:
     *  Inport: '<Root>/yaw_circle_CH2_6'
     *  Sum: '<S1816>/Sum4'
     */
    rtY.yaw_circle_out_CH2_6 = rtDW.u2_g + rtU.yaw_circle_CH2_6;

    /* Update for DiscreteIntegrator: '<S1948>/Integrator' incorporates:
     *  Gain: '<S1945>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_h4 += rtP.ANG_A_I_CH2_6 * rtb_FilterCoefficient_k *
      0.001F;

    /* Update for DiscreteIntegrator: '<S1943>/Filter' */
    rtDW.Filter_DSTATE_o += 0.001F * rtb_FilterCoefficient_dy;

    /* Update for DiscreteIntegrator: '<S1895>/Filter' */
    rtDW.Filter_DSTATE_f += 0.001F * rtb_FilterCoefficient_e4;

    /* Update for DiscreteIntegrator: '<S1900>/Integrator' incorporates:
     *  Gain: '<S1897>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_bg += rtP.ANG_S_I_CH2_6 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S13>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S13>/Switch Case' */

  /* SwitchCase: '<S14>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_status_CH2_7'
   */
  switch ((int32_T)rtU.yaw_status_CH2_7) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S14>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S1965>/Action Port'
     */
    /* Sum: '<S1965>/Sum' incorporates:
     *  Inport: '<Root>/yaw_speed_rpm_CH2_7'
     *  Inport: '<Root>/yaw_target_CH2_7'
     */
    rtb_Sum_pp = rtU.yaw_target_CH2_7 - rtU.yaw_speed_rpm_CH2_7;

    /* Gain: '<S2002>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S1994>/Filter'
     *  Gain: '<S1993>/Derivative Gain'
     *  Sum: '<S1994>/SumD'
     */
    rtb_FilterCoefficient_k = (rtP.SPD_D_CH2_7 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_j) * 100.0F;

    /* Sum: '<S2008>/Sum' incorporates:
     *  DiscreteIntegrator: '<S1999>/Integrator'
     *  Gain: '<S2004>/Proportional Gain'
     */
    u0 = (rtP.SPD_P_CH2_7 * rtb_Sum_pp + rtDW.Integrator_DSTATE_b) +
      rtb_FilterCoefficient_k;

    /* Saturate: '<S2006>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_7' */
      rtY.yaw_SPD_OUT_CH2_7 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_7' */
      rtY.yaw_SPD_OUT_CH2_7 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_SPD_OUT_CH2_7' */
      rtY.yaw_SPD_OUT_CH2_7 = u0;
    }

    /* End of Saturate: '<S2006>/Saturation' */

    /* Update for DiscreteIntegrator: '<S1994>/Filter' */
    rtDW.Filter_DSTATE_j += 0.001F * rtb_FilterCoefficient_k;

    /* Update for DiscreteIntegrator: '<S1999>/Integrator' incorporates:
     *  Gain: '<S1996>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_b += rtP.SPD_I_CH2_7 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S14>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S14>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S1966>/Action Port'
     */
    /* Sum: '<S1966>/Sum' incorporates:
     *  Gain: '<S1966>/Gain'
     *  Inport: '<Root>/yaw_circle_CH2_7'
     *  Inport: '<Root>/yaw_ecd_CH2_7'
     *  Inport: '<Root>/yaw_target_CH2_7'
     *  Sum: '<S1966>/Sum2'
     */
    rtb_FilterCoefficient_k = rtU.yaw_target_CH2_7 - (8191.0F *
      rtU.yaw_circle_CH2_7 + rtU.yaw_ecd_CH2_7);

    /* Abs: '<S1966>/Abs' */
    rtb_Sum_pp = (real32_T)fabs(rtb_FilterCoefficient_k);

    /* Switch: '<S1966>/Switch2' incorporates:
     *  Constant: '<S1966>/Constant'
     */
    if (!(rtb_Sum_pp > rtP.DEADBAND_CH2_7)) {
      rtb_FilterCoefficient_k = 0.0F;
    }

    /* End of Switch: '<S1966>/Switch2' */

    /* Gain: '<S2101>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S2093>/Filter'
     *  Gain: '<S2092>/Derivative Gain'
     *  Sum: '<S2093>/SumD'
     */
    rtb_FilterCoefficient_dy = (rtP.ANG_A_D_CH2_7 * rtb_FilterCoefficient_k -
      rtDW.Filter_DSTATE) * 100.0F;

    /* Switch: '<S1966>/Switch1' incorporates:
     *  Constant: '<S1966>/Constant'
     *  Gain: '<S1966>/Multiply'
     *  Inport: '<Root>/yaw_speed_rpm_CH2_7'
     *  Saturate: '<S2105>/Saturation'
     *  Sum: '<S1966>/Sum1'
     */
    if (rtb_Sum_pp > rtP.DEADBAND_CH2_7) {
      /* Sum: '<S2107>/Sum' incorporates:
       *  DiscreteIntegrator: '<S2098>/Integrator'
       *  Gain: '<S2103>/Proportional Gain'
       */
      u0 = (rtP.ANG_A_P_CH2_7 * rtb_FilterCoefficient_k + rtDW.Integrator_DSTATE)
        + rtb_FilterCoefficient_dy;

      /* Saturate: '<S2105>/Saturation' */
      if (u0 > 16384.0F) {
        u0 = 16384.0F;
      } else if (u0 < -16384.0F) {
        u0 = -16384.0F;
      }

      rtb_Sum_pp = rtP.TRANS_CH2_7 * u0 - rtU.yaw_speed_rpm_CH2_7;
    } else {
      rtb_Sum_pp = 0.0F;
    }

    /* End of Switch: '<S1966>/Switch1' */

    /* Gain: '<S2053>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S2045>/Filter'
     *  Gain: '<S2044>/Derivative Gain'
     *  Sum: '<S2045>/SumD'
     */
    rtb_FilterCoefficient_e4 = (rtP.ANG_S_D_CH2_7 * rtb_Sum_pp -
      rtDW.Filter_DSTATE_c) * 100.0F;

    /* Sum: '<S2059>/Sum' incorporates:
     *  DiscreteIntegrator: '<S2050>/Integrator'
     *  Gain: '<S2055>/Proportional Gain'
     */
    u0 = (rtP.ANG_S_P_CH2_7 * rtb_Sum_pp + rtDW.Integrator_DSTATE_h) +
      rtb_FilterCoefficient_e4;

    /* Saturate: '<S2057>/Saturation' */
    if (u0 > 16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_7' */
      rtY.yaw_ANG_OUT_CH2_7 = 16384.0F;
    } else if (u0 < -16384.0F) {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_7' */
      rtY.yaw_ANG_OUT_CH2_7 = -16384.0F;
    } else {
      /* Outport: '<Root>/yaw_ANG_OUT_CH2_7' */
      rtY.yaw_ANG_OUT_CH2_7 = u0;
    }

    /* End of Saturate: '<S2057>/Saturation' */

    /* Chart: '<S1966>/Chart1' incorporates:
     *  Inport: '<Root>/yaw_ecd_CH2_7'
     *  Inport: '<Root>/yaw_last_ecd_CH2_7'
     *  Sum: '<S1966>/Sum3'
     */
    Chart1(rtU.yaw_ecd_CH2_7 - rtU.yaw_last_ecd_CH2_7, &rtDW.u2,
           &rtDW.sf_Chart1_bf);

    /* Outport: '<Root>/yaw_circle_out_CH2_7' incorporates:
     *  Inport: '<Root>/yaw_circle_CH2_7'
     *  Sum: '<S1966>/Sum4'
     */
    rtY.yaw_circle_out_CH2_7 = rtDW.u2 + rtU.yaw_circle_CH2_7;

    /* Update for DiscreteIntegrator: '<S2098>/Integrator' incorporates:
     *  Gain: '<S2095>/Integral Gain'
     */
    rtDW.Integrator_DSTATE += rtP.ANG_A_I_CH2_7 * rtb_FilterCoefficient_k *
      0.001F;

    /* Update for DiscreteIntegrator: '<S2093>/Filter' */
    rtDW.Filter_DSTATE += 0.001F * rtb_FilterCoefficient_dy;

    /* Update for DiscreteIntegrator: '<S2045>/Filter' */
    rtDW.Filter_DSTATE_c += 0.001F * rtb_FilterCoefficient_e4;

    /* Update for DiscreteIntegrator: '<S2050>/Integrator' incorporates:
     *  Gain: '<S2047>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_h += rtP.ANG_S_I_CH2_7 * rtb_Sum_pp * 0.001F;

    /* End of Outputs for SubSystem: '<S14>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S14>/Switch Case' */
}

/* Model initialize function */
void PID_MODEL_initialize(void)
{
  /* SystemInitialize for IfAction SubSystem: '<S8>/If Action_speed Subsystem1' */

  /* SystemInitialize for Chart: '<S1066>/Chart1' */
  Chart1_Init(&rtDW.u2_o);

  /* End of SystemInitialize for SubSystem: '<S8>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S1>/If Action_speed Subsystem1' */

  /* SystemInitialize for Chart: '<S16>/Chart1' */
  Chart1_Init(&rtDW.u2_ae);

  /* End of SystemInitialize for SubSystem: '<S1>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S2>/If Action_speed Subsystem1' */

  /* SystemInitialize for Chart: '<S166>/Chart1' */
  Chart1_Init(&rtDW.u2_m);

  /* End of SystemInitialize for SubSystem: '<S2>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S3>/If Action_speed Subsystem1' */

  /* SystemInitialize for Chart: '<S316>/Chart1' */
  Chart1_Init(&rtDW.u2_nv);

  /* End of SystemInitialize for SubSystem: '<S3>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S4>/If Action_speed Subsystem1' */

  /* SystemInitialize for Chart: '<S466>/Chart1' */
  Chart1_Init(&rtDW.u2_f);

  /* End of SystemInitialize for SubSystem: '<S4>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S5>/If Action_speed Subsystem1' */

  /* SystemInitialize for Chart: '<S616>/Chart1' */
  Chart1_Init(&rtDW.u2_c);

  /* End of SystemInitialize for SubSystem: '<S5>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S6>/If Action_speed Subsystem1' */

  /* SystemInitialize for Chart: '<S766>/Chart1' */
  Chart1_Init(&rtDW.u2_jm);

  /* End of SystemInitialize for SubSystem: '<S6>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S7>/If Action_speed Subsystem1' */

  /* SystemInitialize for Chart: '<S916>/Chart1' */
  Chart1_Init(&rtDW.u2_nr);

  /* End of SystemInitialize for SubSystem: '<S7>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S9>/If Action_speed Subsystem1' */

  /* SystemInitialize for Chart: '<S1216>/Chart1' */
  Chart1_Init(&rtDW.u2_a);

  /* End of SystemInitialize for SubSystem: '<S9>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S10>/If Action_speed Subsystem1' */

  /* SystemInitialize for Chart: '<S1366>/Chart1' */
  Chart1_Init(&rtDW.u2_ge);

  /* End of SystemInitialize for SubSystem: '<S10>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S11>/If Action_speed Subsystem1' */

  /* SystemInitialize for Chart: '<S1516>/Chart1' */
  Chart1_Init(&rtDW.u2_j);

  /* End of SystemInitialize for SubSystem: '<S11>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S12>/If Action_speed Subsystem1' */

  /* SystemInitialize for Chart: '<S1666>/Chart1' */
  Chart1_Init(&rtDW.u2_n);

  /* End of SystemInitialize for SubSystem: '<S12>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S13>/If Action_speed Subsystem1' */

  /* SystemInitialize for Chart: '<S1816>/Chart1' */
  Chart1_Init(&rtDW.u2_g);

  /* End of SystemInitialize for SubSystem: '<S13>/If Action_speed Subsystem1' */

  /* SystemInitialize for IfAction SubSystem: '<S14>/If Action_speed Subsystem1' */

  /* SystemInitialize for Chart: '<S1966>/Chart1' */
  Chart1_Init(&rtDW.u2);

  /* End of SystemInitialize for SubSystem: '<S14>/If Action_speed Subsystem1' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
