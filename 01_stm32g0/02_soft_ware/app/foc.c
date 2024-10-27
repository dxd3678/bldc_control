/*
 * File: foc.c
 *
 * Code generated for Simulink model 'foc'.
 *
 * Model version                  : 1.39
 * Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
 * C/C++ source code generated on : Tue Jan 11 15:22:36 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "foc.h"

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Model step function */
void foc_step(void)
{
  int32_T sector;
  real32_T T1;
  real32_T T1_tmp;
  real32_T rtb_ualpha;
  real32_T rtb_ubeta;

  /* MATLAB Function: '<S1>/rePark' incorporates:
   *  Inport: '<Root>/theta'
   *  Inport: '<Root>/ud'
   *  Inport: '<Root>/uq'
   */
  T1 = sinf(rtU.theta);
  rtb_ubeta = cosf(rtU.theta);
  rtb_ualpha = (real32_T)rtU.ud * rtb_ubeta - (real32_T)rtU.uq * T1;
  rtb_ubeta = (real32_T)rtU.ud * T1 + (real32_T)rtU.uq * rtb_ubeta;

  /* MATLAB Function: '<S1>/SVPWM MATLAB Function' */
  sector = 0;

  /* Outport: '<Root>/Tcmp1' incorporates:
   *  MATLAB Function: '<S1>/SVPWM MATLAB Function'
   */
  rtY.Tcmp1 = 0.0f;

  /* Outport: '<Root>/Tcmp2' incorporates:
   *  MATLAB Function: '<S1>/SVPWM MATLAB Function'
   */
  rtY.Tcmp2 = 0.0f;

  /* Outport: '<Root>/Tcmp3' incorporates:
   *  MATLAB Function: '<S1>/SVPWM MATLAB Function'
   */
  rtY.Tcmp3 = 0.0f;

  /* MATLAB Function: '<S1>/SVPWM MATLAB Function' incorporates:
   *  Inport: '<Root>/Tpwm'
   *  Inport: '<Root>/udc'
   */
  if (rtb_ubeta > 0.0F) {
    sector = 1;
  }

  if ((1.73205078F * rtb_ualpha - rtb_ubeta) / 2.0F > 0.0F) {
    sector += 2;
  }

  if ((-1.73205078F * rtb_ualpha - rtb_ubeta) / 2.0F > 0.0F) {
    sector += 4;
  }

  switch (sector) {
   case 1:
    T1_tmp = rtU.Tpwm / rtU.udc;
    T1 = (-1.5F * rtb_ualpha + 0.866025388F * rtb_ubeta) * T1_tmp;
    rtb_ualpha = (1.5F * rtb_ualpha + 0.866025388F * rtb_ubeta) * T1_tmp;
    break;

   case 2:
    T1 = (1.5F * rtb_ualpha + 0.866025388F * rtb_ubeta) * (rtU.Tpwm / rtU.udc);
    rtb_ualpha = -(1.73205078F * rtb_ubeta * rtU.Tpwm / rtU.udc);
    break;

   case 3:
    T1 = -((-1.5F * rtb_ualpha + 0.866025388F * rtb_ubeta) * (rtU.Tpwm / rtU.udc));
    rtb_ualpha = 1.73205078F * rtb_ubeta * rtU.Tpwm / rtU.udc;
    break;

   case 4:
    T1 = -(1.73205078F * rtb_ubeta * rtU.Tpwm / rtU.udc);
    rtb_ualpha = (-1.5F * rtb_ualpha + 0.866025388F * rtb_ubeta) * (rtU.Tpwm /
      rtU.udc);
    break;

   case 5:
    T1 = 1.73205078F * rtb_ubeta * rtU.Tpwm / rtU.udc;
    rtb_ualpha = -((1.5F * rtb_ualpha + 0.866025388F * rtb_ubeta) * (rtU.Tpwm /
      rtU.udc));
    break;

   default:
    T1 = -((1.5F * rtb_ualpha + 0.866025388F * rtb_ubeta) * (rtU.Tpwm / rtU.udc));
    rtb_ualpha = -((-1.5F * rtb_ualpha + 0.866025388F * rtb_ubeta) * (rtU.Tpwm /
      rtU.udc));
    break;
  }

  rtb_ubeta = T1 + rtb_ualpha;
  if (rtb_ubeta > rtU.Tpwm) {
    T1 /= rtb_ubeta;
    rtb_ualpha /= T1 + rtb_ualpha;
  }

  rtb_ubeta = (rtU.Tpwm - (T1 + rtb_ualpha)) / 4.0F;
  T1 = T1 / 2.0F + rtb_ubeta;
  switch (sector) {
   case 1:
    /* Outport: '<Root>/Tcmp1' */
    rtY.Tcmp1 = T1;

    /* Outport: '<Root>/Tcmp2' */
    rtY.Tcmp2 = rtb_ubeta;

    /* Outport: '<Root>/Tcmp3' */
    rtY.Tcmp3 = rtb_ualpha / 2.0F + T1;
    break;

   case 2:
    /* Outport: '<Root>/Tcmp1' */
    rtY.Tcmp1 = rtb_ubeta;

    /* Outport: '<Root>/Tcmp2' */
    rtY.Tcmp2 = rtb_ualpha / 2.0F + T1;

    /* Outport: '<Root>/Tcmp3' */
    rtY.Tcmp3 = T1;
    break;

   case 3:
    /* Outport: '<Root>/Tcmp1' */
    rtY.Tcmp1 = rtb_ubeta;

    /* Outport: '<Root>/Tcmp2' */
    rtY.Tcmp2 = T1;

    /* Outport: '<Root>/Tcmp3' */
    rtY.Tcmp3 = rtb_ualpha / 2.0F + T1;
    break;

   case 4:
    /* Outport: '<Root>/Tcmp1' */
    rtY.Tcmp1 = rtb_ualpha / 2.0F + T1;

    /* Outport: '<Root>/Tcmp2' */
    rtY.Tcmp2 = T1;

    /* Outport: '<Root>/Tcmp3' */
    rtY.Tcmp3 = rtb_ubeta;
    break;

   case 5:
    /* Outport: '<Root>/Tcmp1' */
    rtY.Tcmp1 = rtb_ualpha / 2.0F + T1;

    /* Outport: '<Root>/Tcmp2' */
    rtY.Tcmp2 = rtb_ubeta;

    /* Outport: '<Root>/Tcmp3' */
    rtY.Tcmp3 = T1;
    break;

   case 6:
    /* Outport: '<Root>/Tcmp1' */
    rtY.Tcmp1 = T1;

    /* Outport: '<Root>/Tcmp2' */
    rtY.Tcmp2 = rtb_ualpha / 2.0F + T1;

    /* Outport: '<Root>/Tcmp3' */
    rtY.Tcmp3 = rtb_ubeta;
    break;
  }

  /* Outport: '<Root>/sector' incorporates:
   *  MATLAB Function: '<S1>/SVPWM MATLAB Function'
   */
  rtY.sector = (real32_T)sector;
}

/* Model initialize function */
void foc_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
