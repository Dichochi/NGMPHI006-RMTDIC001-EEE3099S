/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MicroMouseTemplate.c
 *
 * Code generated for Simulink model 'MicroMouseTemplate'.
 *
 * Model version                  : 3.52
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Thu Oct 17 19:07:36 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "MicroMouseTemplate.h"
#include "rtwtypes.h"
#include "MicroMouseTemplate_types.h"
#include "MicroMouseTemplate_private.h"
#include <string.h>
#include <math.h>
#include "stm_timer_ll.h"
#include "stm_adc_ll.h"

/* Named constants for Chart: '<S5>/SENSOR_OUTPUT_Processing' */
#define MicroMous_IN_U_TURN_CALIBRATION ((uint8_T)17U)
#define MicroMouseTem_IN_CalibrateOTHER ((uint8_T)1U)
#define MicroMouseTem_IN_CalibrateSIDES ((uint8_T)2U)
#define MicroMouseTem_IN_Line_Following ((uint8_T)8U)
#define MicroMouseTempl_IN_Detect_Sides ((uint8_T)4U)
#define MicroMouseTempl_IN_Intersection ((uint8_T)7U)
#define MicroMouseTemplat_IN_Correction ((uint8_T)3U)
#define MicroMouseTemplat_IN_Initialise ((uint8_T)6U)
#define MicroMouseTemplate_IN_FINISH   ((uint8_T)5U)
#define MicroMouseTemplate_IN_No_Turn  ((uint8_T)9U)
#define MicroMouseTemplate_IN_ON_MODE  ((uint8_T)1U)
#define MicroMouseTemplate_IN_RUN_Done ((uint8_T)10U)
#define MicroMouseTemplate_IN_Return   ((uint8_T)11U)
#define MicroMouseTemplate_IN_START    ((uint8_T)2U)
#define MicroMouseTemplate_IN_Spin_L   ((uint8_T)12U)
#define MicroMouseTemplate_IN_Spin_R   ((uint8_T)13U)
#define MicroMouseTemplate_IN_Stop     ((uint8_T)14U)
#define MicroMouseTemplate_IN_TurnL    ((uint8_T)15U)
#define MicroMouseTemplate_IN_TurnR    ((uint8_T)16U)
#define MicroMouseTemplate_IN_U_Turn   ((uint8_T)18U)
#define MicroMouseTemplate_IN_WAIT     ((uint8_T)19U)

/* user code (top of source file) */
/* System '<Root>' */
extern I2C_HandleTypeDef hi2c2;

/* Exported block states */
real32_T IMU_Accel[3];                 /* '<S7>/Data Store Memory' */
real32_T IMU_Gyro[3];                  /* '<S7>/Data Store Memory1' */
int32_T currTicksRS;                   /* '<S1>/Data Store Memory2' */
int32_T currTicksLS;                   /* '<S1>/Data Store Memory4' */
uint16_T ADC1s[9];                     /* '<S6>/Data Store Memory' */
uint16_T ADC_H[9];                     /* '<S6>/Data Store Memory1' */
uint16_T ADC_L[9];                     /* '<S6>/Data Store Memory2' */
uint16_T Thresholds[8];                /* '<S1>/Data Store Memory1' */
boolean_T Detections[8];               /* '<S1>/Data Store Memory' */

/* Block signals (default storage) */
B_MicroMouseTemplate_T MicroMouseTemplate_B;

/* Block states (default storage) */
DW_MicroMouseTemplate_T MicroMouseTemplate_DW;

/* Real-time model */
static RT_MODEL_MicroMouseTemplate_T MicroMouseTemplate_M_;
RT_MODEL_MicroMouseTemplate_T *const MicroMouseTemplate_M =
  &MicroMouseTemplate_M_;

/* Forward declaration for local functions */
static void MicroMouseTemplate_Detect_Sides(const real_T Quantizer[8]);
static void MicroMouseT_enter_atomic_Return(void);
static void MicroMouseTemp_SystemCore_setup(stm32cube_blocks_AnalogInput__T *obj);
static void MicroMouseT_PWMOutput_setupImpl(stm32cube_blocks_PWMOutput_Mi_T *obj);
static void MicroMous_PWMOutput_setupImpl_d(stm32cube_blocks_PWMOutput_Mi_T *obj);

/* System initialize for atomic system: */
void MicroMouseTe_MATLABSystem1_Init(DW_MATLABSystem1_MicroMouseTe_T *localDW,
  P_MATLABSystem1_MicroMouseTem_T *localP)
{
  int32_T i;

  /* Start for MATLABSystem: '<S48>/MATLAB System1' */
  localDW->objisempty = true;
  localDW->obj.InitialCondition = localP->MATLABSystem1_InitialCondition;
  localDW->obj.isInitialized = 1;

  /*  Number of inputs */
  /*  Initialize the buffer during the first call */
  for (i = 0; i < 25; i++) {
    localDW->obj.Buffer[i] = localDW->obj.InitialCondition;
  }

  /* End of Start for MATLABSystem: '<S48>/MATLAB System1' */
}

/* Output and update for atomic system: */
void MicroMouseTemplat_MATLABSystem1(real_T rtu_0,
  B_MATLABSystem1_MicroMouseTem_T *localB, DW_MATLABSystem1_MicroMouseTe_T
  *localDW, P_MATLABSystem1_MicroMouseTem_T *localP)
{
  /* MATLABSystem: '<S48>/MATLAB System1' */
  if (localDW->obj.InitialCondition != localP->MATLABSystem1_InitialCondition) {
    localDW->obj.InitialCondition = localP->MATLABSystem1_InitialCondition;
  }

  /*  Number of inputs */
  /*  Number of outputs */
  /*  Output the current state of the buffer */
  memcpy(&localB->MATLABSystem1[0], &localDW->obj.Buffer[0], 25U * sizeof(real_T));

  /*  Update the buffer */
  localB->rtu_0[0] = rtu_0;
  memcpy(&localB->rtu_0[1], &localDW->obj.Buffer[0], 24U * sizeof(real_T));
  memcpy(&localDW->obj.Buffer[0], &localB->rtu_0[0], 25U * sizeof(real_T));

  /* End of MATLABSystem: '<S48>/MATLAB System1' */
}

/* System initialize for atomic system: */
void MicroMouseTe_MATLABSystem3_Init(DW_MATLABSystem3_MicroMouseTe_T *localDW,
  P_MATLABSystem3_MicroMouseTem_T *localP)
{
  int32_T i;

  /* Start for MATLABSystem: '<S48>/MATLAB System3' */
  localDW->objisempty = true;
  localDW->obj.InitialCondition = localP->MATLABSystem3_InitialCondition;
  localDW->obj.isInitialized = 1;

  /*  Number of inputs */
  /*  Initialize the buffer during the first call */
  for (i = 0; i < 10; i++) {
    localDW->obj.Buffer[i] = localDW->obj.InitialCondition;
  }

  /* End of Start for MATLABSystem: '<S48>/MATLAB System3' */
}

/* Output and update for atomic system: */
void MicroMouseTemplat_MATLABSystem3(real_T rtu_0,
  B_MATLABSystem3_MicroMouseTem_T *localB, DW_MATLABSystem3_MicroMouseTe_T
  *localDW, P_MATLABSystem3_MicroMouseTem_T *localP)
{
  /* MATLABSystem: '<S48>/MATLAB System3' */
  if (localDW->obj.InitialCondition != localP->MATLABSystem3_InitialCondition) {
    localDW->obj.InitialCondition = localP->MATLABSystem3_InitialCondition;
  }

  /*  Number of inputs */
  /*  Number of outputs */
  /*  Output the current state of the buffer */
  memcpy(&localB->MATLABSystem3[0], &localDW->obj.Buffer[0], 10U * sizeof(real_T));

  /*  Update the buffer */
  localB->rtu_0[0] = rtu_0;
  memcpy(&localB->rtu_0[1], &localDW->obj.Buffer[0], 9U * sizeof(real_T));
  memcpy(&localDW->obj.Buffer[0], &localB->rtu_0[0], 10U * sizeof(real_T));

  /* End of MATLABSystem: '<S48>/MATLAB System3' */
}

real_T rt_roundd(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/* Function for Chart: '<S5>/SENSOR_OUTPUT_Processing' */
static void MicroMouseTemplate_Detect_Sides(const real_T Quantizer[8])
{
  boolean_T out;
  out = ((!MicroMouseTemplate_DW.bitsForTID0.RIGHT_Wall) &&
         (MicroMouseTemplate_DW.temporalCounter_i1 >= 2U));
  if (out) {
    MicroMouseTemplate_B.IR_LED_FWD = 1.0;
    MicroMouseTemplate_B.IR_LED_SIDES = 0.0;
    MicroMouseTemplate_DW.temporalCounter_i1 = 0U;
    MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE = MicroMouseTemplate_IN_TurnR;
    MicroMouseTemplate_DW.GOAL_R++;
  } else {
    out = ((!MicroMouseTemplate_DW.bitsForTID0.FWD_Wall) &&
           (MicroMouseTemplate_DW.temporalCounter_i1 >= 2U));
    if (out) {
      MicroMouseTemplate_B.IR_LED_FWD = 1.0;
      MicroMouseTemplate_B.IR_LED_SIDES = 0.0;
      MicroMouseTemplate_DW.temporalCounter_i1 = 0U;
      MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
        MicroMouseTemplate_IN_No_Turn;
      MicroMouseTemplate_DW.GOAL_R = 0.0;
      MicroMouseTemplate_DW.GOAL_L = 0.0;
    } else {
      out = ((!MicroMouseTemplate_DW.bitsForTID0.LEFT_Wall) &&
             (MicroMouseTemplate_DW.temporalCounter_i1 >= 2U));
      if (out) {
        MicroMouseTemplate_B.IR_LED_FWD = 1.0;
        MicroMouseTemplate_B.IR_LED_SIDES = 0.0;
        MicroMouseTemplate_DW.temporalCounter_i1 = 0U;
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTemplate_IN_TurnL;
        MicroMouseTemplate_DW.GOAL_L++;
      } else {
        out = (MicroMouseTemplate_DW.bitsForTID0.RIGHT_Wall &&
               MicroMouseTemplate_DW.bitsForTID0.LEFT_Wall &&
               (MicroMouseTemplate_DW.temporalCounter_i1 >= 2U));
        if (out) {
          MicroMouseTemplate_B.IR_LED_FWD = 1.0;
          MicroMouseTemplate_B.IR_LED_SIDES = 0.0;
          MicroMouseTemplate_DW.temporalCounter_i1 = 0U;
          MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
            MicroMouseTemplate_IN_U_Turn;
          MicroMouseTemplate_DW.GOAL_R = 0.0;
          MicroMouseTemplate_DW.GOAL_L = 0.0;
        } else {
          MicroMouseTemplate_DW.bitsForTID0.RIGHT_Wall = (Quantizer[3] >
            MicroMouseTemplate_DW.RS_Threshold);
          MicroMouseTemplate_DW.bitsForTID0.LEFT_Wall = (Quantizer[4] >
            MicroMouseTemplate_DW.LS_Threshold);
        }
      }
    }
  }
}

/* Function for Chart: '<S5>/SENSOR_OUTPUT_Processing' */
static void MicroMouseT_enter_atomic_Return(void)
{
  MicroMouseTemplate_B.LW_F = 0.0;
  MicroMouseTemplate_B.RW_F = 0.0;
  MicroMouseTemplate_B.LW_B = 0.0;
  MicroMouseTemplate_B.RW_B = 0.0;
  MicroMouseTemplate_DW.bitsForTID0.FINISH = ((MicroMouseTemplate_DW.GOAL_R ==
    3.0) || (MicroMouseTemplate_DW.GOAL_L == 3.0));
  MicroMouseTemplate_DW.bitsForTID0.FWD_Wall = false;
  MicroMouseTemplate_DW.bitsForTID0.RIGHT_Wall = false;
  MicroMouseTemplate_DW.bitsForTID0.LEFT_Wall = false;
  MicroMouseTemplate_B.LED0 = false;
  MicroMouseTemplate_B.LED1 = false;
  MicroMouseTemplate_B.LED2 = false;
}

static void MicroMouseTemp_SystemCore_setup(stm32cube_blocks_AnalogInput__T *obj)
{
  ADC_Type_T adcStructLoc;
  obj->isSetupComplete = false;

  /* Start for MATLABSystem: '<S47>/Analog to Digital Converter' */
  obj->isInitialized = 1;
  adcStructLoc.InternalBufferPtr = (void*)(&obj->ADCInternalBuffer[0]);
  adcStructLoc.InjectedNoOfConversion = 0U;
  adcStructLoc.peripheralPtr = ADC2;
  adcStructLoc.dmaPeripheralPtr = DMA1;
  adcStructLoc.dmastream = LL_DMA_CHANNEL_2;
  adcStructLoc.DataTransferMode = ADC_DR_TRANSFER;
  adcStructLoc.DmaTransferMode = ADC_DMA_TRANSFER_UNLIMITED;
  adcStructLoc.InternalBufferSize = 9U;
  adcStructLoc.RegularNoOfConversion = 9U;
  obj->ADCHandle = ADC_Handle_Init(&adcStructLoc, ADC_DMA_INTERRUPT_MODE, 1,
    ADC_TRIGGER_AND_READ, LL_ADC_REG_SEQ_SCAN_ENABLE_9RANKS);
  enableADCAutomaticCalibrationOffset(obj->ADCHandle, 2);
  enableADC(obj->ADCHandle);
  startADCConversionForExternalTrigger(obj->ADCHandle, 1);
  obj->isSetupComplete = true;
}

static void MicroMouseT_PWMOutput_setupImpl(stm32cube_blocks_PWMOutput_Mi_T *obj)
{
  TIM_Type_T b;
  boolean_T isSlaveModeTriggerEnabled;

  /* Start for MATLABSystem: '<S40>/PWM Output' */
  b.PeripheralPtr = TIM3;
  b.isCenterAlignedMode = false;

  /* Start for MATLABSystem: '<S40>/PWM Output' */
  b.repetitionCounter = 0U;
  obj->TimerHandle = Timer_Handle_Init(&b);
  enableTimerInterrupts(obj->TimerHandle, 0);
  enableTimerChannel3(obj->TimerHandle, ENABLE_CH);
  enableTimerChannel4(obj->TimerHandle, ENABLE_CH);
  isSlaveModeTriggerEnabled = isSlaveTriggerModeEnabled(obj->TimerHandle);
  if (!isSlaveModeTriggerEnabled) {
    /* Start for MATLABSystem: '<S40>/PWM Output' */
    enableCounter(obj->TimerHandle, false);
  }
}

static void MicroMous_PWMOutput_setupImpl_d(stm32cube_blocks_PWMOutput_Mi_T *obj)
{
  TIM_Type_T b;
  boolean_T isSlaveModeTriggerEnabled;

  /* Start for MATLABSystem: '<S42>/PWM Output' */
  b.PeripheralPtr = TIM3;
  b.isCenterAlignedMode = false;

  /* Start for MATLABSystem: '<S42>/PWM Output' */
  b.repetitionCounter = 0U;
  obj->TimerHandle = Timer_Handle_Init(&b);
  enableTimerInterrupts(obj->TimerHandle, 0);
  enableTimerChannel1(obj->TimerHandle, ENABLE_CH);
  enableTimerChannel2(obj->TimerHandle, ENABLE_CH);
  isSlaveModeTriggerEnabled = isSlaveTriggerModeEnabled(obj->TimerHandle);
  if (!isSlaveModeTriggerEnabled) {
    /* Start for MATLABSystem: '<S42>/PWM Output' */
    enableCounter(obj->TimerHandle, false);
  }
}

/* Model step function */
void MicroMouseTemplate_step(void)
{
  real_T Gain;
  real_T maxV;
  real_T maxV_0;
  real_T maxV_1;
  real_T rtb_FWD_RSANDFWD_LS;
  real_T rtb_RSANDLS;
  int32_T i;
  uint32_T pinReadLoc;
  boolean_T NOT;
  boolean_T NOT1;

  /* MATLABSystem: '<S47>/Analog to Digital Converter' */
  regularReadADCDMA(MicroMouseTemplate_DW.obj.ADCHandle, ADC_TRIGGER_AND_READ,
                    &ADC1s[0]);

  /* DSPFlip: '<S6>/Flip' incorporates:
   *  MATLABSystem: '<S47>/Analog to Digital Converter'
   */
  MicroMouseTemplate_B.Flip[0] = ADC1s[7];
  MicroMouseTemplate_B.Flip[7] = ADC1s[0];
  MicroMouseTemplate_B.Flip[1] = ADC1s[6];
  MicroMouseTemplate_B.Flip[6] = ADC1s[1];
  MicroMouseTemplate_B.Flip[2] = ADC1s[5];
  MicroMouseTemplate_B.Flip[5] = ADC1s[2];
  MicroMouseTemplate_B.Flip[3] = ADC1s[4];
  MicroMouseTemplate_B.Flip[4] = ADC1s[3];

  /* DataTypeConversion: '<S50>/Cast To Double' */
  for (i = 0; i < 8; i++) {
    MicroMouseTemplate_B.CastToDouble[i] = MicroMouseTemplate_B.Flip[i];
  }

  /* End of DataTypeConversion: '<S50>/Cast To Double' */
  MicroMouseTemplat_MATLABSystem1(MicroMouseTemplate_B.CastToDouble[0],
    &MicroMouseTemplate_B.MATLABSystem2_ci,
    &MicroMouseTemplate_DW.MATLABSystem2_ci,
    &MicroMouseTemplate_P.MATLABSystem2_ci);

  /* MinMax: '<S48>/Max of Elements' incorporates:
   *  MATLABSystem: '<S50>/MATLAB System2'
   */
  MicroMouseTemplate_B.maxV =
    MicroMouseTemplate_B.MATLABSystem2_ci.MATLABSystem1[0];
  for (i = 0; i < 24; i++) {
    rtb_RSANDLS = MicroMouseTemplate_B.MATLABSystem2_ci.MATLABSystem1[i + 1];
    if (MicroMouseTemplate_B.maxV < rtb_RSANDLS) {
      MicroMouseTemplate_B.maxV = rtb_RSANDLS;
    }
  }

  MicroMouseTemplat_MATLABSystem3(MicroMouseTemplate_B.CastToDouble[1],
    &MicroMouseTemplate_B.MATLABSystem4, &MicroMouseTemplate_DW.MATLABSystem4,
    &MicroMouseTemplate_P.MATLABSystem4);

  /* MinMax: '<S48>/Max of Elements1' incorporates:
   *  MATLABSystem: '<S48>/MATLAB System4'
   */
  MicroMouseTemplate_B.maxV_m =
    MicroMouseTemplate_B.MATLABSystem4.MATLABSystem3[0];
  for (i = 0; i < 9; i++) {
    rtb_RSANDLS = MicroMouseTemplate_B.MATLABSystem4.MATLABSystem3[i + 1];
    if (MicroMouseTemplate_B.maxV_m < rtb_RSANDLS) {
      MicroMouseTemplate_B.maxV_m = rtb_RSANDLS;
    }
  }

  MicroMouseTemplat_MATLABSystem1(MicroMouseTemplate_B.maxV_m,
    &MicroMouseTemplate_B.MATLABSystem1, &MicroMouseTemplate_DW.MATLABSystem1,
    &MicroMouseTemplate_P.MATLABSystem1);

  /* End of MinMax: '<S48>/Max of Elements1' */

  /* MinMax: '<S48>/Min' incorporates:
   *  MATLABSystem: '<S48>/MATLAB System1'
   */
  MicroMouseTemplate_B.error = MicroMouseTemplate_B.MATLABSystem1.MATLABSystem1
    [0];
  for (i = 0; i < 24; i++) {
    rtb_RSANDLS = MicroMouseTemplate_B.MATLABSystem1.MATLABSystem1[i + 1];
    if (MicroMouseTemplate_B.error > rtb_RSANDLS) {
      MicroMouseTemplate_B.error = rtb_RSANDLS;
    }
  }

  MicroMouseTemplat_MATLABSystem1(MicroMouseTemplate_B.CastToDouble[2],
    &MicroMouseTemplate_B.MATLABSystem1_ci,
    &MicroMouseTemplate_DW.MATLABSystem1_ci,
    &MicroMouseTemplate_P.MATLABSystem1_ci);

  /* MinMax: '<S48>/Max of Elements2' incorporates:
   *  MATLABSystem: '<S50>/MATLAB System1'
   */
  MicroMouseTemplate_B.maxV_m =
    MicroMouseTemplate_B.MATLABSystem1_ci.MATLABSystem1[0];
  for (i = 0; i < 24; i++) {
    rtb_RSANDLS = MicroMouseTemplate_B.MATLABSystem1_ci.MATLABSystem1[i + 1];
    if (MicroMouseTemplate_B.maxV_m < rtb_RSANDLS) {
      MicroMouseTemplate_B.maxV_m = rtb_RSANDLS;
    }
  }

  MicroMouseTemplat_MATLABSystem1(MicroMouseTemplate_B.CastToDouble[3],
    &MicroMouseTemplate_B.MATLABSystem3_c,
    &MicroMouseTemplate_DW.MATLABSystem3_c,
    &MicroMouseTemplate_P.MATLABSystem3_c);

  /* MinMax: '<S48>/Max of Elements3' incorporates:
   *  MATLABSystem: '<S50>/MATLAB System3'
   */
  maxV = MicroMouseTemplate_B.MATLABSystem3_c.MATLABSystem1[0];
  for (i = 0; i < 24; i++) {
    rtb_RSANDLS = MicroMouseTemplate_B.MATLABSystem3_c.MATLABSystem1[i + 1];
    if (maxV < rtb_RSANDLS) {
      maxV = rtb_RSANDLS;
    }
  }

  MicroMouseTemplat_MATLABSystem1(MicroMouseTemplate_B.CastToDouble[4],
    &MicroMouseTemplate_B.MATLABSystem4_c,
    &MicroMouseTemplate_DW.MATLABSystem4_c,
    &MicroMouseTemplate_P.MATLABSystem4_c);

  /* MinMax: '<S48>/Max of Elements4' incorporates:
   *  MATLABSystem: '<S50>/MATLAB System4'
   */
  maxV_0 = MicroMouseTemplate_B.MATLABSystem4_c.MATLABSystem1[0];
  for (i = 0; i < 24; i++) {
    rtb_RSANDLS = MicroMouseTemplate_B.MATLABSystem4_c.MATLABSystem1[i + 1];
    if (maxV_0 < rtb_RSANDLS) {
      maxV_0 = rtb_RSANDLS;
    }
  }

  MicroMouseTemplat_MATLABSystem1(MicroMouseTemplate_B.CastToDouble[5],
    &MicroMouseTemplate_B.MATLABSystem5, &MicroMouseTemplate_DW.MATLABSystem5,
    &MicroMouseTemplate_P.MATLABSystem5);

  /* MinMax: '<S48>/Max of Elements5' incorporates:
   *  MATLABSystem: '<S50>/MATLAB System5'
   */
  Gain = MicroMouseTemplate_B.MATLABSystem5.MATLABSystem1[0];
  for (i = 0; i < 24; i++) {
    rtb_RSANDLS = MicroMouseTemplate_B.MATLABSystem5.MATLABSystem1[i + 1];
    if (Gain < rtb_RSANDLS) {
      Gain = rtb_RSANDLS;
    }
  }

  MicroMouseTemplat_MATLABSystem3(MicroMouseTemplate_B.CastToDouble[6],
    &MicroMouseTemplate_B.MATLABSystem3, &MicroMouseTemplate_DW.MATLABSystem3,
    &MicroMouseTemplate_P.MATLABSystem3);

  /* MinMax: '<S48>/Max of Elements6' incorporates:
   *  MATLABSystem: '<S48>/MATLAB System3'
   */
  maxV_1 = MicroMouseTemplate_B.MATLABSystem3.MATLABSystem3[0];
  for (i = 0; i < 9; i++) {
    rtb_RSANDLS = MicroMouseTemplate_B.MATLABSystem3.MATLABSystem3[i + 1];
    if (maxV_1 < rtb_RSANDLS) {
      maxV_1 = rtb_RSANDLS;
    }
  }

  MicroMouseTemplat_MATLABSystem1(maxV_1, &MicroMouseTemplate_B.MATLABSystem2,
    &MicroMouseTemplate_DW.MATLABSystem2, &MicroMouseTemplate_P.MATLABSystem2);

  /* End of MinMax: '<S48>/Max of Elements6' */

  /* MinMax: '<S48>/Min1' incorporates:
   *  MATLABSystem: '<S48>/MATLAB System2'
   */
  rtb_FWD_RSANDFWD_LS = MicroMouseTemplate_B.MATLABSystem2.MATLABSystem1[0];
  for (i = 0; i < 24; i++) {
    rtb_RSANDLS = MicroMouseTemplate_B.MATLABSystem2.MATLABSystem1[i + 1];
    if (rtb_FWD_RSANDFWD_LS > rtb_RSANDLS) {
      rtb_FWD_RSANDFWD_LS = rtb_RSANDLS;
    }
  }

  MicroMouseTemplat_MATLABSystem1(MicroMouseTemplate_B.CastToDouble[7],
    &MicroMouseTemplate_B.MATLABSystem6, &MicroMouseTemplate_DW.MATLABSystem6,
    &MicroMouseTemplate_P.MATLABSystem6);

  /* MinMax: '<S48>/Max of Elements7' incorporates:
   *  MATLABSystem: '<S50>/MATLAB System6'
   */
  maxV_1 = MicroMouseTemplate_B.MATLABSystem6.MATLABSystem1[0];
  for (i = 0; i < 24; i++) {
    rtb_RSANDLS = MicroMouseTemplate_B.MATLABSystem6.MATLABSystem1[i + 1];
    if (maxV_1 < rtb_RSANDLS) {
      maxV_1 = rtb_RSANDLS;
    }
  }

  /* MinMax: '<S48>/Max of Elements' */
  rtb_RSANDLS = fmod(floor(MicroMouseTemplate_B.maxV), 65536.0);

  /* DataStoreWrite: '<S45>/ADC_H Write' incorporates:
   *  MinMax: '<S48>/Max of Elements'
   */
  ADC_H[0] = (uint16_T)(rtb_RSANDLS < 0.0 ? (int32_T)(uint16_T)-(int16_T)
                        (uint16_T)-rtb_RSANDLS : (int32_T)(uint16_T)rtb_RSANDLS);

  /* MinMax: '<S48>/Min' */
  rtb_RSANDLS = fmod(floor(MicroMouseTemplate_B.error), 65536.0);

  /* DataStoreWrite: '<S45>/ADC_H Write' incorporates:
   *  MinMax: '<S48>/Min'
   */
  ADC_H[1] = (uint16_T)(rtb_RSANDLS < 0.0 ? (int32_T)(uint16_T)-(int16_T)
                        (uint16_T)-rtb_RSANDLS : (int32_T)(uint16_T)rtb_RSANDLS);

  /* MinMax: '<S48>/Max of Elements2' */
  rtb_RSANDLS = fmod(floor(MicroMouseTemplate_B.maxV_m), 65536.0);

  /* DataStoreWrite: '<S45>/ADC_H Write' incorporates:
   *  MinMax: '<S48>/Max of Elements2'
   */
  ADC_H[2] = (uint16_T)(rtb_RSANDLS < 0.0 ? (int32_T)(uint16_T)-(int16_T)
                        (uint16_T)-rtb_RSANDLS : (int32_T)(uint16_T)rtb_RSANDLS);

  /* MinMax: '<S48>/Max of Elements3' */
  rtb_RSANDLS = fmod(floor(maxV), 65536.0);

  /* DataStoreWrite: '<S45>/ADC_H Write' incorporates:
   *  MinMax: '<S48>/Max of Elements3'
   */
  ADC_H[3] = (uint16_T)(rtb_RSANDLS < 0.0 ? (int32_T)(uint16_T)-(int16_T)
                        (uint16_T)-rtb_RSANDLS : (int32_T)(uint16_T)rtb_RSANDLS);

  /* MinMax: '<S48>/Max of Elements4' */
  rtb_RSANDLS = fmod(floor(maxV_0), 65536.0);

  /* DataStoreWrite: '<S45>/ADC_H Write' incorporates:
   *  MinMax: '<S48>/Max of Elements4'
   */
  ADC_H[4] = (uint16_T)(rtb_RSANDLS < 0.0 ? (int32_T)(uint16_T)-(int16_T)
                        (uint16_T)-rtb_RSANDLS : (int32_T)(uint16_T)rtb_RSANDLS);

  /* MinMax: '<S48>/Max of Elements5' */
  rtb_RSANDLS = fmod(floor(Gain), 65536.0);

  /* DataStoreWrite: '<S45>/ADC_H Write' incorporates:
   *  MinMax: '<S48>/Max of Elements5'
   */
  ADC_H[5] = (uint16_T)(rtb_RSANDLS < 0.0 ? (int32_T)(uint16_T)-(int16_T)
                        (uint16_T)-rtb_RSANDLS : (int32_T)(uint16_T)rtb_RSANDLS);

  /* MinMax: '<S48>/Min1' */
  rtb_RSANDLS = fmod(floor(rtb_FWD_RSANDFWD_LS), 65536.0);

  /* DataStoreWrite: '<S45>/ADC_H Write' incorporates:
   *  MinMax: '<S48>/Min1'
   */
  ADC_H[6] = (uint16_T)(rtb_RSANDLS < 0.0 ? (int32_T)(uint16_T)-(int16_T)
                        (uint16_T)-rtb_RSANDLS : (int32_T)(uint16_T)rtb_RSANDLS);

  /* MinMax: '<S48>/Max of Elements7' */
  rtb_RSANDLS = fmod(floor(maxV_1), 65536.0);

  /* DataStoreWrite: '<S45>/ADC_H Write' incorporates:
   *  Constant: '<S45>/Constant'
   *  MinMax: '<S48>/Max of Elements7'
   */
  ADC_H[7] = (uint16_T)(rtb_RSANDLS < 0.0 ? (int32_T)(uint16_T)-(int16_T)
                        (uint16_T)-rtb_RSANDLS : (int32_T)(uint16_T)rtb_RSANDLS);
  ADC_H[8] = MicroMouseTemplate_P.Constant_Value_b;

  /* MinMax: '<S49>/Max of Elements8' incorporates:
   *  MATLABSystem: '<S50>/MATLAB System2'
   */
  MicroMouseTemplate_B.error =
    MicroMouseTemplate_B.MATLABSystem2_ci.MATLABSystem1[0];
  for (i = 0; i < 24; i++) {
    rtb_RSANDLS = MicroMouseTemplate_B.MATLABSystem2_ci.MATLABSystem1[i + 1];
    if (MicroMouseTemplate_B.error > rtb_RSANDLS) {
      MicroMouseTemplate_B.error = rtb_RSANDLS;
    }
  }

  MicroMouseTemplat_MATLABSystem1(MicroMouseTemplate_B.CastToDouble[1],
    &MicroMouseTemplate_B.MATLABSystem1_c,
    &MicroMouseTemplate_DW.MATLABSystem1_c,
    &MicroMouseTemplate_P.MATLABSystem1_c);

  /* MinMax: '<S49>/Max of Elements9' incorporates:
   *  MATLABSystem: '<S49>/MATLAB System1'
   */
  rtb_FWD_RSANDFWD_LS = MicroMouseTemplate_B.MATLABSystem1_c.MATLABSystem1[0];
  for (i = 0; i < 24; i++) {
    rtb_RSANDLS = MicroMouseTemplate_B.MATLABSystem1_c.MATLABSystem1[i + 1];
    if (rtb_FWD_RSANDFWD_LS > rtb_RSANDLS) {
      rtb_FWD_RSANDFWD_LS = rtb_RSANDLS;
    }
  }

  /* MinMax: '<S49>/Max of Elements10' incorporates:
   *  MATLABSystem: '<S50>/MATLAB System1'
   */
  MicroMouseTemplate_B.maxV_m =
    MicroMouseTemplate_B.MATLABSystem1_ci.MATLABSystem1[0];
  for (i = 0; i < 24; i++) {
    rtb_RSANDLS = MicroMouseTemplate_B.MATLABSystem1_ci.MATLABSystem1[i + 1];
    if (MicroMouseTemplate_B.maxV_m > rtb_RSANDLS) {
      MicroMouseTemplate_B.maxV_m = rtb_RSANDLS;
    }
  }

  /* MinMax: '<S49>/Max of Elements11' incorporates:
   *  MATLABSystem: '<S50>/MATLAB System3'
   */
  maxV = MicroMouseTemplate_B.MATLABSystem3_c.MATLABSystem1[0];
  for (i = 0; i < 24; i++) {
    rtb_RSANDLS = MicroMouseTemplate_B.MATLABSystem3_c.MATLABSystem1[i + 1];
    if (maxV > rtb_RSANDLS) {
      maxV = rtb_RSANDLS;
    }
  }

  /* MinMax: '<S49>/Max of Elements12' incorporates:
   *  MATLABSystem: '<S50>/MATLAB System4'
   */
  maxV_0 = MicroMouseTemplate_B.MATLABSystem4_c.MATLABSystem1[0];
  for (i = 0; i < 24; i++) {
    rtb_RSANDLS = MicroMouseTemplate_B.MATLABSystem4_c.MATLABSystem1[i + 1];
    if (maxV_0 > rtb_RSANDLS) {
      maxV_0 = rtb_RSANDLS;
    }
  }

  /* MinMax: '<S49>/Max of Elements13' incorporates:
   *  MATLABSystem: '<S50>/MATLAB System5'
   */
  Gain = MicroMouseTemplate_B.MATLABSystem5.MATLABSystem1[0];
  for (i = 0; i < 24; i++) {
    rtb_RSANDLS = MicroMouseTemplate_B.MATLABSystem5.MATLABSystem1[i + 1];
    if (Gain > rtb_RSANDLS) {
      Gain = rtb_RSANDLS;
    }
  }

  MicroMouseTemplat_MATLABSystem1(MicroMouseTemplate_B.CastToDouble[6],
    &MicroMouseTemplate_B.MATLABSystem2_c,
    &MicroMouseTemplate_DW.MATLABSystem2_c,
    &MicroMouseTemplate_P.MATLABSystem2_c);

  /* MinMax: '<S49>/Max of Elements14' incorporates:
   *  MATLABSystem: '<S49>/MATLAB System2'
   */
  maxV_1 = MicroMouseTemplate_B.MATLABSystem2_c.MATLABSystem1[0];
  for (i = 0; i < 24; i++) {
    rtb_RSANDLS = MicroMouseTemplate_B.MATLABSystem2_c.MATLABSystem1[i + 1];
    if (maxV_1 > rtb_RSANDLS) {
      maxV_1 = rtb_RSANDLS;
    }
  }

  /* MinMax: '<S49>/Max of Elements15' incorporates:
   *  MATLABSystem: '<S50>/MATLAB System6'
   */
  MicroMouseTemplate_B.maxV = MicroMouseTemplate_B.MATLABSystem6.MATLABSystem1[0];
  for (i = 0; i < 24; i++) {
    rtb_RSANDLS = MicroMouseTemplate_B.MATLABSystem6.MATLABSystem1[i + 1];
    if (MicroMouseTemplate_B.maxV > rtb_RSANDLS) {
      MicroMouseTemplate_B.maxV = rtb_RSANDLS;
    }
  }

  /* MinMax: '<S49>/Max of Elements8' */
  rtb_RSANDLS = fmod(floor(MicroMouseTemplate_B.error), 65536.0);

  /* DataStoreWrite: '<S45>/ADC_L Write' incorporates:
   *  MinMax: '<S49>/Max of Elements8'
   */
  ADC_L[0] = (uint16_T)(rtb_RSANDLS < 0.0 ? (int32_T)(uint16_T)-(int16_T)
                        (uint16_T)-rtb_RSANDLS : (int32_T)(uint16_T)rtb_RSANDLS);

  /* MinMax: '<S49>/Max of Elements9' */
  rtb_RSANDLS = fmod(floor(rtb_FWD_RSANDFWD_LS), 65536.0);

  /* DataStoreWrite: '<S45>/ADC_L Write' incorporates:
   *  MinMax: '<S49>/Max of Elements9'
   */
  ADC_L[1] = (uint16_T)(rtb_RSANDLS < 0.0 ? (int32_T)(uint16_T)-(int16_T)
                        (uint16_T)-rtb_RSANDLS : (int32_T)(uint16_T)rtb_RSANDLS);

  /* MinMax: '<S49>/Max of Elements10' */
  rtb_RSANDLS = fmod(floor(MicroMouseTemplate_B.maxV_m), 65536.0);

  /* DataStoreWrite: '<S45>/ADC_L Write' incorporates:
   *  MinMax: '<S49>/Max of Elements10'
   */
  ADC_L[2] = (uint16_T)(rtb_RSANDLS < 0.0 ? (int32_T)(uint16_T)-(int16_T)
                        (uint16_T)-rtb_RSANDLS : (int32_T)(uint16_T)rtb_RSANDLS);

  /* MinMax: '<S49>/Max of Elements11' */
  rtb_RSANDLS = fmod(floor(maxV), 65536.0);

  /* DataStoreWrite: '<S45>/ADC_L Write' incorporates:
   *  MinMax: '<S49>/Max of Elements11'
   */
  ADC_L[3] = (uint16_T)(rtb_RSANDLS < 0.0 ? (int32_T)(uint16_T)-(int16_T)
                        (uint16_T)-rtb_RSANDLS : (int32_T)(uint16_T)rtb_RSANDLS);

  /* MinMax: '<S49>/Max of Elements12' */
  rtb_RSANDLS = fmod(floor(maxV_0), 65536.0);

  /* DataStoreWrite: '<S45>/ADC_L Write' incorporates:
   *  MinMax: '<S49>/Max of Elements12'
   */
  ADC_L[4] = (uint16_T)(rtb_RSANDLS < 0.0 ? (int32_T)(uint16_T)-(int16_T)
                        (uint16_T)-rtb_RSANDLS : (int32_T)(uint16_T)rtb_RSANDLS);

  /* MinMax: '<S49>/Max of Elements13' */
  rtb_RSANDLS = fmod(floor(Gain), 65536.0);

  /* DataStoreWrite: '<S45>/ADC_L Write' incorporates:
   *  MinMax: '<S49>/Max of Elements13'
   */
  ADC_L[5] = (uint16_T)(rtb_RSANDLS < 0.0 ? (int32_T)(uint16_T)-(int16_T)
                        (uint16_T)-rtb_RSANDLS : (int32_T)(uint16_T)rtb_RSANDLS);

  /* MinMax: '<S49>/Max of Elements14' */
  rtb_RSANDLS = fmod(floor(maxV_1), 65536.0);

  /* DataStoreWrite: '<S45>/ADC_L Write' incorporates:
   *  MinMax: '<S49>/Max of Elements14'
   */
  ADC_L[6] = (uint16_T)(rtb_RSANDLS < 0.0 ? (int32_T)(uint16_T)-(int16_T)
                        (uint16_T)-rtb_RSANDLS : (int32_T)(uint16_T)rtb_RSANDLS);

  /* MinMax: '<S49>/Max of Elements15' */
  rtb_RSANDLS = fmod(floor(MicroMouseTemplate_B.maxV), 65536.0);

  /* DataStoreWrite: '<S45>/ADC_L Write' incorporates:
   *  Constant: '<S45>/Constant1'
   *  MinMax: '<S49>/Max of Elements15'
   */
  ADC_L[7] = (uint16_T)(rtb_RSANDLS < 0.0 ? (int32_T)(uint16_T)-(int16_T)
                        (uint16_T)-rtb_RSANDLS : (int32_T)(uint16_T)rtb_RSANDLS);
  ADC_L[8] = MicroMouseTemplate_P.Constant1_Value_c;
  for (i = 0; i < 8; i++) {
    /* DataStoreWrite: '<S1>/Data Store Write' */
    Detections[i] = false;

    /* DataStoreWrite: '<S1>/Data Store Write1' */
    Thresholds[i] = 0U;
  }

  /* DataStoreWrite: '<S1>/Data Store Write2' */
  currTicksRS = 0;

  /* DataStoreWrite: '<S1>/Data Store Write3' */
  currTicksLS = 0;

  /* Quantizer: '<Root>/Quantizer' incorporates:
   *  DSPFlip: '<S6>/Flip'
   *  DataTypeConversion: '<Root>/Cast To Double'
   *  Gain: '<Root>/Gain'
   *  MATLABSystem: '<S47>/Analog to Digital Converter'
   */
  MicroMouseTemplate_B.error = rt_roundd((real_T)((uint32_T)
    MicroMouseTemplate_P.Gain_Gain_o * ADC1s[0]) * 1.4901161193847656E-8 /
    MicroMouseTemplate_P.Quantizer_Interval) *
    MicroMouseTemplate_P.Quantizer_Interval;

  /* Quantizer: '<Root>/Quantizer' */
  MicroMouseTemplate_B.CastToDouble[0] = MicroMouseTemplate_B.error;

  /* Quantizer: '<Root>/Quantizer' incorporates:
   *  DSPFlip: '<S6>/Flip'
   *  DataTypeConversion: '<Root>/Cast To Double'
   *  Gain: '<Root>/Gain'
   *  MATLABSystem: '<S47>/Analog to Digital Converter'
   */
  Gain = rt_roundd((real_T)((uint32_T)MicroMouseTemplate_P.Gain_Gain_o * ADC1s[1])
                   * 1.4901161193847656E-8 /
                   MicroMouseTemplate_P.Quantizer_Interval) *
    MicroMouseTemplate_P.Quantizer_Interval;

  /* Quantizer: '<Root>/Quantizer' */
  MicroMouseTemplate_B.CastToDouble[1] = Gain;

  /* Quantizer: '<Root>/Quantizer' incorporates:
   *  DSPFlip: '<S6>/Flip'
   *  DataTypeConversion: '<Root>/Cast To Double'
   *  Gain: '<Root>/Gain'
   *  MATLABSystem: '<S47>/Analog to Digital Converter'
   */
  rtb_FWD_RSANDFWD_LS = rt_roundd((real_T)((uint32_T)
    MicroMouseTemplate_P.Gain_Gain_o * ADC1s[2]) * 1.4901161193847656E-8 /
    MicroMouseTemplate_P.Quantizer_Interval) *
    MicroMouseTemplate_P.Quantizer_Interval;

  /* Quantizer: '<Root>/Quantizer' */
  MicroMouseTemplate_B.CastToDouble[2] = rtb_FWD_RSANDFWD_LS;

  /* Quantizer: '<Root>/Quantizer' incorporates:
   *  DSPFlip: '<S6>/Flip'
   *  DataTypeConversion: '<Root>/Cast To Double'
   *  Gain: '<Root>/Gain'
   *  MATLABSystem: '<S47>/Analog to Digital Converter'
   */
  rtb_RSANDLS = rt_roundd((real_T)((uint32_T)MicroMouseTemplate_P.Gain_Gain_o *
    ADC1s[3]) * 1.4901161193847656E-8 / MicroMouseTemplate_P.Quantizer_Interval)
    * MicroMouseTemplate_P.Quantizer_Interval;

  /* Quantizer: '<Root>/Quantizer' */
  MicroMouseTemplate_B.CastToDouble[3] = rtb_RSANDLS;

  /* Quantizer: '<Root>/Quantizer' incorporates:
   *  DSPFlip: '<S6>/Flip'
   *  DataTypeConversion: '<Root>/Cast To Double'
   *  Gain: '<Root>/Gain'
   *  MATLABSystem: '<S47>/Analog to Digital Converter'
   */
  MicroMouseTemplate_B.maxV_m = rt_roundd((real_T)((uint32_T)
    MicroMouseTemplate_P.Gain_Gain_o * ADC1s[4]) * 1.4901161193847656E-8 /
    MicroMouseTemplate_P.Quantizer_Interval) *
    MicroMouseTemplate_P.Quantizer_Interval;

  /* Quantizer: '<Root>/Quantizer' */
  MicroMouseTemplate_B.CastToDouble[4] = MicroMouseTemplate_B.maxV_m;

  /* Quantizer: '<Root>/Quantizer' incorporates:
   *  DSPFlip: '<S6>/Flip'
   *  DataTypeConversion: '<Root>/Cast To Double'
   *  Gain: '<Root>/Gain'
   *  MATLABSystem: '<S47>/Analog to Digital Converter'
   */
  maxV = rt_roundd((real_T)((uint32_T)MicroMouseTemplate_P.Gain_Gain_o * ADC1s[5])
                   * 1.4901161193847656E-8 /
                   MicroMouseTemplate_P.Quantizer_Interval) *
    MicroMouseTemplate_P.Quantizer_Interval;

  /* Quantizer: '<Root>/Quantizer' */
  MicroMouseTemplate_B.CastToDouble[5] = maxV;

  /* Quantizer: '<Root>/Quantizer' incorporates:
   *  DSPFlip: '<S6>/Flip'
   *  DataTypeConversion: '<Root>/Cast To Double'
   *  Gain: '<Root>/Gain'
   *  MATLABSystem: '<S47>/Analog to Digital Converter'
   */
  maxV_1 = rt_roundd((real_T)((uint32_T)MicroMouseTemplate_P.Gain_Gain_o *
    ADC1s[6]) * 1.4901161193847656E-8 / MicroMouseTemplate_P.Quantizer_Interval)
    * MicroMouseTemplate_P.Quantizer_Interval;

  /* Quantizer: '<Root>/Quantizer' */
  MicroMouseTemplate_B.CastToDouble[6] = maxV_1;

  /* Quantizer: '<Root>/Quantizer' incorporates:
   *  DSPFlip: '<S6>/Flip'
   *  DataTypeConversion: '<Root>/Cast To Double'
   *  Gain: '<Root>/Gain'
   *  MATLABSystem: '<S47>/Analog to Digital Converter'
   */
  maxV_0 = rt_roundd((real_T)((uint32_T)MicroMouseTemplate_P.Gain_Gain_o *
    ADC1s[7]) * 1.4901161193847656E-8 / MicroMouseTemplate_P.Quantizer_Interval)
    * MicroMouseTemplate_P.Quantizer_Interval;

  /* Quantizer: '<Root>/Quantizer' */
  MicroMouseTemplate_B.CastToDouble[7] = maxV_0;

  /* MATLABSystem: '<S66>/Digital Port Read' */
  pinReadLoc = LL_GPIO_ReadInputPort(GPIOE);

  /* Logic: '<S9>/NOT' incorporates:
   *  MATLABSystem: '<S66>/Digital Port Read'
   * */
  NOT = ((pinReadLoc & 64U) == 0U);

  /* MATLABSystem: '<S68>/Digital Port Read' */
  pinReadLoc = LL_GPIO_ReadInputPort(GPIOB);

  /* Logic: '<S9>/NOT1' incorporates:
   *  MATLABSystem: '<S68>/Digital Port Read'
   * */
  NOT1 = ((pinReadLoc & 4U) == 0U);

  /* Gain: '<S5>/Gain' incorporates:
   *  Sum: '<S5>/Add'
   */
  Gain = (Gain + maxV_1) * MicroMouseTemplate_P.Gain_Gain;

  /* Chart: '<S5>/SENSOR_OUTPUT_Processing' incorporates:
   *  Quantizer: '<Root>/Quantizer'
   */
  if (MicroMouseTemplate_DW.temporalCounter_i1 < 15U) {
    MicroMouseTemplate_DW.temporalCounter_i1++;
  }

  if (MicroMouseTemplate_DW.bitsForTID0.is_active_c3_MicroMouseTemplate == 0U) {
    MicroMouseTemplate_DW.bitsForTID0.is_active_c3_MicroMouseTemplate = 1U;
    MicroMouseTemplate_DW.bitsForTID0.is_c3_MicroMouseTemplate =
      MicroMouseTemplate_IN_START;
    MicroMouseTemplate_B.LED0 = true;
    MicroMouseTemplate_B.LED1 = true;
    MicroMouseTemplate_B.LED2 = true;
  } else if (MicroMouseTemplate_DW.bitsForTID0.is_c3_MicroMouseTemplate ==
             MicroMouseTemplate_IN_ON_MODE) {
    switch (MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE) {
     case MicroMouseTem_IN_CalibrateOTHER:
      if (MicroMouseTemplate_DW.temporalCounter_i1 >= 3U) {
        MicroMouseTemplate_B.IR_LED_FWD = 1.0;
        MicroMouseTemplate_B.IR_LED_SIDES = 0.0;
        MicroMouseTemplate_B.IR_LED_DOWN = 1.0;
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTemplate_IN_WAIT;
      } else {
        MicroMouseTemplate_DW.FWD_Threshold = Gain * 0.8;
      }
      break;

     case MicroMouseTem_IN_CalibrateSIDES:
      if (MicroMouseTemplate_DW.temporalCounter_i1 >= 3U) {
        MicroMouseTemplate_DW.temporalCounter_i1 = 0U;
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTem_IN_CalibrateOTHER;
        MicroMouseTemplate_B.IR_LED_FWD = 1.0;
        MicroMouseTemplate_B.IR_LED_SIDES = 0.0;
        MicroMouseTemplate_B.IR_LED_DOWN = 1.0;
      } else {
        MicroMouseTemplate_B.LED0 = !MicroMouseTemplate_B.LED0;
        MicroMouseTemplate_B.LED2 = !MicroMouseTemplate_B.LED2;
        MicroMouseTemplate_DW.RS_Threshold = rtb_RSANDLS * 0.95;
        MicroMouseTemplate_DW.LS_Threshold = MicroMouseTemplate_B.maxV_m * 0.8;
      }
      break;

     case MicroMouseTemplat_IN_Correction:
      if (MicroMouseTemplate_DW.temporalCounter_i1 >= 8U) {
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTem_IN_Line_Following;
        MicroMouseTemplate_B.LED0 = false;
        MicroMouseTemplate_B.LED1 = false;
      } else {
        MicroMouseTemplate_B.error = (rtb_FWD_RSANDFWD_LS - maxV) * 100.0 / 3.3;
        MicroMouseTemplate_B.LW_F = 79.5 - MicroMouseTemplate_B.error * 0.16;
        MicroMouseTemplate_B.RW_F = MicroMouseTemplate_B.error * 0.16 + 80.0;
      }
      break;

     case MicroMouseTempl_IN_Detect_Sides:
      MicroMouseTemplate_Detect_Sides(MicroMouseTemplate_B.CastToDouble);
      break;

     case MicroMouseTemplate_IN_FINISH:
      if (MicroMouseTemplate_DW.temporalCounter_i1 >= 2U) {
        MicroMouseTemplate_B.LED0 = false;
        MicroMouseTemplate_B.LED1 = false;
        MicroMouseTemplate_B.LED2 = false;
        MicroMouseTemplate_DW.temporalCounter_i1 = 0U;
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTemplate_IN_Spin_R;
      } else {
        MicroMouseTemplate_B.LED0 = !MicroMouseTemplate_B.LED0;
        MicroMouseTemplate_B.LED1 = !MicroMouseTemplate_B.LED1;
        MicroMouseTemplate_B.LED2 = !MicroMouseTemplate_B.LED2;
      }
      break;

     case MicroMouseTemplat_IN_Initialise:
      if (NOT1) {
        MicroMouseTemplate_DW.temporalCounter_i1 = 0U;
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTem_IN_CalibrateSIDES;
        MicroMouseTemplate_B.IR_LED_FWD = 0.0;
        MicroMouseTemplate_B.IR_LED_SIDES = 1.0;
      }
      break;

     case MicroMouseTempl_IN_Intersection:
      if ((MicroMouseTemplate_B.error > 3.25) && (maxV_0 > 3.25)) {
        MicroMouseTemplate_B.LW_F = 0.0;
        MicroMouseTemplate_B.RW_F = 0.0;
        MicroMouseTemplate_DW.temporalCounter_i1 = 0U;
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTempl_IN_Detect_Sides;
        MicroMouseTemplate_DW.bitsForTID0.FWD_Wall = (Gain >
          MicroMouseTemplate_DW.FWD_Threshold);
        MicroMouseTemplate_B.IR_LED_FWD = 0.0;
        MicroMouseTemplate_B.IR_LED_SIDES = 1.0;
      } else {
        MicroMouseTemplate_B.error = (rtb_FWD_RSANDFWD_LS - maxV) * 100.0 / 3.3;
        MicroMouseTemplate_B.LW_F = 80.0 - MicroMouseTemplate_B.error * 0.4;
        MicroMouseTemplate_B.RW_F = MicroMouseTemplate_B.error * 0.4 + 80.0;
      }
      break;

     case MicroMouseTem_IN_Line_Following:
      if (NOT1) {
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTemplate_IN_Stop;
        MicroMouseTemplate_B.LED0 = false;
        MicroMouseTemplate_B.LW_F = 0.0;
        MicroMouseTemplate_B.RW_F = 0.0;
        MicroMouseTemplate_B.LW_B = 0.0;
        MicroMouseTemplate_B.RW_B = 0.0;
      } else if ((MicroMouseTemplate_B.error < 3.25) && (maxV_0 < 3.25)) {
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTempl_IN_Intersection;
      } else {
        MicroMouseTemplate_B.error = (rtb_FWD_RSANDFWD_LS - maxV) * 100.0 / 3.3;
        MicroMouseTemplate_B.LW_F = 79.5 - MicroMouseTemplate_B.error * 0.4;
        MicroMouseTemplate_B.RW_F = MicroMouseTemplate_B.error * 0.4 + 80.0;
      }
      break;

     case MicroMouseTemplate_IN_No_Turn:
      MicroMouseTemplate_DW.temporalCounter_i1 = 0U;
      MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
        MicroMouseTemplate_IN_Return;
      MicroMouseT_enter_atomic_Return();
      break;

     case MicroMouseTemplate_IN_RUN_Done:
      if (MicroMouseTemplate_DW.temporalCounter_i1 >= 2U) {
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTemplate_IN_WAIT;
      }
      break;

     case MicroMouseTemplate_IN_Return:
      if (MicroMouseTemplate_DW.bitsForTID0.FINISH) {
        MicroMouseTemplate_DW.temporalCounter_i1 = 0U;
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTemplate_IN_FINISH;
        MicroMouseTemplate_B.LED0 = true;
        MicroMouseTemplate_B.LED1 = false;
        MicroMouseTemplate_B.LED2 = true;
      } else {
        MicroMouseTemplate_DW.temporalCounter_i1 = 0U;
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTemplat_IN_Correction;
      }
      break;

     case MicroMouseTemplate_IN_Spin_L:
      if (MicroMouseTemplate_DW.temporalCounter_i1 >= 11U) {
        MicroMouseTemplate_DW.temporalCounter_i1 = 0U;
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTemplate_IN_RUN_Done;
        MicroMouseTemplate_B.RW_F = 0.0;
        MicroMouseTemplate_B.LW_B = 0.0;
        MicroMouseTemplate_B.LED0 = false;
        MicroMouseTemplate_B.LED1 = false;
        MicroMouseTemplate_B.LED2 = false;
      } else {
        MicroMouseTemplate_B.RW_F = 90.0;
        MicroMouseTemplate_B.LW_B = 90.0;
      }
      break;

     case MicroMouseTemplate_IN_Spin_R:
      if (MicroMouseTemplate_DW.temporalCounter_i1 >= 11U) {
        MicroMouseTemplate_B.RW_B = 0.0;
        MicroMouseTemplate_B.LW_F = 0.0;
        MicroMouseTemplate_DW.temporalCounter_i1 = 0U;
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTemplate_IN_Spin_L;
      } else {
        MicroMouseTemplate_B.RW_B = 90.0;
        MicroMouseTemplate_B.LW_F = 90.0;
      }
      break;

     case MicroMouseTemplate_IN_Stop:
      if (NOT) {
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTemplate_IN_WAIT;
      }
      break;

     case MicroMouseTemplate_IN_TurnL:
      if (MicroMouseTemplate_DW.temporalCounter_i1 >= 6U) {
        MicroMouseTemplate_DW.temporalCounter_i1 = 0U;
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTemplate_IN_Return;
        MicroMouseT_enter_atomic_Return();
      } else {
        MicroMouseTemplate_B.LED0 = !MicroMouseTemplate_B.LED0;
        MicroMouseTemplate_B.RW_F = 90.0;
        MicroMouseTemplate_B.LW_B = 90.0;
      }
      break;

     case MicroMouseTemplate_IN_TurnR:
      if (MicroMouseTemplate_DW.temporalCounter_i1 >= 6U) {
        MicroMouseTemplate_DW.temporalCounter_i1 = 0U;
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTemplate_IN_Return;
        MicroMouseT_enter_atomic_Return();
      } else {
        MicroMouseTemplate_B.LED2 = !MicroMouseTemplate_B.LED2;
        MicroMouseTemplate_B.RW_B = 90.0;
        MicroMouseTemplate_B.LW_F = 90.0;
      }
      break;

     case MicroMous_IN_U_TURN_CALIBRATION:
      if (MicroMouseTemplate_DW.temporalCounter_i1 >= 11U) {
        MicroMouseTemplate_B.LED1 = false;
        MicroMouseTemplate_B.RW_F = 0.0;
        MicroMouseTemplate_B.LW_B = 0.0;
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTemplate_IN_WAIT;
      } else {
        MicroMouseTemplate_B.LED1 = !MicroMouseTemplate_B.LED1;
        MicroMouseTemplate_B.RW_F = 90.0;
        MicroMouseTemplate_B.LW_B = 90.0;
      }
      break;

     case MicroMouseTemplate_IN_U_Turn:
      if (MicroMouseTemplate_DW.temporalCounter_i1 >= 11U) {
        MicroMouseTemplate_DW.temporalCounter_i1 = 0U;
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTemplate_IN_Return;
        MicroMouseT_enter_atomic_Return();
      } else {
        MicroMouseTemplate_B.LED1 = !MicroMouseTemplate_B.LED1;
        MicroMouseTemplate_B.RW_F = 90.0;
        MicroMouseTemplate_B.LW_B = 90.0;
      }
      break;

     default:
      /* case IN_WAIT: */
      if (NOT) {
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMouseTem_IN_Line_Following;
        MicroMouseTemplate_B.LED0 = false;
        MicroMouseTemplate_B.LED1 = false;
      } else if (NOT1) {
        MicroMouseTemplate_DW.temporalCounter_i1 = 0U;
        MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
          MicroMous_IN_U_TURN_CALIBRATION;
      }
      break;
    }

    /* case IN_START: */
  } else if (NOT) {
    MicroMouseTemplate_DW.bitsForTID0.is_c3_MicroMouseTemplate =
      MicroMouseTemplate_IN_ON_MODE;
    MicroMouseTemplate_DW.bitsForTID0.is_ON_MODE =
      MicroMouseTemplat_IN_Initialise;
    MicroMouseTemplate_B.IR_LED_DOWN = 1.0;
    MicroMouseTemplate_B.IR_LED_FWD = 1.0;
    MicroMouseTemplate_B.IR_LED_SIDES = 0.0;
    MicroMouseTemplate_B.LED0 = false;
    MicroMouseTemplate_B.LED1 = false;
    MicroMouseTemplate_B.LED2 = false;
    MicroMouseTemplate_DW.GOAL_R = 0.0;
    MicroMouseTemplate_DW.GOAL_L = 0.0;
  }

  /* End of Chart: '<S5>/SENSOR_OUTPUT_Processing' */

  /* Switch: '<S3>/DOWN_RS AND DOWN_LS' incorporates:
   *  Constant: '<S3>/Constant'
   *  Constant: '<S3>/Constant1'
   */
  if (MicroMouseTemplate_B.IR_LED_DOWN >
      MicroMouseTemplate_P.DOWN_RSANDDOWN_LS_Threshold) {
    MicroMouseTemplate_B.error = MicroMouseTemplate_P.Constant_Value;
  } else {
    MicroMouseTemplate_B.error = MicroMouseTemplate_P.Constant1_Value;
  }

  /* End of Switch: '<S3>/DOWN_RS AND DOWN_LS' */

  /* Switch: '<S3>/FWD_RS AND FWD_LS' incorporates:
   *  Constant: '<S3>/Constant'
   *  Constant: '<S3>/Constant1'
   */
  if (MicroMouseTemplate_B.IR_LED_FWD >
      MicroMouseTemplate_P.FWD_RSANDFWD_LS_Threshold) {
    rtb_FWD_RSANDFWD_LS = MicroMouseTemplate_P.Constant_Value;
  } else {
    rtb_FWD_RSANDFWD_LS = MicroMouseTemplate_P.Constant1_Value;
  }

  /* End of Switch: '<S3>/FWD_RS AND FWD_LS' */

  /* Switch: '<S3>/RS AND LS' incorporates:
   *  Constant: '<S3>/Constant'
   *  Constant: '<S3>/Constant1'
   */
  if (MicroMouseTemplate_B.IR_LED_SIDES > MicroMouseTemplate_P.RSANDLS_Threshold)
  {
    rtb_RSANDLS = MicroMouseTemplate_P.Constant_Value;
  } else {
    rtb_RSANDLS = MicroMouseTemplate_P.Constant1_Value;
  }

  /* End of Switch: '<S3>/RS AND LS' */

  /* Outputs for Atomic SubSystem: '<Root>/GPIO for IR LEDs' */
  /* MATLABSystem: '<S19>/Digital Port Write' */
  MicroMouseTemplate_B.portNameLoc = GPIOE;
  if (MicroMouseTemplate_B.error != 0.0) {
    i = 512;
  } else {
    i = 0;
  }

  LL_GPIO_SetOutputPin(MicroMouseTemplate_B.portNameLoc, (uint32_T)i);
  LL_GPIO_ResetOutputPin(MicroMouseTemplate_B.portNameLoc, ~(uint32_T)i & 512U);

  /* End of MATLABSystem: '<S19>/Digital Port Write' */

  /* MATLABSystem: '<S21>/Digital Port Write' */
  MicroMouseTemplate_B.portNameLoc = GPIOE;
  if (MicroMouseTemplate_B.error != 0.0) {
    i = 16384;
  } else {
    i = 0;
  }

  LL_GPIO_SetOutputPin(MicroMouseTemplate_B.portNameLoc, (uint32_T)i);
  LL_GPIO_ResetOutputPin(MicroMouseTemplate_B.portNameLoc, ~(uint32_T)i & 16384U);

  /* End of MATLABSystem: '<S21>/Digital Port Write' */

  /* MATLABSystem: '<S23>/Digital Port Write' */
  MicroMouseTemplate_B.portNameLoc = GPIOE;
  if (rtb_FWD_RSANDFWD_LS != 0.0) {
    i = 256;
  } else {
    i = 0;
  }

  LL_GPIO_SetOutputPin(MicroMouseTemplate_B.portNameLoc, (uint32_T)i);
  LL_GPIO_ResetOutputPin(MicroMouseTemplate_B.portNameLoc, ~(uint32_T)i & 256U);

  /* End of MATLABSystem: '<S23>/Digital Port Write' */

  /* MATLABSystem: '<S25>/Digital Port Write' */
  MicroMouseTemplate_B.portNameLoc = GPIOE;
  if (rtb_FWD_RSANDFWD_LS != 0.0) {
    i = 32768;
  } else {
    i = 0;
  }

  LL_GPIO_SetOutputPin(MicroMouseTemplate_B.portNameLoc, (uint32_T)i);
  LL_GPIO_ResetOutputPin(MicroMouseTemplate_B.portNameLoc, ~(uint32_T)i & 32768U);

  /* End of MATLABSystem: '<S25>/Digital Port Write' */

  /* MATLABSystem: '<S27>/Digital Port Write' */
  MicroMouseTemplate_B.portNameLoc = GPIOE;
  if (rtb_RSANDLS != 0.0) {
    i = 4096;
  } else {
    i = 0;
  }

  LL_GPIO_SetOutputPin(MicroMouseTemplate_B.portNameLoc, (uint32_T)i);
  LL_GPIO_ResetOutputPin(MicroMouseTemplate_B.portNameLoc, ~(uint32_T)i & 4096U);

  /* End of MATLABSystem: '<S27>/Digital Port Write' */

  /* MATLABSystem: '<S29>/Digital Port Write' */
  MicroMouseTemplate_B.portNameLoc = GPIOB;
  if (MicroMouseTemplate_B.error != 0.0) {
    i = 4096;
  } else {
    i = 0;
  }

  LL_GPIO_SetOutputPin(MicroMouseTemplate_B.portNameLoc, (uint32_T)i);
  LL_GPIO_ResetOutputPin(MicroMouseTemplate_B.portNameLoc, ~(uint32_T)i & 4096U);

  /* End of MATLABSystem: '<S29>/Digital Port Write' */

  /* MATLABSystem: '<S31>/Digital Port Write' */
  MicroMouseTemplate_B.portNameLoc = GPIOE;
  if (MicroMouseTemplate_B.error != 0.0) {
    i = 8192;
  } else {
    i = 0;
  }

  LL_GPIO_SetOutputPin(MicroMouseTemplate_B.portNameLoc, (uint32_T)i);
  LL_GPIO_ResetOutputPin(MicroMouseTemplate_B.portNameLoc, ~(uint32_T)i & 8192U);

  /* End of MATLABSystem: '<S31>/Digital Port Write' */

  /* MATLABSystem: '<S33>/Digital Port Write' */
  MicroMouseTemplate_B.portNameLoc = GPIOE;
  if (rtb_RSANDLS != 0.0) {
    i = 2048;
  } else {
    i = 0;
  }

  LL_GPIO_SetOutputPin(MicroMouseTemplate_B.portNameLoc, (uint32_T)i);
  LL_GPIO_ResetOutputPin(MicroMouseTemplate_B.portNameLoc, ~(uint32_T)i & 2048U);

  /* End of MATLABSystem: '<S33>/Digital Port Write' */
  /* End of Outputs for SubSystem: '<Root>/GPIO for IR LEDs' */

  /* MATLABSystem: '<S40>/PWM Output' */
  setDutyCycleInPercentageChannel3(MicroMouseTemplate_DW.obj_n.TimerHandle,
    MicroMouseTemplate_B.LW_B);
  setDutyCycleInPercentageChannel4(MicroMouseTemplate_DW.obj_n.TimerHandle,
    MicroMouseTemplate_B.LW_F);

  /* MATLABSystem: '<S42>/PWM Output' */
  setDutyCycleInPercentageChannel1(MicroMouseTemplate_DW.obj_g.TimerHandle,
    MicroMouseTemplate_B.RW_F);
  setDutyCycleInPercentageChannel2(MicroMouseTemplate_DW.obj_g.TimerHandle,
    MicroMouseTemplate_B.RW_B);

  /* MATLABSystem: '<S38>/Digital Port Write' incorporates:
   *  Constant: '<S4>/Constant'
   */
  MicroMouseTemplate_B.portNameLoc = GPIOD;
  if (MicroMouseTemplate_P.Constant_Value_l != 0.0) {
    i = 128;
  } else {
    i = 0;
  }

  LL_GPIO_SetOutputPin(MicroMouseTemplate_B.portNameLoc, (uint32_T)i);
  LL_GPIO_ResetOutputPin(MicroMouseTemplate_B.portNameLoc, ~(uint32_T)i & 128U);

  /* End of MATLABSystem: '<S38>/Digital Port Write' */

  /* MATLABSystem: '<S58>/Digital Port Write' */
  MicroMouseTemplate_B.portNameLoc = GPIOC;
  if (MicroMouseTemplate_B.LED0) {
    i = 8192;
  } else {
    i = 0;
  }

  LL_GPIO_SetOutputPin(MicroMouseTemplate_B.portNameLoc, (uint32_T)i);
  LL_GPIO_ResetOutputPin(MicroMouseTemplate_B.portNameLoc, ~(uint32_T)i & 8192U);

  /* End of MATLABSystem: '<S58>/Digital Port Write' */

  /* MATLABSystem: '<S60>/Digital Port Write' */
  MicroMouseTemplate_B.portNameLoc = GPIOC;
  if (MicroMouseTemplate_B.LED1) {
    i = 16384;
  } else {
    i = 0;
  }

  LL_GPIO_SetOutputPin(MicroMouseTemplate_B.portNameLoc, (uint32_T)i);
  LL_GPIO_ResetOutputPin(MicroMouseTemplate_B.portNameLoc, ~(uint32_T)i & 16384U);

  /* End of MATLABSystem: '<S60>/Digital Port Write' */

  /* MATLABSystem: '<S62>/Digital Port Write' */
  MicroMouseTemplate_B.portNameLoc = GPIOC;
  if (MicroMouseTemplate_B.LED2) {
    i = 32768;
  } else {
    i = 0;
  }

  LL_GPIO_SetOutputPin(MicroMouseTemplate_B.portNameLoc, (uint32_T)i);
  LL_GPIO_ResetOutputPin(MicroMouseTemplate_B.portNameLoc, ~(uint32_T)i & 32768U);

  /* End of MATLABSystem: '<S62>/Digital Port Write' */

  /* MATLABSystem: '<S56>/Digital Port Write' incorporates:
   *  Constant: '<S8>/Constant'
   */
  MicroMouseTemplate_B.portNameLoc = GPIOB;
  if (MicroMouseTemplate_P.Constant_Value_i != 0.0) {
    i = 8;
  } else {
    i = 0;
  }

  LL_GPIO_SetOutputPin(MicroMouseTemplate_B.portNameLoc, (uint32_T)i);
  LL_GPIO_ResetOutputPin(MicroMouseTemplate_B.portNameLoc, ~(uint32_T)i & 8U);

  /* End of MATLABSystem: '<S56>/Digital Port Write' */

  /* user code (Update function Body) */

  /* System '<Root>' */
  refreshIMUValues();

  /* System '<Root>' */
  CustomWhile();
}

/* Model initialize function */
void MicroMouseTemplate_initialize(void)
{
  {
    int32_T i;
    for (i = 0; i < 9; i++) {
      /* Start for DataStoreMemory: '<S6>/Data Store Memory' */
      ADC1s[i] = MicroMouseTemplate_P.DataStoreMemory_InitialValue_p;

      /* Start for DataStoreMemory: '<S6>/Data Store Memory1' */
      ADC_H[i] = MicroMouseTemplate_P.DataStoreMemory1_InitialValue_p;

      /* Start for DataStoreMemory: '<S6>/Data Store Memory2' */
      ADC_L[i] = MicroMouseTemplate_P.DataStoreMemory2_InitialValue_p;
    }

    for (i = 0; i < 8; i++) {
      /* Start for DataStoreMemory: '<S1>/Data Store Memory' */
      Detections[i] = MicroMouseTemplate_P.DataStoreMemory_InitialValue_p4;

      /* Start for DataStoreMemory: '<S1>/Data Store Memory1' */
      Thresholds[i] = MicroMouseTemplate_P.DataStoreMemory1_InitialValue_j;
    }

    /* Start for DataStoreMemory: '<S1>/Data Store Memory2' */
    currTicksRS = MicroMouseTemplate_P.DataStoreMemory2_InitialValue;

    /* Start for DataStoreMemory: '<S1>/Data Store Memory4' */
    currTicksLS = MicroMouseTemplate_P.DataStoreMemory4_InitialValue;

    /* Start for DataStoreMemory: '<S7>/Data Store Memory' */
    IMU_Accel[0] = MicroMouseTemplate_P.DataStoreMemory_InitialValue;

    /* Start for DataStoreMemory: '<S7>/Data Store Memory1' */
    IMU_Gyro[0] = MicroMouseTemplate_P.DataStoreMemory1_InitialValue;

    /* Start for DataStoreMemory: '<S7>/Data Store Memory' */
    IMU_Accel[1] = MicroMouseTemplate_P.DataStoreMemory_InitialValue;

    /* Start for DataStoreMemory: '<S7>/Data Store Memory1' */
    IMU_Gyro[1] = MicroMouseTemplate_P.DataStoreMemory1_InitialValue;

    /* Start for DataStoreMemory: '<S7>/Data Store Memory' */
    IMU_Accel[2] = MicroMouseTemplate_P.DataStoreMemory_InitialValue;

    /* Start for DataStoreMemory: '<S7>/Data Store Memory1' */
    IMU_Gyro[2] = MicroMouseTemplate_P.DataStoreMemory1_InitialValue;

    /* user code (Initialize function Body) */

    /* System '<Root>' */
    initIMU();

    /* Start for MATLABSystem: '<S47>/Analog to Digital Converter' */
    MicroMouseTemplate_DW.obj.isInitialized = 0;
    MicroMouseTemplate_DW.obj.matlabCodegenIsDeleted = false;
    MicroMouseTemp_SystemCore_setup(&MicroMouseTemplate_DW.obj);
    MicroMouseTe_MATLABSystem1_Init(&MicroMouseTemplate_DW.MATLABSystem2_ci,
      &MicroMouseTemplate_P.MATLABSystem2_ci);
    MicroMouseTe_MATLABSystem3_Init(&MicroMouseTemplate_DW.MATLABSystem4,
      &MicroMouseTemplate_P.MATLABSystem4);
    MicroMouseTe_MATLABSystem1_Init(&MicroMouseTemplate_DW.MATLABSystem1,
      &MicroMouseTemplate_P.MATLABSystem1);
    MicroMouseTe_MATLABSystem1_Init(&MicroMouseTemplate_DW.MATLABSystem1_ci,
      &MicroMouseTemplate_P.MATLABSystem1_ci);
    MicroMouseTe_MATLABSystem1_Init(&MicroMouseTemplate_DW.MATLABSystem3_c,
      &MicroMouseTemplate_P.MATLABSystem3_c);
    MicroMouseTe_MATLABSystem1_Init(&MicroMouseTemplate_DW.MATLABSystem4_c,
      &MicroMouseTemplate_P.MATLABSystem4_c);
    MicroMouseTe_MATLABSystem1_Init(&MicroMouseTemplate_DW.MATLABSystem5,
      &MicroMouseTemplate_P.MATLABSystem5);
    MicroMouseTe_MATLABSystem3_Init(&MicroMouseTemplate_DW.MATLABSystem3,
      &MicroMouseTemplate_P.MATLABSystem3);
    MicroMouseTe_MATLABSystem1_Init(&MicroMouseTemplate_DW.MATLABSystem2,
      &MicroMouseTemplate_P.MATLABSystem2);
    MicroMouseTe_MATLABSystem1_Init(&MicroMouseTemplate_DW.MATLABSystem6,
      &MicroMouseTemplate_P.MATLABSystem6);
    MicroMouseTe_MATLABSystem1_Init(&MicroMouseTemplate_DW.MATLABSystem1_c,
      &MicroMouseTemplate_P.MATLABSystem1_c);
    MicroMouseTe_MATLABSystem1_Init(&MicroMouseTemplate_DW.MATLABSystem2_c,
      &MicroMouseTemplate_P.MATLABSystem2_c);

    /* Start for MATLABSystem: '<S40>/PWM Output' */
    MicroMouseTemplate_DW.obj_n.matlabCodegenIsDeleted = false;
    MicroMouseTemplate_DW.obj_n.isSetupComplete = false;
    MicroMouseTemplate_DW.obj_n.isInitialized = 1;
    MicroMouseT_PWMOutput_setupImpl(&MicroMouseTemplate_DW.obj_n);
    MicroMouseTemplate_DW.obj_n.isSetupComplete = true;

    /* Start for MATLABSystem: '<S42>/PWM Output' */
    MicroMouseTemplate_DW.obj_g.matlabCodegenIsDeleted = false;
    MicroMouseTemplate_DW.obj_g.isSetupComplete = false;
    MicroMouseTemplate_DW.obj_g.isInitialized = 1;
    MicroMous_PWMOutput_setupImpl_d(&MicroMouseTemplate_DW.obj_g);
    MicroMouseTemplate_DW.obj_g.isSetupComplete = true;
  }
}

/* Model terminate function */
void MicroMouseTemplate_terminate(void)
{
  /* Terminate for MATLABSystem: '<S47>/Analog to Digital Converter' */
  if (!MicroMouseTemplate_DW.obj.matlabCodegenIsDeleted) {
    MicroMouseTemplate_DW.obj.matlabCodegenIsDeleted = true;
    if ((MicroMouseTemplate_DW.obj.isInitialized == 1) &&
        MicroMouseTemplate_DW.obj.isSetupComplete) {
      ADC_Handle_Deinit(MicroMouseTemplate_DW.obj.ADCHandle,
                        ADC_DMA_INTERRUPT_MODE, 1);
    }
  }

  /* End of Terminate for MATLABSystem: '<S47>/Analog to Digital Converter' */

  /* Terminate for MATLABSystem: '<S40>/PWM Output' */
  if (!MicroMouseTemplate_DW.obj_n.matlabCodegenIsDeleted) {
    MicroMouseTemplate_DW.obj_n.matlabCodegenIsDeleted = true;
    if ((MicroMouseTemplate_DW.obj_n.isInitialized == 1) &&
        MicroMouseTemplate_DW.obj_n.isSetupComplete) {
      disableCounter(MicroMouseTemplate_DW.obj_n.TimerHandle);
      disableTimerInterrupts(MicroMouseTemplate_DW.obj_n.TimerHandle, 0);
      disableTimerChannel3(MicroMouseTemplate_DW.obj_n.TimerHandle, ENABLE_CH);
      disableTimerChannel4(MicroMouseTemplate_DW.obj_n.TimerHandle, ENABLE_CH);
    }
  }

  /* End of Terminate for MATLABSystem: '<S40>/PWM Output' */

  /* Terminate for MATLABSystem: '<S42>/PWM Output' */
  if (!MicroMouseTemplate_DW.obj_g.matlabCodegenIsDeleted) {
    MicroMouseTemplate_DW.obj_g.matlabCodegenIsDeleted = true;
    if ((MicroMouseTemplate_DW.obj_g.isInitialized == 1) &&
        MicroMouseTemplate_DW.obj_g.isSetupComplete) {
      disableCounter(MicroMouseTemplate_DW.obj_g.TimerHandle);
      disableTimerInterrupts(MicroMouseTemplate_DW.obj_g.TimerHandle, 0);
      disableTimerChannel1(MicroMouseTemplate_DW.obj_g.TimerHandle, ENABLE_CH);
      disableTimerChannel2(MicroMouseTemplate_DW.obj_g.TimerHandle, ENABLE_CH);
    }
  }

  /* End of Terminate for MATLABSystem: '<S42>/PWM Output' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
