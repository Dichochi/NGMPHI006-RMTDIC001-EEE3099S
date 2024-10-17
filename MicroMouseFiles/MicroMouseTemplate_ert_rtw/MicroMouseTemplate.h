/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MicroMouseTemplate.h
 *
 * Code generated for Simulink model 'MicroMouseTemplate'.
 *
 * Model version                  : 3.50
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Thu Oct 17 18:17:46 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef MicroMouseTemplate_h_
#define MicroMouseTemplate_h_
#ifndef MicroMouseTemplate_COMMON_INCLUDES_
#define MicroMouseTemplate_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "main.h"
#endif                                 /* MicroMouseTemplate_COMMON_INCLUDES_ */

#include "MicroMouseTemplate_types.h"
#include <stddef.h>
#include "MW_target_hardware_resources.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* user code (top of header file) */
#include "MicroMouseProgramming\Core\Inc\IMU.h"
#include "MicroMouseProgramming\Core\Inc\CustomWhile.h"
#include "MicroMouseProgramming\Core\Inc\Motors.h"

/* Block signals for system '<S48>/MATLAB System1' */
typedef struct {
  real_T MATLABSystem1[25];            /* '<S48>/MATLAB System1' */
  real_T rtu_0[25];
} B_MATLABSystem1_MicroMouseTem_T;

/* Block states (default storage) for system '<S48>/MATLAB System1' */
typedef struct {
  CircularBuffer_MicroMouseTemp_T obj; /* '<S48>/MATLAB System1' */
  boolean_T objisempty;                /* '<S48>/MATLAB System1' */
} DW_MATLABSystem1_MicroMouseTe_T;

/* Block signals for system '<S48>/MATLAB System3' */
typedef struct {
  real_T MATLABSystem3[10];            /* '<S48>/MATLAB System3' */
  real_T rtu_0[10];
} B_MATLABSystem3_MicroMouseTem_T;

/* Block states (default storage) for system '<S48>/MATLAB System3' */
typedef struct {
  CircularBuffer_MicroMouseTe_p_T obj; /* '<S48>/MATLAB System3' */
  boolean_T objisempty;                /* '<S48>/MATLAB System3' */
} DW_MATLABSystem3_MicroMouseTe_T;

/* Block signals (default storage) */
typedef struct {
  real_T CastToDouble[8];              /* '<S50>/Cast To Double' */
  uint16_T Flip[8];                    /* '<S6>/Flip' */
  GPIO_TypeDef * portNameLoc;
  real_T IR_LED_DOWN;                  /* '<S5>/SENSOR_OUTPUT_Processing' */
  real_T IR_LED_FWD;                   /* '<S5>/SENSOR_OUTPUT_Processing' */
  real_T IR_LED_SIDES;                 /* '<S5>/SENSOR_OUTPUT_Processing' */
  real_T RW_F;                         /* '<S5>/SENSOR_OUTPUT_Processing' */
  real_T RW_B;                         /* '<S5>/SENSOR_OUTPUT_Processing' */
  real_T LW_F;                         /* '<S5>/SENSOR_OUTPUT_Processing' */
  real_T LW_B;                         /* '<S5>/SENSOR_OUTPUT_Processing' */
  real_T maxV;
  real_T maxV_m;
  real_T maxV_c;
  real_T maxV_k;
  boolean_T LED0;                      /* '<S5>/SENSOR_OUTPUT_Processing' */
  boolean_T LED2;                      /* '<S5>/SENSOR_OUTPUT_Processing' */
  boolean_T LED1;                      /* '<S5>/SENSOR_OUTPUT_Processing' */
  B_MATLABSystem1_MicroMouseTem_T MATLABSystem6;/* '<S48>/MATLAB System1' */
  B_MATLABSystem1_MicroMouseTem_T MATLABSystem5;/* '<S48>/MATLAB System1' */
  B_MATLABSystem1_MicroMouseTem_T MATLABSystem4_c;/* '<S48>/MATLAB System1' */
  B_MATLABSystem1_MicroMouseTem_T MATLABSystem3_c;/* '<S48>/MATLAB System1' */
  B_MATLABSystem1_MicroMouseTem_T MATLABSystem2_ci;/* '<S48>/MATLAB System1' */
  B_MATLABSystem1_MicroMouseTem_T MATLABSystem1_ci;/* '<S48>/MATLAB System1' */
  B_MATLABSystem1_MicroMouseTem_T MATLABSystem2_c;/* '<S48>/MATLAB System1' */
  B_MATLABSystem1_MicroMouseTem_T MATLABSystem1_c;/* '<S48>/MATLAB System1' */
  B_MATLABSystem3_MicroMouseTem_T MATLABSystem4;/* '<S48>/MATLAB System3' */
  B_MATLABSystem3_MicroMouseTem_T MATLABSystem3;/* '<S48>/MATLAB System3' */
  B_MATLABSystem1_MicroMouseTem_T MATLABSystem2;/* '<S48>/MATLAB System1' */
  B_MATLABSystem1_MicroMouseTem_T MATLABSystem1;/* '<S48>/MATLAB System1' */
} B_MicroMouseTemplate_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  stm32cube_blocks_AnalogInput__T obj; /* '<S47>/Analog to Digital Converter' */
  stm32cube_blocks_PWMOutput_Mi_T obj_g;/* '<S42>/PWM Output' */
  stm32cube_blocks_PWMOutput_Mi_T obj_n;/* '<S40>/PWM Output' */
  real_T RS_Threshold;                 /* '<S5>/SENSOR_OUTPUT_Processing' */
  real_T LS_Threshold;                 /* '<S5>/SENSOR_OUTPUT_Processing' */
  real_T FWD_Threshold;                /* '<S5>/SENSOR_OUTPUT_Processing' */
  real_T GOAL_L;                       /* '<S5>/SENSOR_OUTPUT_Processing' */
  real_T GOAL_R;                       /* '<S5>/SENSOR_OUTPUT_Processing' */
  struct {
    uint_T is_ON_MODE:5;               /* '<S5>/SENSOR_OUTPUT_Processing' */
    uint_T is_c3_MicroMouseTemplate:2; /* '<S5>/SENSOR_OUTPUT_Processing' */
    uint_T is_active_c3_MicroMouseTemplate:1;/* '<S5>/SENSOR_OUTPUT_Processing' */
    uint_T RIGHT_Wall:1;               /* '<S5>/SENSOR_OUTPUT_Processing' */
    uint_T LEFT_Wall:1;                /* '<S5>/SENSOR_OUTPUT_Processing' */
    uint_T FWD_Wall:1;                 /* '<S5>/SENSOR_OUTPUT_Processing' */
    uint_T FINISH:1;                   /* '<S5>/SENSOR_OUTPUT_Processing' */
  } bitsForTID0;

  uint8_T temporalCounter_i1;          /* '<S5>/SENSOR_OUTPUT_Processing' */
  DW_MATLABSystem1_MicroMouseTe_T MATLABSystem6;/* '<S48>/MATLAB System1' */
  DW_MATLABSystem1_MicroMouseTe_T MATLABSystem5;/* '<S48>/MATLAB System1' */
  DW_MATLABSystem1_MicroMouseTe_T MATLABSystem4_c;/* '<S48>/MATLAB System1' */
  DW_MATLABSystem1_MicroMouseTe_T MATLABSystem3_c;/* '<S48>/MATLAB System1' */
  DW_MATLABSystem1_MicroMouseTe_T MATLABSystem2_ci;/* '<S48>/MATLAB System1' */
  DW_MATLABSystem1_MicroMouseTe_T MATLABSystem1_ci;/* '<S48>/MATLAB System1' */
  DW_MATLABSystem1_MicroMouseTe_T MATLABSystem2_c;/* '<S48>/MATLAB System1' */
  DW_MATLABSystem1_MicroMouseTe_T MATLABSystem1_c;/* '<S48>/MATLAB System1' */
  DW_MATLABSystem3_MicroMouseTe_T MATLABSystem4;/* '<S48>/MATLAB System3' */
  DW_MATLABSystem3_MicroMouseTe_T MATLABSystem3;/* '<S48>/MATLAB System3' */
  DW_MATLABSystem1_MicroMouseTe_T MATLABSystem2;/* '<S48>/MATLAB System1' */
  DW_MATLABSystem1_MicroMouseTe_T MATLABSystem1;/* '<S48>/MATLAB System1' */
} DW_MicroMouseTemplate_T;

/* Parameters for system: '<S48>/MATLAB System1' */
struct P_MATLABSystem1_MicroMouseTem_T_ {
  real_T MATLABSystem1_InitialCondition;/* Expression: 0
                                         * Referenced by: '<S48>/MATLAB System1'
                                         */
};

/* Parameters for system: '<S48>/MATLAB System3' */
struct P_MATLABSystem3_MicroMouseTem_T_ {
  real_T MATLABSystem3_InitialCondition;/* Expression: 0
                                         * Referenced by: '<S48>/MATLAB System3'
                                         */
};

/* Parameters (default storage) */
struct P_MicroMouseTemplate_T_ {
  real_T Constant_Value;               /* Expression: 1
                                        * Referenced by: '<S3>/Constant'
                                        */
  real_T Quantizer_Interval;           /* Expression: 0.01
                                        * Referenced by: '<Root>/Quantizer'
                                        */
  real_T Gain_Gain;                    /* Expression: 0.5
                                        * Referenced by: '<S5>/Gain'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S3>/Constant1'
                                        */
  real_T DOWN_RSANDDOWN_LS_Threshold;  /* Expression: 0
                                        * Referenced by: '<S3>/DOWN_RS AND DOWN_LS'
                                        */
  real_T FWD_RSANDFWD_LS_Threshold;    /* Expression: 0
                                        * Referenced by: '<S3>/FWD_RS AND FWD_LS'
                                        */
  real_T RSANDLS_Threshold;            /* Expression: 0
                                        * Referenced by: '<S3>/RS AND LS'
                                        */
  real_T Constant_Value_l;             /* Expression: 1
                                        * Referenced by: '<S4>/Constant'
                                        */
  real_T Constant_Value_i;             /* Expression: 1
                                        * Referenced by: '<S8>/Constant'
                                        */
  int32_T DataStoreMemory2_InitialValue;
                            /* Computed Parameter: DataStoreMemory2_InitialValue
                             * Referenced by: '<S1>/Data Store Memory2'
                             */
  int32_T DataStoreMemory4_InitialValue;
                            /* Computed Parameter: DataStoreMemory4_InitialValue
                             * Referenced by: '<S1>/Data Store Memory4'
                             */
  real32_T DataStoreMemory_InitialValue;
                             /* Computed Parameter: DataStoreMemory_InitialValue
                              * Referenced by: '<S7>/Data Store Memory'
                              */
  real32_T DataStoreMemory1_InitialValue;
                            /* Computed Parameter: DataStoreMemory1_InitialValue
                             * Referenced by: '<S7>/Data Store Memory1'
                             */
  uint16_T Gain_Gain_o;                /* Computed Parameter: Gain_Gain_o
                                        * Referenced by: '<Root>/Gain'
                                        */
  uint16_T Constant_Value_b;           /* Computed Parameter: Constant_Value_b
                                        * Referenced by: '<S45>/Constant'
                                        */
  uint16_T Constant1_Value_c;          /* Computed Parameter: Constant1_Value_c
                                        * Referenced by: '<S45>/Constant1'
                                        */
  uint16_T DataStoreMemory_InitialValue_p;
                           /* Computed Parameter: DataStoreMemory_InitialValue_p
                            * Referenced by: '<S6>/Data Store Memory'
                            */
  uint16_T DataStoreMemory1_InitialValue_p;
                          /* Computed Parameter: DataStoreMemory1_InitialValue_p
                           * Referenced by: '<S6>/Data Store Memory1'
                           */
  uint16_T DataStoreMemory2_InitialValue_p;
                          /* Computed Parameter: DataStoreMemory2_InitialValue_p
                           * Referenced by: '<S6>/Data Store Memory2'
                           */
  uint16_T DataStoreMemory1_InitialValue_j;
                          /* Computed Parameter: DataStoreMemory1_InitialValue_j
                           * Referenced by: '<S1>/Data Store Memory1'
                           */
  boolean_T DataStoreMemory_InitialValue_p4;
                          /* Computed Parameter: DataStoreMemory_InitialValue_p4
                           * Referenced by: '<S1>/Data Store Memory'
                           */
  P_MATLABSystem1_MicroMouseTem_T MATLABSystem6;/* '<S48>/MATLAB System1' */
  P_MATLABSystem1_MicroMouseTem_T MATLABSystem5;/* '<S48>/MATLAB System1' */
  P_MATLABSystem1_MicroMouseTem_T MATLABSystem4_c;/* '<S48>/MATLAB System1' */
  P_MATLABSystem1_MicroMouseTem_T MATLABSystem3_c;/* '<S48>/MATLAB System1' */
  P_MATLABSystem1_MicroMouseTem_T MATLABSystem2_ci;/* '<S48>/MATLAB System1' */
  P_MATLABSystem1_MicroMouseTem_T MATLABSystem1_ci;/* '<S48>/MATLAB System1' */
  P_MATLABSystem1_MicroMouseTem_T MATLABSystem2_c;/* '<S48>/MATLAB System1' */
  P_MATLABSystem1_MicroMouseTem_T MATLABSystem1_c;/* '<S48>/MATLAB System1' */
  P_MATLABSystem3_MicroMouseTem_T MATLABSystem4;/* '<S48>/MATLAB System3' */
  P_MATLABSystem3_MicroMouseTem_T MATLABSystem3;/* '<S48>/MATLAB System3' */
  P_MATLABSystem1_MicroMouseTem_T MATLABSystem2;/* '<S48>/MATLAB System1' */
  P_MATLABSystem1_MicroMouseTem_T MATLABSystem1;/* '<S48>/MATLAB System1' */
};

/* Code_Instrumentation_Declarations_Placeholder */

/* Real-time Model Data Structure */
struct tag_RTM_MicroMouseTemplate_T {
  const char_T * volatile errorStatus;
};

/* Block parameters (default storage) */
extern P_MicroMouseTemplate_T MicroMouseTemplate_P;

/* Block signals (default storage) */
extern B_MicroMouseTemplate_T MicroMouseTemplate_B;

/* Block states (default storage) */
extern DW_MicroMouseTemplate_T MicroMouseTemplate_DW;

/*
 * Exported States
 *
 * Note: Exported states are block states with an exported global
 * storage class designation.  Code generation will declare the memory for these
 * states and exports their symbols.
 *
 */
extern real32_T IMU_Accel[3];          /* '<S7>/Data Store Memory' */
extern real32_T IMU_Gyro[3];           /* '<S7>/Data Store Memory1' */
extern int32_T currTicksRS;            /* '<S1>/Data Store Memory2' */
extern int32_T currTicksLS;            /* '<S1>/Data Store Memory4' */
extern uint16_T ADC1s[9];              /* '<S6>/Data Store Memory' */
extern uint16_T ADC_H[9];              /* '<S6>/Data Store Memory1' */
extern uint16_T ADC_L[9];              /* '<S6>/Data Store Memory2' */
extern uint16_T Thresholds[8];         /* '<S1>/Data Store Memory1' */
extern boolean_T Detections[8];        /* '<S1>/Data Store Memory' */

/* Model entry point functions */
extern void MicroMouseTemplate_initialize(void);
extern void MicroMouseTemplate_step(void);
extern void MicroMouseTemplate_terminate(void);

/* Real-time Model object */
extern RT_MODEL_MicroMouseTemplate_T *const MicroMouseTemplate_M;
extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Cast To Double1' : Unused code path elimination
 * Block '<S45>/Cast1' : Eliminate redundant data type conversion
 * Block '<S45>/Cast3' : Eliminate redundant data type conversion
 * Block '<S50>/Rate Transition' : Eliminated since input and output rates are identical
 * Block '<S50>/Rate Transition1' : Eliminated since input and output rates are identical
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'MicroMouseTemplate'
 * '<S1>'   : 'MicroMouseTemplate/Detections and Thresholds'
 * '<S2>'   : 'MicroMouseTemplate/GPIO for IR LEDs'
 * '<S3>'   : 'MicroMouseTemplate/IR LED Pattern'
 * '<S4>'   : 'MicroMouseTemplate/Motors'
 * '<S5>'   : 'MicroMouseTemplate/Sensors'
 * '<S6>'   : 'MicroMouseTemplate/Subsystem'
 * '<S7>'   : 'MicroMouseTemplate/Subsystem1'
 * '<S8>'   : 'MicroMouseTemplate/Subsystem2'
 * '<S9>'   : 'MicroMouseTemplate/Subsystem3'
 * '<S10>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_DOWN_LEFT'
 * '<S11>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_DOWN_RIGHT'
 * '<S12>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_FWD_LEFT'
 * '<S13>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_FWD_RIGHT'
 * '<S14>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_LEFT'
 * '<S15>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_MOT_LEFT'
 * '<S16>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_MOT_RIGHT'
 * '<S17>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_RIGHT'
 * '<S18>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_DOWN_LEFT/ECSoC'
 * '<S19>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_DOWN_LEFT/ECSoC/ECSimCodegen'
 * '<S20>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_DOWN_RIGHT/ECSoC'
 * '<S21>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_DOWN_RIGHT/ECSoC/ECSimCodegen'
 * '<S22>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_FWD_LEFT/ECSoC'
 * '<S23>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_FWD_LEFT/ECSoC/ECSimCodegen'
 * '<S24>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_FWD_RIGHT/ECSoC'
 * '<S25>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_FWD_RIGHT/ECSoC/ECSimCodegen'
 * '<S26>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_LEFT/ECSoC'
 * '<S27>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_LEFT/ECSoC/ECSimCodegen'
 * '<S28>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_MOT_LEFT/ECSoC'
 * '<S29>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_MOT_LEFT/ECSoC/ECSimCodegen'
 * '<S30>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_MOT_RIGHT/ECSoC'
 * '<S31>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_MOT_RIGHT/ECSoC/ECSimCodegen'
 * '<S32>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_RIGHT/ECSoC'
 * '<S33>'  : 'MicroMouseTemplate/GPIO for IR LEDs/LED_RIGHT/ECSoC/ECSimCodegen'
 * '<S34>'  : 'MicroMouseTemplate/Motors/MOTOR_EN'
 * '<S35>'  : 'MicroMouseTemplate/Motors/PWM Output'
 * '<S36>'  : 'MicroMouseTemplate/Motors/PWM Output1'
 * '<S37>'  : 'MicroMouseTemplate/Motors/MOTOR_EN/ECSoC'
 * '<S38>'  : 'MicroMouseTemplate/Motors/MOTOR_EN/ECSoC/ECSimCodegen'
 * '<S39>'  : 'MicroMouseTemplate/Motors/PWM Output/ECSoC'
 * '<S40>'  : 'MicroMouseTemplate/Motors/PWM Output/ECSoC/ECSimCodegen'
 * '<S41>'  : 'MicroMouseTemplate/Motors/PWM Output1/ECSoC'
 * '<S42>'  : 'MicroMouseTemplate/Motors/PWM Output1/ECSoC/ECSimCodegen'
 * '<S43>'  : 'MicroMouseTemplate/Sensors/SENSOR_OUTPUT_Processing'
 * '<S44>'  : 'MicroMouseTemplate/Subsystem/ADC2 IN10  IN1'
 * '<S45>'  : 'MicroMouseTemplate/Subsystem/get Highs and Lows'
 * '<S46>'  : 'MicroMouseTemplate/Subsystem/ADC2 IN10  IN1/ECSoC'
 * '<S47>'  : 'MicroMouseTemplate/Subsystem/ADC2 IN10  IN1/ECSoC/ECSimCodegen'
 * '<S48>'  : 'MicroMouseTemplate/Subsystem/get Highs and Lows/Subsystem'
 * '<S49>'  : 'MicroMouseTemplate/Subsystem/get Highs and Lows/Subsystem1'
 * '<S50>'  : 'MicroMouseTemplate/Subsystem/get Highs and Lows/Subsystem2'
 * '<S51>'  : 'MicroMouseTemplate/Subsystem2/CTRL_LEDs'
 * '<S52>'  : 'MicroMouseTemplate/Subsystem2/LED_0'
 * '<S53>'  : 'MicroMouseTemplate/Subsystem2/LED_1'
 * '<S54>'  : 'MicroMouseTemplate/Subsystem2/LED_2'
 * '<S55>'  : 'MicroMouseTemplate/Subsystem2/CTRL_LEDs/ECSoC'
 * '<S56>'  : 'MicroMouseTemplate/Subsystem2/CTRL_LEDs/ECSoC/ECSimCodegen'
 * '<S57>'  : 'MicroMouseTemplate/Subsystem2/LED_0/ECSoC'
 * '<S58>'  : 'MicroMouseTemplate/Subsystem2/LED_0/ECSoC/ECSimCodegen'
 * '<S59>'  : 'MicroMouseTemplate/Subsystem2/LED_1/ECSoC'
 * '<S60>'  : 'MicroMouseTemplate/Subsystem2/LED_1/ECSoC/ECSimCodegen'
 * '<S61>'  : 'MicroMouseTemplate/Subsystem2/LED_2/ECSoC'
 * '<S62>'  : 'MicroMouseTemplate/Subsystem2/LED_2/ECSoC/ECSimCodegen'
 * '<S63>'  : 'MicroMouseTemplate/Subsystem3/SW_1'
 * '<S64>'  : 'MicroMouseTemplate/Subsystem3/SW_2'
 * '<S65>'  : 'MicroMouseTemplate/Subsystem3/SW_1/ECSoC'
 * '<S66>'  : 'MicroMouseTemplate/Subsystem3/SW_1/ECSoC/ECSimCodegen'
 * '<S67>'  : 'MicroMouseTemplate/Subsystem3/SW_2/ECSoC'
 * '<S68>'  : 'MicroMouseTemplate/Subsystem3/SW_2/ECSoC/ECSimCodegen'
 */
#endif                                 /* MicroMouseTemplate_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
