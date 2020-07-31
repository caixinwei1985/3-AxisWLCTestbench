#ifndef MOTO_H
#define MOTO_H
#include <stdint.h>
#include "main.h"

#define MOTO_POSITION_OVERNEAR  0x00
#define MOTO_POSITION_NEAR      0x01
#define MOTO_POSITION_ZERONEAR  0x02
#define MOTO_POSITION_ZERO      0x03
#define MOTO_POSITION_ZEROFAR   0x04
#define MOTO_POSITION_FAR       0x05
#define MOTO_POSITION_OVERFAR   0x06

#define MOTO_STATUS_INHISR      0x20      // 禁止步进刷新中断
#define MOTO_STATUS_POSITIVE    0x40      // 电机正向运动
#define MOTO_STATUS_RUN         0x80      // 电机运动中  


typedef enum {AxisX = 0,AxisY = 1, AxisZ = 2,AxisUnknown = 0xff} Axis_t;

/**
  * A structure storges related to moto movement control
  */
typedef struct {
  uint8_t  stage;               // For the whole process is divided into 3 stages, acceleration(0),uniform(1) and deceleration(2)
  uint8_t  is_steps_aligned;    // A flag to indicate timer repeatition interrupt is if aligned to either ALIGNMENT_STEPS or 1
  uint8_t  status_flags;        // A bit-map flags,reference to MOTO_STATUS_XXXX macro definition
  uint8_t  position;            // A value to indicate approximate position moto is,reference to MOTO_POSITION_XXX macro definition
  uint32_t steps[3];            // Store steps needed to run at every stage
  uint32_t curr_speed;          // Store current driver pulse frequency,unit in pulse/second
  uint32_t acclerate;
} MotoCtrl_t;

typedef struct{
  uint8_t  axis;
  uint8_t  acc;
  uint16_t speed;
  int32_t  steps;
}MotoMoveDef_t;

void MOTO_Enable(void);
void MOTO_Disable(void);
void MOTO_Select(Axis_t axis);
void MOTO_Move(Axis_t axis,int32_t steps,uint32_t speed);
void MOTO_Config(uint8_t acc,uint8_t dec, uint16_t startspeed);
void MOTO_Stop(Axis_t axis);
void MOTO_Reset(Axis_t axis, uint16_t speed);
void MOTO_OnZeroPointEnter(Axis_t axis);
void MOTO_OnNearPointEnter(Axis_t axis);
void MOTO_OnFarPointEnter(Axis_t axis);
void MOTO_OnZeroPointExit(Axis_t axis);
uint32_t MOTO_OnNearPointExit(Axis_t axis);
uint32_t MOTO_OnFarPointExit(Axis_t axis);
void MOTO_IsAtZeroPoint(Axis_t axis);
void MOTO_IsStop(void);
uint32_t MOTO_GetDirection(Axis_t axis);
uint32_t MOTO_ISRHandler(Axis_t axis);
Axis_t MOTO_GetRunningAxis(void);
void MOTO_EmergentBreak(Axis_t axis);
void MOTO_Init(void);
void MOTO_Sync(void);
uint32_t MOTO_GetRemaindSteps(Axis_t axis);
uint8_t MOTO_GetPosition(Axis_t axis);
#endif