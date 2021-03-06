#include "moto.h"
#include "main.h"

#define ALIGNMENT_STEPS (32)
#define TIM_CLOCK (12000000L)
#define  EMERGENTSTOPSPEED (4000)
#define  EMERGENTSTOPSTEPS  (1200)

uint8_t   GetMotionDirection(void);
static MotoCtrl_t ctrlinfo[3];
static GPIO_TypeDef* dirPort[3];
static TIM_TypeDef* tim[3];
static uint32_t dirPin[3] ;
static uint32_t motoTimCh[3] ;
static uint32_t startSpeed;
static uint16_t emergentStopSteps;
static uint8_t acceleration;
static uint8_t deceleration;
static uint8_t sync_flag;

void MOTO_Sync(void)
{
  sync_flag = 1;
}


void MOTO_Init(void)
{
  uint32_t i;
  tim[0] = TIM1;
  dirPin[0] = MOTOXDIR_Pin;
  dirPort[0] = MOTOXDIR_GPIO_Port; 
  motoTimCh[0] = LL_TIM_CHANNEL_CH4;
  tim[1] = TIM3;
  dirPin[1] = MOTOYDIR_Pin;
  dirPort[1] = MOTOYDIR_GPIO_Port;
  motoTimCh[1] = LL_TIM_CHANNEL_CH4;
  tim[2] = TIM1;
  dirPin[2] = MOTOZDIR_Pin;
  dirPort[2] = MOTOZDIR_GPIO_Port;
  motoTimCh[2] = LL_TIM_CHANNEL_CH2;
  ctrlinfo[0].position = RTC->BKP0R;
  ctrlinfo[1].position = RTC->BKP1R;
  ctrlinfo[2].position = RTC->BKP2R;
  for(i=0;i<3;i++)
  {
    ctrlinfo[i].status_flags = 0;
  }
  startSpeed = 5000;
  emergentStopSteps = 1200;
  acceleration = 8;
  deceleration = 16;
  sync_flag = 0;
  MOTO_Enable();
  
}

void MOTO_Config(uint8_t acc,uint8_t dec,uint16_t startspeed)
{
  acceleration = acc;
  deceleration = dec;
  if(startspeed < 200)
    startspeed = 200;
  startSpeed = startspeed;
}
__weak void MOTO_Positive(Axis_t axis)
{
  LL_GPIO_ResetOutputPin(dirPort[axis],dirPin[axis]);
  ctrlinfo[axis].status_flags |= MOTO_STATUS_POSITIVE;
}

__weak void MOTO_Negitvie(Axis_t axis)
{
  LL_GPIO_SetOutputPin(dirPort[axis],dirPin[axis]);
  ctrlinfo[axis].status_flags &= ~MOTO_STATUS_POSITIVE;
}

__weak void MOTO_Enable(void)
{
  // 
  LL_GPIO_SetOutputPin(MOTOEN_GPIO_Port,MOTOEN_Pin);
}

__weak void MOTO_Disable(void)
{
  LL_GPIO_ResetOutputPin(MOTOEN_GPIO_Port,MOTOEN_Pin);
}

__weak void MOTO_Start(uint8_t ch)
{
//  // 关闭所有PWM通道
//  LL_TIM_CC_DisableChannel(tim[axis],0xffffffff);
  // 启动指定轴向的PWM通道
  LL_TIM_CC_EnableChannel(tim[ch],motoTimCh[ch]);
  // 标记当前轴向的状态为 RUN
  ctrlinfo[ch].status_flags |= MOTO_STATUS_RUN;  
    // 最低速度启动
  tim[ch]->ARR = TIM_CLOCK / ctrlinfo[ch].curr_speed;
  switch(ch)
  {
    case 0:
      TIM1->CCR4=TIM1->ARR/2;
      break;
    case 1:
      TIM3->CCR4=TIM3->ARR/2;
      break;
    case 2:
      TIM1->CCR2=TIM1->ARR/2;
      break;
    default:
      break;
  }
  // 使能PWM信号的外部管脚输出
  tim[ch]->BDTR |= TIM_BDTR_MOE;
  tim[ch]->CNT = 0;
  tim[ch]->CR1 |= TIM_CR1_CEN;
  tim[ch]->RCR = ALIGNMENT_STEPS-1;
  tim[ch]->DIER = TIM_DIER_UIE;

}

void MOTO_Set(Axis_t axis,int32_t steps,int32_t speed)
{
  int32_t total_speed_delta = speed - startSpeed;
  if(total_speed_delta < 0)
    total_speed_delta = 0;
  //计算起步加速过程需要的步数
  uint32_t acclerate_steps = total_speed_delta/acceleration;
  if(0 == steps)
    return;
  // 设置运动方向
  if(steps>0)
    MOTO_Positive(axis);
  if(steps<0)
  {
    steps ^= 0xffffffff;
    steps += 1;
    MOTO_Negitvie(axis);
  }
  steps |= 1;
  //起步步数向上对齐到ALIGMENT_STEPS
//  if(acclerate_steps > 0)
  {
    acclerate_steps -= acclerate_steps % ALIGNMENT_STEPS;
    acclerate_steps += ALIGNMENT_STEPS;
  }
  if(steps > acclerate_steps * 2)
  {
    ctrlinfo[axis].steps[0] = acclerate_steps;
    ctrlinfo[axis].steps[1] = steps - acclerate_steps*2;
    ctrlinfo[axis].steps[1] += ALIGNMENT_STEPS - ctrlinfo[axis].steps[1] % ALIGNMENT_STEPS;
    ctrlinfo[axis].steps[2] = steps - ctrlinfo[axis].steps[0] - ctrlinfo[axis].steps[1];
  }
  else
  {
    ctrlinfo[axis].steps[0] = steps/2;
    ctrlinfo[axis].steps[0] += ALIGNMENT_STEPS - ctrlinfo[axis].steps[0] % ALIGNMENT_STEPS;
    ctrlinfo[axis].steps[2] = steps- ctrlinfo[axis].steps[0];
    ctrlinfo[axis].steps[1] = 0;
  }

  ctrlinfo[axis].stage = 0;
  ctrlinfo[axis].curr_speed = startSpeed;
  ctrlinfo[axis].is_steps_aligned = 1;
  ctrlinfo[axis].acclerate = acceleration;
}

__weak void MOTO_Stop(Axis_t axis)
{
  ctrlinfo[axis].status_flags &= ~MOTO_STATUS_RUN;
  LL_TIM_CC_DisableChannel(tim[axis],motoTimCh[axis]);
  LL_TIM_DisableCounter(tim[axis]);
}

void MOTO_Reset(Axis_t axis,uint16_t speed)
{
  MotoCtrl_t* pmc = &ctrlinfo[axis];
  switch(pmc->position)
  {
    case MOTO_POSITION_FAR:
    case MOTO_POSITION_OVERFAR:
    case MOTO_POSITION_ZEROFAR:
      MOTO_Set(axis,0x80000000,speed);
      break;
    case MOTO_POSITION_NEAR:
    case MOTO_POSITION_OVERNEAR:
    case MOTO_POSITION_ZERONEAR:
      MOTO_Set(axis,0x7fffffff,speed);
  }
}

///@brife  在对应的PWM周期溢出终端中调用
///@param  对应的轴向
///@retval 返回当前的转速，0则表示进步完成
uint32_t MOTO_ISRHandler(Axis_t axis)
{
  MotoCtrl_t *pmc = &ctrlinfo[axis];
  uint8_t stage = pmc->stage;
  if(pmc->status_flags&MOTO_STATUS_INHISR)
    return pmc->curr_speed;
#if 1
  if(pmc->is_steps_aligned)
  {
    pmc->steps[stage] -= ALIGNMENT_STEPS;
    //当剩余的步数不足ALIGNMENT_STEPS时，不再ALIGNMENT_STEPS个脉冲中断一次，
    //而是1个脉冲中断1次
    if(pmc->steps[stage]<ALIGNMENT_STEPS)
    {
      pmc->is_steps_aligned = 0;
      //
      LL_TIM_SetRepetitionCounter(tim[axis],0);
    }
    //加速阶段
    if(0 == pmc->stage)
      pmc->curr_speed += pmc->acclerate * ALIGNMENT_STEPS;
    //减速阶段
    if(2 == pmc->stage)
      pmc->curr_speed -= pmc->acclerate * ALIGNMENT_STEPS;
  }
  else
    pmc->steps[stage]--;
#else 
  if(pmc->is_steps_aligned)
  {
    pmc->steps[stage] -= ALIGNMENT_STEPS;
    //当剩余的步数不足ALIGNMENT_STEPS时，不再ALIGNMENT_STEPS个脉冲中断一次，
    //而是1个脉冲中断1次
    if(pmc->steps[2]<ALIGNMENT_STEPS)
    {
      pmc->is_steps_aligned = 0;
      //
      LL_TIM_SetRepetitionCounter(TIM1,0);
    }
  }
  if(sync_flag)
  {
    sync_flag = 0;
    //加速阶段
    if(0 == pmc->stage)
      pmc->curr_speed += pmc->acclerate;
    //减速阶段
    if(2 == pmc->stage)
      pmc->curr_speed -= pmc->acclerate;
  }
#endif
  while(0 == pmc->steps[stage])
  {
    stage++;
    if(stage > 2)
      break;
  }
  pmc->stage = stage;
  if(stage > 2)
    return 0;
  else
    return pmc->curr_speed;
  
}
/*
*/

static void MOTO_BakupPosition(Axis_t axis,uint8_t position)
{
  switch(axis)
  {
    case AxisX:RTC->BKP0R &= ~0x00ff;RTC->BKP0R |= position;break;
    case AxisY:RTC->BKP1R &= ~0x00ff;RTC->BKP1R |= position;break;
    case AxisZ:RTC->BKP2R &= ~0x00ff;RTC->BKP2R |= position;break;
    default:break;
  }
}
/*
 * 进入到零点位置
*/
void MOTO_OnZeroPointEnter(Axis_t axis)
{
  MotoCtrl_t* pmc = & ctrlinfo[axis];
  pmc->position = MOTO_POSITION_ZERO; 
  MOTO_BakupPosition(axis,pmc->position);
}

/*
 * 离开零点位置
*/
void MOTO_OnZeroPointExit(Axis_t axis)
{
  MotoCtrl_t* pmc = &ctrlinfo[axis];
  if(GetMotionDirection()){
    pmc->position = MOTO_POSITION_ZEROFAR;
  }
  else{
    pmc->position = MOTO_POSITION_ZERONEAR;
  }
  MOTO_BakupPosition(axis,pmc->position);
}

/*
 * 进入到近点位置
*/
void MOTO_OnNearPointEnter(Axis_t axis)
{
  ctrlinfo[axis].position = MOTO_POSITION_NEAR;
  MOTO_BakupPosition(axis,MOTO_POSITION_NEAR);
}

/*
 * 离开近点位置
*/
///@retval 0:exit near point normally;1:over near point
uint32_t MOTO_OnNearPointExit(Axis_t axis)
{
  MotoCtrl_t* pmc = &ctrlinfo[axis];
  uint32_t rs = 0;
  if(GetMotionDirection()){
    pmc->position = MOTO_POSITION_ZERONEAR;       //电机正向运行，在零点近点之间
  }
  else{
    pmc->position = MOTO_POSITION_OVERNEAR;       //电机反向运行，越过了近点
    rs = 1;
    // MOTO_OverNearHook();
  }
  MOTO_BakupPosition(axis,pmc->position);
  return rs;
}

/*
 * 进入远点位置
*/
void MOTO_OnFarPointEnter(Axis_t axis)
{
  ctrlinfo[axis].position = MOTO_POSITION_FAR;
  MOTO_BakupPosition(axis,MOTO_POSITION_FAR);
}

/*
 * 离开远点位置
*/
///@retval 0:exit far point normally;1:over far point
uint32_t MOTO_OnFarPointExit(Axis_t axis)
{
  MotoCtrl_t* pmc = &ctrlinfo[axis];
  uint32_t rs = 0;
  if(GetMotionDirection()){
    pmc->position = MOTO_POSITION_OVERFAR;   // 如果电机运行为正向，越过了远点
    rs = 1;
  }
  else{
    pmc->position = MOTO_POSITION_ZEROFAR;   // 电机运行为反向，在零点远点之间
    // MOTO_OverFarHook();
  }
  MOTO_BakupPosition(axis,pmc->position);
  return rs;
}
void MOTO_SetStatus(uint32_t status)
{
  
}
Axis_t MOTO_GetRunningAxis()
{
  Axis_t axis;
  for(axis = AxisX;axis<=AxisZ;axis++)
  {
    if(ctrlinfo[axis].status_flags&MOTO_STATUS_RUN)
      return axis;
  }
  return AxisUnknown;
}
void MOTO_SetPosition(Axis_t axis, uint32_t pos)
{
  ctrlinfo[axis].position = pos;
}
uint8_t MOTO_GetPosition(Axis_t axis)
{
  return ctrlinfo[axis].position;
}

///@retval  1:positive;2:negitive
uint32_t MOTO_GetDirection(Axis_t axis)
{
  return ctrlinfo[axis].status_flags & MOTO_STATUS_POSITIVE;
}
///@brife  触发紧急制动时，使用该方法配置制动过程
void MOTO_EmergentBreak(Axis_t axis)
{
  MotoCtrl_t* pmc = &ctrlinfo[axis];
  
  //禁用步数刷新处理
  pmc->status_flags |= MOTO_STATUS_INHISR; 
  pmc->stage = 2;
  pmc->steps[0] = 0;
  pmc->steps[1] = 0;
  if(pmc->curr_speed < startSpeed)
  {
    pmc->steps[2] = 0;
  }
  else
  {
    uint32_t speed_delta = pmc->curr_speed - startSpeed;
    pmc->steps[2]= speed_delta / deceleration ;
  }
  //启用
  pmc->status_flags &= ~MOTO_STATUS_INHISR;
  
}

uint32_t MOTO_GetRemaindSteps(Axis_t axis)
{
  return ctrlinfo[axis].steps[0]+ctrlinfo[axis].steps[2]+ctrlinfo[axis].steps[1];
}