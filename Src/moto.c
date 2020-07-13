#include "moto.h"
#include "main.h"

#define ALIGNMENT_STEPS (32)
#define TIM_CLOCK (48000000L)
#define  EMERGENTSTOPSPEED (10000)
#define  EMERGENTSTOPSTEPS  (5000)
static MotoCtrl_t ctrlinfo[3];
static GPIO_TypeDef* dirPort[3];
static uint32_t dirPin[3] ;
static uint32_t axisCh[3] ;

void MOTO_Init(void)
{
  uint32_t i;
  dirPin[0] = MOTOXDIR_Pin;
  dirPort[0] = MOTOXDIR_GPIO_Port; 
  axisCh[0] = LL_TIM_CHANNEL_CH4;
  dirPin[1] = MOTOYDIR_Pin;
  dirPort[1] = MOTOYDIR_GPIO_Port;
  axisCh[1] = LL_TIM_CHANNEL_CH3;
  dirPin[2] = MOTOZDIR_Pin;
  dirPort[2] = MOTOZDIR_GPIO_Port;
  axisCh[2] = LL_TIM_CHANNEL_CH2;
  ctrlinfo[0].position = RTC->BKP0R;
  ctrlinfo[1].position = RTC->BKP1R;
  ctrlinfo[2].position = RTC->BKP2R;
  for(i=0;i<3;i++)
  {
    ctrlinfo[i].status_flags = 0;
  }
  
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
  LL_GPIO_ResetOutputPin(MOTOEN_GPIO_Port,MOTOEN_Pin);
}

__weak void MOTO_Disable(void)
{
  LL_GPIO_SetOutputPin(MOTOEN_GPIO_Port,MOTOEN_Pin);
}

__weak void MOTO_Select(Axis_t axis)
{

}

__weak void MOTO_Start(Axis_t axis)
{
  // �ر�����PWMͨ��
  LL_TIM_CC_DisableChannel(TIM1,0xffffffff);
  // ����ָ�������PWMͨ��
  LL_TIM_CC_EnableChannel(TIM1,axisCh[axis]);
  // ��ǵ�ǰ�����״̬Ϊ RUN
  ctrlinfo[axis].status_flags |= MOTO_STATUS_RUN;  
    // ����ٶ�����
  TIM1->ARR = TIM_CLOCK / ctrlinfo[axis].curr_speed;
  switch(axis)
  {
    case AxisX:
      TIM1->CCR4=TIM1->ARR/2;
      break;
    case AxisY:
      TIM1->CCR3=TIM1->ARR/2;
      break;
    case AxisZ:
      TIM1->CCR2=TIM1->ARR/2;
      break;
    default:
      break;
  }
  TIM1->BDTR |= TIM_BDTR_MOE;
  TIM1->CNT = 0;
  TIM1->CR1 |= TIM_CR1_CEN;
  TIM1->DIER |= TIM_DIER_UIE;
  TIM1->RCR = ALIGNMENT_STEPS;
}

void MOTO_Move(Axis_t axis,int32_t steps,uint32_t speed,uint32_t acclerate)
{
  uint32_t lower_speed = 2500;
  uint32_t total_speed_delta = speed - lower_speed;
  //�����𲽼��ٹ�����Ҫ�Ĳ���
  uint32_t acclerate_steps = total_speed_delta/acclerate*ALIGNMENT_STEPS;
  if(0 == steps)
  return;
  // �����˶�����
  if(steps>0)
    MOTO_Positive(axis);
  if(steps<0)
  {
    steps ^= 0xffffffff;
    steps += 1;
    MOTO_Negitvie(axis);
  }
  steps |= 1;
  //�𲽲������϶��뵽ALIGMENT_STEPS
  acclerate_steps -= acclerate_steps % ALIGNMENT_STEPS;
  acclerate_steps += ALIGNMENT_STEPS;
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
  ctrlinfo[axis].curr_speed = lower_speed;
  ctrlinfo[axis].is_steps_aligned = 1;
  ctrlinfo[axis].acclerate = acclerate;

  MOTO_Start(axis);
}

__weak void MOTO_Stop(Axis_t axis)
{
  ctrlinfo[axis].status_flags &= ~MOTO_STATUS_RUN;
  LL_TIM_DisableCounter(TIM1);
}

void MOTO_Reset(Axis_t axis)
{
  MotoCtrl_t* pmc = &ctrlinfo[axis];

  switch(pmc->position)
  {
    case MOTO_POSITION_FAR:
    case MOTO_POSITION_OVERFAR:
    case MOTO_POSITION_ZEROFAR:
      MOTO_Move(axis,0x80000000,60000,5);
      break;
    case MOTO_POSITION_NEAR:
    case MOTO_POSITION_OVERNEAR:
    case MOTO_POSITION_ZERONEAR:
      MOTO_Move(axis,0x7fffffff,60000,5);
  }
}

///@brife  �ڶ�Ӧ��PWM��������ն��е���
///@param  ��Ӧ������
///@retval ���ص�ǰ��ת�٣�0���ʾ�������
uint32_t MOTO_ISRHandler(Axis_t axis)
{
  MotoCtrl_t *pmc = &ctrlinfo[axis];
  uint8_t stage = pmc->stage;
  if(pmc->status_flags&MOTO_STATUS_INHISR)
    return pmc->curr_speed;
  
  if(pmc->is_steps_aligned)
  {
    pmc->steps[stage] -= ALIGNMENT_STEPS;
    //��ʣ��Ĳ�������ALIGNMENT_STEPSʱ������ALIGNMENT_STEPS�������ж�һ�Σ�
    //����1�������ж�1��
    if(pmc->steps[2]<ALIGNMENT_STEPS)
    {
      pmc->is_steps_aligned = 0;
      //
      LL_TIM_SetRepetitionCounter(TIM1,0);
    }
    //���ٽ׶�
    if(0 == pmc->stage)
      pmc->curr_speed += pmc->acclerate;
    //���ٽ׶�
    if(2 == pmc->stage)
      pmc->curr_speed -= pmc->acclerate;
  }
  else
    pmc->steps[stage]--;
  
  if(0 == pmc->steps[stage])
    stage++;
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
 * ���뵽���λ��
*/
void MOTO_OnZeroPointEnter(Axis_t axis)
{
  MotoCtrl_t* pmc = & ctrlinfo[axis];
  pmc->position = MOTO_POSITION_ZERO; 
  MOTO_BakupPosition(axis,pmc->position);
}

/*
 * �뿪���λ��
*/
void MOTO_OnZeroPointExit(Axis_t axis)
{
  MotoCtrl_t* pmc = &ctrlinfo[axis];
  if(pmc->status_flags & MOTO_STATUS_POSITIVE){
    pmc->position = MOTO_POSITION_ZEROFAR;
  }
  else{
    pmc->position = MOTO_POSITION_ZERONEAR;
  }
  MOTO_BakupPosition(axis,pmc->position);
}

/*
 * ���뵽����λ��
*/
void MOTO_OnNearPointEnter(Axis_t axis)
{
  ctrlinfo[axis].position = MOTO_POSITION_NEAR;
  MOTO_BakupPosition(axis,MOTO_POSITION_NEAR);
}

/*
 * �뿪����λ��
*/
///@retval 0:exit near point normally;1:over near point
uint32_t MOTO_OnNearPointExit(Axis_t axis)
{
  MotoCtrl_t* pmc = &ctrlinfo[axis];
  uint32_t rs = 0;
  if(pmc->status_flags & MOTO_STATUS_POSITIVE){
    pmc->position = MOTO_POSITION_ZERONEAR;       //����������У���������֮��
  }
  else{
    pmc->position = MOTO_POSITION_OVERNEAR;       //����������У�Խ���˽���
    rs = 1;
    // MOTO_OverNearHook();
  }
  MOTO_BakupPosition(axis,pmc->position);
}

/*
 * ����Զ��λ��
*/
void MOTO_OnFarPointEnter(Axis_t axis)
{
  ctrlinfo[axis].position = MOTO_POSITION_FAR;
  MOTO_BakupPosition(axis,MOTO_POSITION_FAR);
}

/*
 * �뿪Զ��λ��
*/
///@retval 0:exit far point normally;1:over far point
uint32_t MOTO_OnFarPointExit(Axis_t axis)
{
  MotoCtrl_t* pmc = &ctrlinfo[axis];
  uint32_t rs = 0;
  if(pmc->status_flags & MOTO_STATUS_POSITIVE){
    pmc->position = MOTO_POSITION_OVERFAR;   // ����������Ϊ����Խ����Զ��
  }
  else{
    pmc->position = MOTO_POSITION_ZEROFAR;   // �������Ϊ���������Զ��֮��
    rs = 1;
    // MOTO_OverFarHook();
  }
  MOTO_BakupPosition(axis,pmc->position);
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
///@brife  ���������ƶ�ʱ��ʹ�ø÷��������ƶ�����
void MOTO_EmergentBreak(Axis_t axis)
{
  MotoCtrl_t* pmc = &ctrlinfo[axis];
  
  //���ò���ˢ�´���
  pmc->status_flags |= MOTO_STATUS_INHISR; 
  pmc->stage = 2;
  pmc->steps[2] = EMERGENTSTOPSTEPS;
  pmc->steps[0] = 0;
  pmc->steps[1] = 0;
  if(pmc->curr_speed < EMERGENTSTOPSPEED)
    pmc->acclerate = 0;
  else
  {
    uint32_t speed_delta = pmc->curr_speed - EMERGENTSTOPSPEED;
    pmc->acclerate = speed_delta * ALIGNMENT_STEPS / EMERGENTSTOPSTEPS ;
  }
  //����
  pmc->status_flags &= ~MOTO_STATUS_INHISR;
  
}

uint32_t MOTO_GetRemaindSteps(Axis_t axis)
{
  return ctrlinfo[axis].steps[0]+ctrlinfo[axis].steps[2]+ctrlinfo[axis].steps[1];
}