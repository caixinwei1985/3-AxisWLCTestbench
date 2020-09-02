/*
 * File      : app_rtthread.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2012 - 2018, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include <rtthread.h>
#include "moto.h"
#include "comm_definition.h"
#include "cmdlink.h"
#include "usart.h"
#include "iwdg.h"
typedef void (*MoveAxisFuncType)(int32_t,uint32_t);
typedef void (*StopAxisFuncType)(void);
uint32_t READ_IO(GPIO_TypeDef* port,uint16_t pin)     
{
  if((LL_GPIO_ReadInputPort(port) & pin) == 0)
    return 0;
  else
    return 1;
//  return (((port->IDR & pin) != 0)?1:0);
}
__STATIC_INLINE void WRITE_IO(GPIO_TypeDef* port,uint16_t pin,uint16_t val) 
{  
  if(val!=0){
    LL_GPIO_SetOutputPin(port,pin);}
  else{
    LL_GPIO_ResetOutputPin(port,pin);}
}
static Axis_t motionAxis;
static uint8_t motionDirection;
static MoveAxisFuncType MoveAxisFunc[3];
static StopAxisFuncType StopAxisFunc[3];
static struct rt_thread thread_moto;
static struct rt_thread thread_emergent;
static struct rt_thread thread_io;
static struct rt_thread thread_idle;
ALIGN(8)
static rt_uint32_t  thread_moto_stack[128];
ALIGN(8)
static rt_uint32_t thread_emergent_stack[128];
ALIGN(8)
static rt_uint32_t thread_io_stack[128];
ALIGN(8)
static rt_uint32_t thread_idle_stack[32];
struct rt_mailbox mb_moto;
struct rt_mailbox mb_IO_rw;
struct rt_mailbox mb_IO_trigger;
static rt_uint32_t mb_io_pool[4];
static rt_uint32_t mb_moto_pool[4];
static rt_uint32_t mb_iotrig_pool[8];
struct rt_event   evt_moto_pos;

Axis_t  GetMotionAxis()
{
  return motionAxis;
}
uint8_t GetMotionDirection()
{
  return motionDirection;
}

static void MoveAxisX(int32_t steps,uint32_t speed)
{
  rt_kputs("Move Axis X\n");
  motionAxis = AxisX;
  motionDirection = steps > 0 ? 1:0;
  MOTO_Set(0,steps,speed);
  MOTO_Set(1,steps,speed);
  MOTO_Start(0);
  MOTO_Start(1);
}
static __inline void StopAxisX()
{
  MOTO_Stop(0);
  MOTO_Stop(1);
}
static void MoveAxisY(int32_t steps,uint32_t speed)
{
  rt_kputs("Move Axis Y\n");
  motionAxis = AxisY;
  motionDirection = steps > 0 ? 1:0;
  MOTO_Set(0,steps,speed);
  MOTO_Set(1,0-steps,speed);
  MOTO_Start(0);
  MOTO_Start(1);
  
}
static __inline void StopAxisY()
{
  MOTO_Stop(0);
  MOTO_Stop(1);
}
static void MoveAxisZ(int32_t steps,uint32_t speed)
{
  rt_kputs("Move Axis Z\n");
  motionAxis = AxisZ;
  motionDirection = steps>0?1:0;  
//  if(pos == MOTO_POSITION_FAR || pos == MOTO_POSITION_OVERFAR){
//    if(motionDirection)
//      rt_event_send(&evt_moto_pos,MOTO_EVENT_STEPOVER_2);
//  }
//  if(pos == MOTO_POSITION_NEAR || pos == MOTO_POSITION_OVERNEAR){
//    if(!motionDirection)
//      rt_event_send(&evt_moto_pos,MOTO_EVENT_STEPOVER_2);
//  }
  MOTO_Set(2,steps,speed);
  MOTO_Start(2);
}
static __inline void StopAxisZ()
{
  MOTO_Stop(2);
}
void Moto_Emergent_Break_Task(void* pdata);
void Moto_Ctrl_Task(void* pdata)
{
  uint8_t* pframe;
  uint32_t axis_evt[3];
  rt_kputs("Moto task start\n");
  motionAxis = AxisUnknown;
  MoveAxisFunc[AxisX] = MoveAxisX;
  MoveAxisFunc[AxisY] = MoveAxisY;
  MoveAxisFunc[AxisZ] = MoveAxisZ;
  StopAxisFunc[AxisX] = StopAxisX;
  StopAxisFunc[AxisY] = StopAxisY;
  StopAxisFunc[AxisZ] = StopAxisZ;
  for(;;)
  {
    rt_mb_recv(&mb_moto,(rt_uint32_t*)&pframe,RT_WAITING_FOREVER);

    // 应答主机命令
    send_hex_message(pframe,2);
    MOTO_Enable();
    rt_thread_delay(20);
    if(pframe[1] == 0x00)
    {
      rt_uint32_t e = 0;
      Axis_t axis = (Axis_t)pframe[2];
//      uint8_t acc = pframe[3];
      uint16_t speed = pframe[3]*256+pframe[4];
      int32_t steps = (pframe[5]<<24)+(pframe[6]<<16)+(pframe[7]<<8)+pframe[8];
      uint16_t timeout = (pframe[9]<<8)+pframe[10];
      timeout = timeout*10;
      axis_evt[AxisX] = MOTO_EVENT_STEPOVER_0 | MOTO_EVENT_STEPOVER_1;
      axis_evt[AxisY] = MOTO_EVENT_STEPOVER_0 | MOTO_EVENT_STEPOVER_1;
      axis_evt[AxisZ] = MOTO_EVENT_STEPOVER_2;
      // 释放用于存储消息命令的内存块
      rt_mp_free(pframe);
      rt_event_recv(&evt_moto_pos,0xffffffff,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,0,RT_NULL);
      // 启动紧急制动任务
      rt_thread_init(&thread_emergent,"emer",Moto_Emergent_Break_Task,(void*)(rt_uint32_t)axis,thread_emergent_stack,
          sizeof(thread_emergent_stack),4,10);
      rt_thread_startup(&thread_emergent);
      
      MoveAxisFunc[axis](steps,speed);
      if(rt_event_recv(&evt_moto_pos,axis_evt[axis],RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,timeout,RT_NULL) != RT_EOK) {
        StopAxisFunc[axis]();
        report_notify(NTF_MOTO_OT_X + axis*6);
      } else {
        StopAxisX();
        report_notify(NTF_MOTO_COMP_X + axis*6);
      }

      rt_thread_detach(&thread_emergent);
      motionAxis = AxisUnknown;
    }
    else if(pframe[1] == 0x01)    // reset 
    {
      rt_uint32_t rs;
      rt_uint32_t evt;
      Axis_t axis = pframe[2];
      uint16_t speed = pframe[3]*256 + pframe[4];
      uint32_t timeout = pframe[5]*256 + pframe[6];
      int32_t steps;
      rt_mp_free(pframe);
      if(MOTO_GetPosition(axis) != MOTO_POSITION_ZERO)
      {
        rt_event_recv(&evt_moto_pos,0xffffffff,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,0,RT_NULL);
        // 启动紧急制动任务
        rt_thread_init(&thread_emergent,"emer",Moto_Emergent_Break_Task,(void*)(rt_uint32_t)axis,thread_emergent_stack,
            sizeof(thread_emergent_stack),4,10);
        rt_thread_startup(&thread_emergent);
        
        switch(MOTO_GetPosition(axis)){
          case MOTO_POSITION_NEAR:
          case MOTO_POSITION_OVERNEAR:
          case MOTO_POSITION_ZERONEAR:
            steps = 2000000;
            break;
          case MOTO_POSITION_ZERO:
          case MOTO_POSITION_FAR:
          case MOTO_POSITION_OVERFAR:
          case MOTO_POSITION_ZEROFAR:
            steps = -2000000;
            break;
        }
        MoveAxisFunc[axis](steps,1000);
        rt_kprintf("Reset Axis[%d]\n",axis);
//        if(!motionDirection){
          rs = rt_event_recv(&evt_moto_pos,(MOTO_EVENT_ZERO_ENTER_X<<9*axis),  RT_EVENT_FLAG_CLEAR|RT_EVENT_FLAG_OR,timeout,RT_NULL);
//        } else {
//          rs = rt_event_recv(&evt_moto_pos,(MOTO_EVENT_ZERO_ENTER_X<<9*axis),RT_EVENT_FLAG_CLEAR|RT_EVENT_FLAG_OR,timeout,RT_NULL);
//          StopAxisFunc[axis]();
//          if(rs == RT_EOK){
//            rt_thread_delay(200);
//            MoveAxisFunc[axis](-100000,1000);
//            rs = rt_event_recv(&evt_moto_pos,(MOTO_EVENT_ZERO_EXIT_X<<9*axis),RT_EVENT_FLAG_CLEAR|RT_EVENT_FLAG_OR,timeout,RT_NULL);
//          }
//        }
        StopAxisFunc[axis]();
        if(rs != RT_EOK)
        {
          report_notify(NTF_MOTO_OT_X + axis*6);
        }
        else
        {
          report_notify(NTF_MOTO_RESET_X + axis*6);
        }
        rt_thread_detach(&thread_emergent);
      }
      rt_kprintf("Position:%d\n",MOTO_GetPosition(axis));
      motionAxis = AxisUnknown;
    }
    else if(0x02 == pframe[1])
    {
      uint16_t startspeed = pframe[4]*256+pframe[5];
      uint8_t acc = pframe[2];
      uint8_t dec = pframe[3];
      rt_mp_free(pframe);
      MOTO_Config(acc,dec,startspeed);
      rt_kprintf("Moto Config Acc:%d,Dec:%d,StartSpeed:%d\n",acc,dec,startspeed);
    }
//    MOTO_Disable();
  }
}

void Moto_Emergent_Break_Task(void* pdata)
{
//  for(;;)
  {
    rt_uint32_t value;
    rt_uint32_t flags = MOTO_EVENT_NEAR_ENTER_X|MOTO_EVENT_FAR_ENTER_X|MOTO_EVENT_OVERFAR_X|MOTO_EVENT_OVERNEAR_X;
    Axis_t axis = (Axis_t)pdata;
    flags = flags << 9*axis;
    //等待紧急制动信号
    rt_event_recv(&evt_moto_pos, flags, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,RT_NULL);
    // 关闭调度器
    rt_enter_critical();
    //强制关闭电机控制线程
    rt_thread_detach(&thread_moto);
    //启动调度器
    rt_exit_critical();
    //清空邮箱内容   
    while(rt_mb_recv(&mb_moto,&value,0)==RT_EOK);
    // 启动紧急制动动作
//    MOTO_EmergentBreak(axis);
    if(axis == AxisZ) {
      MOTO_EmergentBreak(2);
      rt_event_recv(&evt_moto_pos,MOTO_EVENT_STEPOVER_2,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,RT_NULL);
    } else {
      MOTO_EmergentBreak(0);
      MOTO_EmergentBreak(1);
      rt_event_recv(&evt_moto_pos,MOTO_EVENT_STEPOVER_0 | MOTO_EVENT_STEPOVER_1,RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,RT_NULL);
    }


    // 等待电机紧急刹车结束
//    rt_event_recv(&evt_moto_pos,(MOTO_EVENT_STEPOVER_X<<9*axis),RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,RT_NULL);
    switch(axis)
    {
      case AxisX:report_notify(NTF_MOTO_EMER_X);break;
      case AxisY:report_notify(NTF_MOTO_EMER_Y);break;
      case AxisZ:report_notify(NTF_MOTO_EMER_Z);break; 
      default:break;
    }
    log("Moto Emergent stop\n");
//    log("EMB over,reset start\n");
//    // 当前轴向电机复位
//    MOTO_Reset(axis);
//    // 等待电机回到零点位置
//    if(RT_EOK == rt_event_recv(&evt_moto_pos,MOTO_EVENT_ZERO_ENTER,RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,RT_NULL))
//    {
//      // notify host reset finished
//      log("Return to zero ponit\n");
//    }
//    else
//    {
//      log("Emergent reset timeout\n");
//      //回零超时处理
//    }
//    // 启动紧急制动动作
//    MOTO_EmergentBreak(axis);
//    // 等待电机步进结束
//    rt_event_recv(&evt_moto_pos,MOTO_EVENT_STEPOVER,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,RT_NULL);
//    MOTO_Disable();
//    // 清空事件
    rt_event_recv(&evt_moto_pos,0xffffffff,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,0,RT_NULL);
    // 重新启动电机服务任务
    rt_thread_init(&thread_moto,"moto",Moto_Ctrl_Task,RT_NULL,thread_moto_stack,sizeof(thread_moto_stack),15,10);
    rt_enter_critical();
    rt_thread_startup(&thread_moto);
    rt_exit_critical();
    motionAxis = AxisUnknown;
  }
}
__STATIC_INLINE rt_uint16_t ReadReg(rt_int16_t cmd)
{
  uint16_t val = 0xffff;
  switch(cmd)
  {
    case REG_XSW0_R:
      val = READ_IO(XSW0_GPIO_Port,XSW0_Pin);
      break;
    case REG_XSW1_R:
      val = READ_IO(XSW1_GPIO_Port,XSW1_Pin);
      break;
    case REG_XSW2_R:
      val = READ_IO(XSW2_GPIO_Port,XSW2_Pin);
      break;
    case REG_YSW0_R:
      val = READ_IO(YSW0_GPIO_Port,YSW0_Pin);
      break;
    case REG_YSW1_R:
      val = READ_IO(YSW1_GPIO_Port,YSW1_Pin);
      break;
    case REG_YSW2_R:
      val = READ_IO(YSW2_GPIO_Port,YSW2_Pin);
      break;
    case REG_ZSW0_R:
      val = READ_IO(ZSW0_GPIO_Port,ZSW0_Pin);
      break;
    case REG_ZSW1_R:
      val = READ_IO(ZSW1_GPIO_Port,ZSW1_Pin);
      break;
    case REG_ZSW2_R:
      val = READ_IO(ZSW2_GPIO_Port,ZSW2_Pin);
      break;
    case REG_JS1BT_R:
      val = READ_IO(BT_JS1_GPIO_Port,BT_JS1_Pin);
      break;
    case REG_JS1D0_R:
      val = READ_IO(JS1_D0_GPIO_Port,JS1_D0_Pin);
      break;
    case REG_JS1D1_R:
      val = READ_IO(JS1_D1_GPIO_Port,JS1_D1_Pin);
      break;
    case REG_JS1D2_R:
      val = READ_IO(JS1_D2_GPIO_Port,JS1_D2_Pin);
      break;
    case REG_JS1D3_R:
      val = READ_IO(JS1_D3_GPIO_Port,JS1_D3_Pin);
      break;
    case REG_JS2BT_R:
      val = READ_IO(BT_JS2_GPIO_Port,BT_JS2_Pin);
      break;
    case REG_JS2D0_R:
      val = READ_IO(JS2_D0_GPIO_Port,JS2_D0_Pin);
      break;
    case REG_JS2D1_R:
      val = READ_IO(JS2_D1_GPIO_Port,JS2_D1_Pin);
      break;
    case REG_JS2D2_R:
      val = READ_IO(JS2_D2_GPIO_Port,JS2_D2_Pin);
      break;
    case REG_JS2D3_R:
      val = READ_IO(JS2_D3_GPIO_Port,JS2_D3_Pin);
      break;
    case REG_START_R:
      val = READ_IO(BT_START_GPIO_PORT,BT_START_Pin);
      break;
    case REG_PROT0_R:
      val = READ_IO(PROT0_GPIO_Port,PROT0_Pin);
      break;
    case REG_PROT1_R:
      val = READ_IO(PROT1_GPIO_Port,PROT1_Pin);
      break;
    case REG_PROT2_R:
      val = READ_IO(PROT2_GPIO_Port,PROT2_Pin);
      break;
    case REG_NTC1_R:
      val = ADC_Get_NTC_Volt(ADC_CHANNEL_NTC1);
      break;
    case REG_NTC2_R:
      val = ADC_Get_NTC_Volt(ADC_CHANNEL_NTC2);
      break;
    case REG_NTC3_R:
      val = ADC_Get_NTC_Volt(ADC_CHANNEL_NTC3);
      break;
    case REG_MOTCUR_R:
      break;
    case REG_VBAT_R:
      val = ADC_Get_VBAT();
      break;
    case REG_XPOS_R :
      val = MOTO_GetPosition(AxisX);
      break;
    case REG_YPOS_R:
      val = MOTO_GetPosition(AxisY);
      break;
    case REG_ZPOS_R:
      val = MOTO_GetPosition(AxisZ);
      break;
  }
  return val;
}

 /**
  * @brife Write Out pin state.
  * @note  As the beginnig logic,write low means output closed.But actually,Lamp,Fan,Laser,power switch are high valid
  * @param out pin logic address
  * @param state gonna to set
  */
static void WriteReg(rt_uint16_t cmd,rt_uint16_t val)
{
  Axis_t axis = MOTO_GetRunningAxis();
  switch(cmd)
  {
    case REG_BUZZER_W:
      WRITE_IO(SW_BUZZER_GPIO_Port,SW_BUZZER_Pin,val);
      break;
    case REG_LED_W:
      WRITE_IO(SW_LED0_GPIO_Port,SW_LED0_Pin,val);
      break;
    case REG_LAMP_W:
      val = !val;
      WRITE_IO(SW_LAMP_GPIO_Port,SW_LAMP_Pin,val);
      break;
    case REG_LASER_W:
      val = !val;
      WRITE_IO(SW_LASER_GPIO_Port,SW_LASER_Pin,val);
      break;
    case REG_FAN1_W:
      val = !val;;
      WRITE_IO(FAN1_GPIO_Port,FAN1_Pin,val);
      break;
    case REG_FAN2_W:
      val = !val;
      WRITE_IO(FAN2_GPIO_Port,FAN2_Pin,val);
      break;
    case REG_SW24V_W:
      val = !val;
      WRITE_IO(SW_24V_GPIO_Port,SW_24V_Pin,val);
      break;
    case REG_SW5V_W:
      val = !val;
      WRITE_IO(SW_5V_GPIO_Port,SW_5V_Pin,val);
      break;
    case REG_DRVEN_W:
      if(axis == AxisUnknown)
        WRITE_IO(MOTOEN_GPIO_Port,MOTOEN_Pin,val);
      break;
    case REG_XDIR_W:
      if(axis == AxisUnknown)
        WRITE_IO(MOTOXDIR_GPIO_Port,MOTOXDIR_Pin,val);
      break;
    case REG_YDIR_W:
      if(axis == AxisUnknown)
        WRITE_IO(MOTOYDIR_GPIO_Port,MOTOYDIR_Pin,val);
      break;
    case REG_ZDIR_W:
      if(axis == AxisUnknown)
        WRITE_IO(MOTOZDIR_GPIO_Port,MOTOZDIR_Pin,val);
      break;
    case REG_HPO1_W:
      val = !val;
      WRITE_IO(HPOUT1_GPIO_Port,HPOUT1_Pin,val);
      break;
    case REG_HPO2_W:
      val = !val;
      WRITE_IO(HPOUT2_GPIO_Port,HPOUT2_Pin,val);
      break;
  }

  
}
void IO_ReadWrite_Task(void* pdata)
{
  for(;;)
  {
    rt_uint8_t* pframe;
    rt_uint16_t cmd;
    rt_uint16_t val;
    rt_mb_recv(&mb_IO_rw,(rt_uint32_t*)&pframe,RT_WAITING_FOREVER);
    // 获取命令代码
    cmd = pframe[0]*256 + pframe[1];
    val = pframe[2]*256 + pframe[3];
    if(pframe[0] == 0x01)
    {
      WriteReg(cmd,val);
      send_hex_message(pframe,2);
    }
    else if(pframe[0] == 0x02)
    {
      val = ReadReg(cmd);
      pframe[2] = val>>8;
      pframe[3] = val&0xff;
      send_hex_message(pframe,4);
      
    }
    //pframe指向的是内存池中内存块，使用完后需要释放
    rt_mp_free(pframe);
  }
  rt_kputs("IO task exit\n");
}

void User_Idle_Task(void* pdata)
{
  for(;;)
  {
    HAL_Delay(100);
//    WriteReg(REG_SW24V_W,0);
    HAL_Delay(100);
//    WriteReg(REG_SW24V_W,1);
  }
}
extern struct rt_thread thread_print;
extern struct rt_thread thread_recv;
void MX_RT_Thread_Init(void)
{
  MOTO_Init();
  rt_mb_init(&mb_IO_rw,"io",mb_io_pool,sizeof(mb_io_pool)/4,RT_IPC_FLAG_FIFO);
  rt_mb_init(&mb_moto, "moto",mb_moto_pool,sizeof(mb_moto_pool)/4,RT_IPC_FLAG_FIFO);
  rt_mb_init(&mb_IO_trigger,"trigger",mb_iotrig_pool,sizeof(mb_iotrig_pool)/4,RT_IPC_FLAG_FIFO);
  rt_event_init(&evt_moto_pos,"Mpos",RT_IPC_FLAG_PRIO);
  
  rt_thread_init(&thread_moto,"moto",Moto_Ctrl_Task,RT_NULL,thread_moto_stack,sizeof(thread_moto_stack),11,10);
  rt_thread_startup(&thread_moto);

  rt_thread_init(&thread_idle,"idle",User_Idle_Task,RT_NULL,thread_idle_stack,sizeof(thread_idle_stack),30,2);
  rt_thread_startup(&thread_idle);
  //IO读写线程
  rt_thread_init(&thread_io,"io",IO_ReadWrite_Task,RT_NULL,thread_io_stack,sizeof(thread_io_stack),12,20);
  rt_thread_startup(&thread_io);
  
  rt_thread_startup(&thread_print);
  rt_thread_startup(&thread_recv);
}

// 对所有机械触点的触发做处理
void Input_Trigger_Proc(rt_uint16_t notify)
{
  Axis_t axis = MOTO_GetRunningAxis();
  switch(notify)
  {
    case NTF_RXCOL_TRIG:
      MOTO_Stop(axis);
      rt_event_send(&evt_moto_pos,MOTO_EVENT_RX_COLLID);
      break;
    case NTF_START_PRESS:
      break;
    case NTF_JS1BT_PRESS:
      break;
    case NTF_JS2BT_PRESS:
      break;
    case NTF_JS1D0_PRESS:
      break;
    case NTF_JS1D1_PRESS:
      break;
    case NTF_JS1D2_PRESS:
      break;
    case NTF_JS1D3_PRESS:
      break;
    case NTF_JS2D0_PRESS:
      break;
    case NTF_JS2D1_PRESS:
      break;
    case NTF_JS2D2_PRESS:
      break;
    case NTF_JS2D3_PRESS:
      break;
    default:
      break;
  }
}
void Button_Notify(uint16_t notify)
{
  report_notify(notify);
  Input_Trigger_Proc(notify);
}
void MX_RT_Thread_Process(void)
{
  Axis_t axis;
  rt_uint32_t mail;
  report_notify(NTF_DEVICE_RESET);
  uint32_t loopcnt = 0;           
  rt_kprintf("Program created at %s,%s\n",__TIME__,__DATE__);
  for(;;)
  {
//    HAL_IWDG_Refresh(&hiwdg);
//    rt_kputs("wait for delay 1\n");
    rt_thread_delay(200);
    LL_GPIO_SetOutputPin(SW_LED0_GPIO_Port,SW_LED0_Pin);
//    rt_kputs("abcdefghijklmnopqrstuvwxyz\n");
//    rt_kputs("abcdefghijklmnopqrstuvwxyz\n");
    rt_thread_delay(200);  
//    rt_kputs("ABCDEFGHIJKLMNOPQRSTUVWXYZ\n");
//    rt_kputs("ABCDEFGHIJKLMNOPQRSTUVWXYZ\n");
    LL_GPIO_ResetOutputPin(SW_LED0_GPIO_Port,SW_LED0_Pin);
//    rt_kprintf("Loop times : %lu\n",++loopcnt);
    axis = GetMotionAxis();

    if(axis != AxisUnknown)
    {
      if(axis == AxisZ)
        rt_kprintf("Motor:%d -Remaind steps:%lu\n",2,MOTO_GetRemaindSteps(2));
      else { 
        rt_kprintf("Motor:%d -Remaind steps:%lu\n",0,MOTO_GetRemaindSteps(0));
        rt_kprintf("Motor:%d -Remaind steps:%lu\n",1,MOTO_GetRemaindSteps(1));
      }
    }
    else
    {
//      rt_kprintf("USART2 CR1:0x%08x, ",USART2->CR1);
      if(Uart_Get_Error())
      {
        rt_kputs("Frame error reset\n");
        Uart_Reset_Error();
      }
    }
    
  }
}
