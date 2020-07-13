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

rt_event_t  Evt_Moto_Pos;
rt_thread_t Thread_Moto;
rt_thread_t Thread_Emergent;
rt_mq_t     Mq_Moto;
void MotoCtrlTask(void* pdata)
{
  MotoInitDef_t motoinit;
  for(;;)
  {
    rt_mq_recv(Mq_Moto,&motoinit,sizeof(MotoInitDef_t),RT_WAITING_FOREVER);
    MOTO_Move(motoinit.axis,motoinit.steps,motoinit.speed,motoinit.acc);
  }
}

void MotoEmergentBreakTask(void* pdata)
{
  for(;;)
  {
    //等待紧急制动信号
    rt_event_recv(Evt_Moto_Pos,MOTO_EVENT_NEAR_ENTER|MOTO_EVENT_FAR_ENTER|MOTO_EVENT_OVERFAR|MOTO_EVENT_OVERNEAR,
                  RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,RT_NULL);
    // 启动紧急制动动作
    MOTO_EmergentBreak(MOTO_GetRunningAxis());
    // 关闭调度器
    rt_enter_critical();
    if(Thread_Moto != RT_NULL)
    {
      //强制关闭电机控制线程
      rt_thread_delete(Thread_Moto);
      Thread_Moto = rt_thread_create("Moto",MotoCtrlTask,RT_NULL,256,5,10);
    }
    //启动调度器
    rt_exit_critical();
    // 等待电机步进结束
    rt_event_recv(Evt_Moto_Pos,MOTO_EVENT_STEPOVER,RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,RT_NULL);
    
    // 当前轴向电机复位
    MOTO_Reset(MOTO_GetRunningAxis());
    // 等待电机回到零点位置
    if(RT_EOK == rt_event_recv(Evt_Moto_Pos,MOTO_EVENT_ZERO_ENTER,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,20000,RT_NULL))
    {
      
      // 启动紧急制动动作
      MOTO_EmergentBreak(MOTO_GetRunningAxis());
      // 等待电机步进结束
      rt_event_recv(Evt_Moto_Pos,MOTO_EVENT_STEPOVER,RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,RT_NULL);
      // notify host reset finished
      // 重新启动电机服务任务
      if(RT_NULL != Thread_Moto)
        rt_thread_startup(Thread_Moto);
    }
    else
    {
      //回零超时处理
    }
  }
}
void MX_RT_Thread_Init(void)
{
  Evt_Moto_Pos = rt_event_create("Mpos",RT_IPC_FLAG_PRIO);
  
}

void MX_RT_Thread_Process(void)
{

}

