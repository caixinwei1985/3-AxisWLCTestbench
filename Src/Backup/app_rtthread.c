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
    //�ȴ������ƶ��ź�
    rt_event_recv(Evt_Moto_Pos,MOTO_EVENT_NEAR_ENTER|MOTO_EVENT_FAR_ENTER|MOTO_EVENT_OVERFAR|MOTO_EVENT_OVERNEAR,
                  RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,RT_NULL);
    // ���������ƶ�����
    MOTO_EmergentBreak(MOTO_GetRunningAxis());
    // �رյ�����
    rt_enter_critical();
    if(Thread_Moto != RT_NULL)
    {
      //ǿ�ƹرյ�������߳�
      rt_thread_delete(Thread_Moto);
      Thread_Moto = rt_thread_create("Moto",MotoCtrlTask,RT_NULL,256,5,10);
    }
    //����������
    rt_exit_critical();
    // �ȴ������������
    rt_event_recv(Evt_Moto_Pos,MOTO_EVENT_STEPOVER,RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,RT_NULL);
    
    // ��ǰ��������λ
    MOTO_Reset(MOTO_GetRunningAxis());
    // �ȴ�����ص����λ��
    if(RT_EOK == rt_event_recv(Evt_Moto_Pos,MOTO_EVENT_ZERO_ENTER,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,20000,RT_NULL))
    {
      
      // ���������ƶ�����
      MOTO_EmergentBreak(MOTO_GetRunningAxis());
      // �ȴ������������
      rt_event_recv(Evt_Moto_Pos,MOTO_EVENT_STEPOVER,RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,RT_NULL);
      // notify host reset finished
      // �������������������
      if(RT_NULL != Thread_Moto)
        rt_thread_startup(Thread_Moto);
    }
    else
    {
      //���㳬ʱ����
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

