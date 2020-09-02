#include "cmdlink.h"
#include "stm32f091xc.h"
//#include "stm32f0xx_hal.h"
#include <rtthread.h>
#include <string.h>
#include "isp.h"
USART_TypeDef*  s_usarts[UART_CH_MAX];

uint8_t f_inbuf[UART_CH_MAX][INBUFLENMAX] = {0};
uint8_t f_outbuf[UART_CH_MAX][OUTBUFLENMAX] = {0};

uint8_t f_lastbyteflag[UART_CH_MAX] = {0};
// indicate the byte queue tranfer is going
uint8_t f_is_tranfering[UART_CH_MAX] = {0};
uint32_t recved_time;
//extern uint32_t systick;
//receive buffer writing index
volatile uint16_t f_inbuf_wr_idx[UART_CH_MAX] = {0}; 

//receive buffer reading index
volatile uint16_t f_inbuf_rd_idx[UART_CH_MAX] = {0};

//count of byte to read
volatile uint16_t f_inbuf_cnt[UART_CH_MAX] = {0};
volatile uint16_t f_inbuf_requiredbytes[UART_CH_MAX] = {0};

volatile uint16_t f_outbuf_wr_idx[UART_CH_MAX] = {0};
volatile uint16_t f_outbuf_rd_idx[UART_CH_MAX] = {0};

//count of byte to transfer 
volatile uint16_t f_outbuf_cnt[UART_CH_MAX] = {0};
volatile uint16_t f_outbuf_requiredspace[UART_CH_MAX];
// read timeout timer counter
volatile int16_t f_inbuf_rd_to[UART_CH_MAX];
struct rt_semaphore tx_sem[UART_CH_MAX];
struct rt_semaphore rx_sem[UART_CH_MAX];

struct rt_messagequeue print_mq;
static print_msg_t print_mq_buf[PRINT_MQ_LEN];

// 用于字符串打印输出的内存池
static struct rt_mempool mp_print;
ALIGN(8)
static uint8_t mpool_print[2048];

// 用于hex命令输出的内存池
static struct rt_mempool mp_hexcmd;
ALIGN(8)
static uint8_t mpool_hexcmd[512];
// 接收任务发送命令队列的内存池
static struct rt_mempool mp_recv;
static uint8_t mpool_revc[256];

struct rt_thread thread_print;
struct rt_thread thread_recv;
ALIGN(8)
static rt_uint32_t thread_print_stack[128];
ALIGN(8)
static rt_uint32_t thread_recv_stack[128];

extern struct rt_mailbox mb_moto;
extern struct rt_mailbox mb_IO_rw;
static uint8_t msg_header[4];
static uint8_t msg_chk;
void print_task(void* pdata)
{
  print_msg_t msg;
  uint32_t i;
  uint8_t padds[8] ={0};
  for(;;)
  {
    rt_mq_recv(&print_mq,&msg,sizeof(print_msg_t),RT_WAITING_FOREVER);
    msg_chk = 0;
    if(msg.type == 0)
    {
      msg_header[0] = 0xcc;
      msg_header[1] = 0x33;
    }
    if(msg.type == 1)
    {
      msg_header[0] = 0xaa;
      msg_header[1] = 0x55;
    }
    msg_header[2] = (msg.length+5)>>8;
    msg_header[3] = (msg.length+5)&0xff;
    
    for(i=0;i<4;i++)
      msg_chk ^= msg_header[i];
    for(i=0;i<msg.length;i++)
      msg_chk ^= msg.pmsg[i];
    async_write(1,padds,4);
    async_write(1,msg_header,4);
    async_write(1,msg.pmsg,msg.length);
    async_write(1,&msg_chk,1);
    rt_mp_free(msg.pmsg);
  }
}
__weak void frame_proc_callback(uint8_t* frame,uint16_t count)
{
  
  if( 0x03 == *frame )
  {
    // 从内存池中申请存储帧信息的缓冲区
    uint8_t* p = rt_mp_alloc(&mp_recv,20);
    rt_memcpy(p,frame,count);
    if(RT_EOK != rt_mb_send_wait(&mb_moto,(rt_uint32_t)p,100))
    {
      rt_kputs("moto mb fail\n");
      rt_mp_free(p);
    }
  }
  else if(0x01 == *frame  || 0x02 == *frame )
  {
    rt_uint8_t *p = rt_mp_alloc(&mp_recv,20);
    rt_memcpy(p,frame,count);
    if(RT_EOK != rt_mb_send_wait(&mb_IO_rw,(rt_uint32_t)p,100))
    {
      rt_kputs("io mb fail\n");
      rt_mp_free(p);
    }
  }
  else if(0xff == *frame )
  {
    if( 0x00 == *(frame+1))
    {
      BootConfig_System(); 
    }
    if(0x01 == *(frame+1))
    {
      NVIC_SystemReset();
    }
  }
  
}
void recv_task(void* pdata)
{
  uint8_t b;
  uint16_t len;
  uint8_t chk;
  uint8_t buf[32];
  for(;;)
  {
    if(0==async_read(1,&b,1))
      continue;
    if(0x5A == b)
    {
      chk = 0;
      chk ^= b;
      if(0 == async_read(1,&b,1))
        continue;
      if(0xa5 == b)
      {
        chk ^= b;
        if(0 != async_read(1,buf,2))
        {
          len = buf[0]*256 + buf[1];
          if(0 != async_read(1,buf+2,len-4))
          {
            uint16_t i;
            /*buf里包含length，命令，和chk，长度为len-2，不算chk，长度为len-3*/
            for(i=0;i<len-3;i++)
              chk ^= buf[i];
            if(chk == buf[len-3])
            {
              // 传入操作帧首地址，不包含传输帧的前导和长度字节
              frame_proc_callback(buf+2,len-5);
            }
          }
        }
      }
    }
  }
}
void init_cmdlink(void)
{
  uint8_t ch = 0;
  s_usarts[0] = USART1;
  s_usarts[1] = USART2;
  for(ch = 0; ch < UART_CH_MAX; ch++)
  {
    f_inbuf_wr_idx[ch]  = 0;
    f_inbuf_rd_idx[ch]  = 0;
    f_outbuf_wr_idx[ch] = 0;
    f_outbuf_rd_idx[ch] = 0;
    f_inbuf_cnt[ch]     = 0;
    f_outbuf_cnt[ch]    = 0;
    rt_sem_init(&tx_sem[ch],"0",0,RT_IPC_FLAG_PRIO);
    f_outbuf_requiredspace[ch] = 0xffff;
    rt_sem_init(&rx_sem[ch],"",0,RT_IPC_FLAG_PRIO);
    f_inbuf_requiredbytes[ch] = 0xffff;
  }
  rt_mq_init(&print_mq,"mq",(void*)print_mq_buf,sizeof(print_msg_t),sizeof(print_mq_buf),RT_IPC_FLAG_FIFO);
  //启动打印任务
  rt_thread_init(&thread_print,"print",print_task,RT_NULL,thread_print_stack,sizeof(thread_print_stack),15,20);
  rt_thread_init(&thread_recv,"recv",recv_task,RT_NULL,thread_recv_stack,sizeof(thread_recv_stack),14,20);
  rt_mp_init(&mp_print,"mpp",mpool_print,sizeof(mpool_print),64);
  rt_mp_init(&mp_recv,"mpr",mpool_revc,sizeof(mpool_revc),16);
  rt_mp_init(&mp_hexcmd,"hex",mpool_hexcmd,sizeof(mpool_hexcmd),32);
  
}

void flush_cmdlink()
{
    uint8_t ch = 0;
    for(ch = 0; ch < UART_CH_MAX; ch++)
    {
      f_inbuf_wr_idx[ch]  = 0;
      f_inbuf_rd_idx[ch]  = 0;
      f_inbuf_cnt[ch]     = 0;
    }    
}

uint8_t sync_read(uint8_t ch,uint8_t* pbuff,uint8_t count,uint16_t timeout)
{
    uint8_t i;
    for( i = 0;i < count; i++)
    {
      f_inbuf_rd_to[ch] = timeout;
      while(f_inbuf_cnt[ch] == 0 && f_inbuf_rd_to[ch] > 0);
      
      if(f_inbuf_rd_to[ch] == 0)
      {
          return 0;
      }

      pbuff[i] = f_inbuf[ch][f_inbuf_rd_idx[ch]];
      f_inbuf[ch][f_inbuf_rd_idx[ch]] = 0xcc;
      f_inbuf_rd_idx[ch]++;
      if(INBUFLENMAX == f_inbuf_rd_idx[ch])
      {
        f_inbuf_rd_idx[ch] = 0;
      }
      s_usarts[ch]->CR1 &= ~USART_CR1_RXNEIE;
      f_inbuf_cnt[ch]--;
      s_usarts[ch]->CR1 |= USART_CR1_RXNEIE;
    }
    return i;
}

/*********************************************************
*        name:async_read
* description:
*  parameters:
*return value:
**********************************************************/
uint8_t async_read(uint8_t ch,uint8_t* pbuff,uint8_t count)
{
  uint8_t i;
  LL_USART_DisableIT_RXNE(s_usarts[ch]);
  if(0 == f_inbuf_cnt[ch])
  {
    if(f_inbuf_rd_idx[ch] != f_inbuf_wr_idx[ch])
    {
      f_inbuf_rd_idx[ch] = 0;
      f_inbuf_wr_idx[ch] = 0;
    }
  }
  LL_USART_EnableIT_RXNE(s_usarts[ch]);
  if(count > f_inbuf_cnt[ch])
  {
    f_inbuf_requiredbytes[ch] = count;
    if(RT_EOK != rt_sem_take(&rx_sem[ch],RT_WAITING_FOREVER))   
    { 
      f_inbuf_requiredbytes[ch] = 0xffff;
      return 0;
    }
    // 若某些异常状况导致的信号量等待结束，再次确认所剩字节数是否满足读取需求
    if(count > f_inbuf_cnt[ch])
      return 0;
  }
  for(i = 0;i < count;i++)
  {
    pbuff[i] = f_inbuf[ch][f_inbuf_rd_idx[ch]];
    f_inbuf[ch][f_inbuf_rd_idx[ch]] = 0XCC;
    f_inbuf_rd_idx[ch]++;
    if(INBUFLENMAX == f_inbuf_rd_idx[ch])
    {
      f_inbuf_rd_idx[ch] = 0;
    }
    s_usarts[ch]->CR1 &= ~USART_CR1_RXNEIE;
    f_inbuf_cnt[ch]--;
    s_usarts[ch]->CR1 |= USART_CR1_RXNEIE;
  }
  return i;
}

/*********************************************************
*        name:async_write
* description:
*  parameters:
*return value:
**********************************************************/
uint8_t async_write(uint8_t ch,uint8_t* pbuff,uint8_t count)
{
  uint8_t i;
  LL_USART_DisableIT_TXE(s_usarts[ch]);
  if(0 == f_outbuf_cnt[ch])
  {
    if(f_outbuf_rd_idx[ch] != f_outbuf_wr_idx[ch])
    {
      f_outbuf_rd_idx[ch] = 0;
      f_outbuf_wr_idx[ch] = 0;
    }
  }
  LL_USART_EnableIT_TXE(s_usarts[ch]);
  // 对于写操作，总能在有限时长内获得足够的写入空间
  while(count > OUTBUFLENMAX - f_outbuf_cnt[ch])
  {
    f_outbuf_requiredspace[ch] = count;
    if(RT_EOK != rt_sem_take(&tx_sem[ch],100))
    {
      f_outbuf_requiredspace[ch] = 0xffff;
    }
  }
  
  for(i = 0;i < count;i++)
  {
    f_outbuf[ch][f_outbuf_wr_idx[ch]] = pbuff[i];
      f_outbuf_wr_idx[ch]++;

    if(OUTBUFLENMAX == f_outbuf_wr_idx[ch])
    {
      f_outbuf_wr_idx[ch] = 0;
    }
    LL_USART_DisableIT_TXE(s_usarts[ch]);
    f_outbuf_cnt[ch]++;
    LL_USART_EnableIT_TXE(s_usarts[ch]);
  }
//  if(f_is_tranfering[ch] == 0)
//  {
//    s_usarts[ch]->TDR = f_outbuf[ch][f_outbuf_rd_idx[ch]++];
//    if(f_outbuf_rd_idx[ch] == sizeof(f_outbuf[ch]))
//      f_outbuf_rd_idx[ch] = 0;
//    f_outbuf_cnt[ch]--;
//    f_is_tranfering[ch] = 1;
//  }
//  LL_USART_EnableIT_TXE(s_usarts[ch]);
  return i;  
}

void hal_transfer_byte_IT(uint8_t ch)
{
  if(f_outbuf_cnt[ch]>0)
  {
    s_usarts[ch]->TDR = f_outbuf[ch][f_outbuf_rd_idx[ch]++];
    if(f_outbuf_rd_idx[ch] == sizeof(f_outbuf[ch]))
      f_outbuf_rd_idx[ch] = 0;
    f_outbuf_cnt[ch]--;
    
    /** @Note:必须使用大于等于条件，否则极小概率会出现数据不同步，条件永远无法满足，导致发送线程锁死 */
    if(sizeof(f_outbuf[ch])-f_outbuf_cnt[ch] >= f_outbuf_requiredspace[ch])
    {
      f_outbuf_requiredspace[ch]= 0xffff;
      rt_sem_release(&tx_sem[ch]);
    }
  }
  else
  {
    f_is_tranfering[ch] = 0;
    s_usarts[ch]->CR1 &= ~USART_CR1_TXEIE;
  }
}

void hal_receive_byte_IT(uint8_t ch)
{
    
  f_inbuf[ch][f_inbuf_wr_idx[ch]] = s_usarts[ch]->RDR;
  f_inbuf_wr_idx[ch]++;
  if(INBUFLENMAX == f_inbuf_wr_idx[ch])
  {
    f_inbuf_wr_idx[ch] = 0;
  }
  f_inbuf_cnt[ch]++;   
  if(f_inbuf_cnt[ch] > INBUFLENMAX)
    f_inbuf_cnt[ch] = INBUFLENMAX;
  
  /*** @Note: 此处切记不可使用 判等 条件比较，必须是用大于等于，防止数据出错后永远无法满足条件，导致线程锁死*/
  if(f_inbuf_cnt[ch] >= f_inbuf_requiredbytes[ch] )
  {
    rt_sem_release(&rx_sem[ch]);
    f_inbuf_requiredbytes[ch] = 0xffff;
  }
}

uint32_t calc_msglength(uint32_t header)
{
  uint32_t len;
  if(header < 0x20)       
  {
    len = 1 ;
  }
  else if(header < 0x80)
  {
    len = 2 + ((header - 0x20) >> 4);
  }
  else if(header < 0xE0)
  {
    len = 8 + ((header - 0x80) >> 3);
  }
  else
  {
    len = 20 + ((header -0xE0) >> 2);
  } 
  return len;
}
uint8_t get_XORchecksum(uint8_t* data, int16_t count)
{
	int i;
	uint8_t checksum = 0;
	for(i = 0;i< count;i++)
	{
		checksum ^= data[i];
	}
	return checksum;
}

// 输出控制台字符串
void rt_hw_console_output(const char *str)
{
  print_msg_t msg;
  uint16_t len = strlen(str);
  void* p = rt_mp_alloc(&mp_print,20);
  if(p == RT_NULL)
    async_write(1,(uint8_t*)"CO malloc failed",20);
  else
  {
    rt_memcpy(p,str,len);
    msg.type = 0;
    msg.pmsg = p;
    msg.length = len;
    if(RT_EOK != rt_mq_send(&print_mq,&msg,sizeof(msg)))
    {
      rt_mp_free(p);
    }
  }
}

// 发送hex格式的字节信息串
void send_hex_message(const uint8_t* buf,uint16_t len)
{
  print_msg_t msg;
  // 空间在此分配，将在print_task中进行释放
  void*p = rt_mp_alloc(&mp_hexcmd,20);
  if(p == RT_NULL)
    async_write(1,(uint8_t*)"CO malloc failed",20);
  else
  {
    rt_memcpy(p,buf,len);
    msg.type = 1;
    msg.pmsg = p;
    msg.length = len;
    if(RT_EOK != rt_mq_send(&print_mq,&msg,sizeof(msg)))
    {
      rt_mp_free(p);
    }
  }
}

void report_notify(uint16_t notify)
{
  uint8_t buf[2];
  buf[0] = notify>>8;
  buf[1] = notify&0xff;
  send_hex_message(buf,2);
}

