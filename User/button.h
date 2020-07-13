#ifndef _BUTTON_H
#define _BUTTON_H
#include "main.h"

#define BT_START_IDX    0
#define BT_JS1_IDX      1
#define BT_JS2_IDX      2
#define BT_RXC_IDX      3
#define BT_JS1D0_IDX    4
#define BT_JS1D1_IDX    5
#define BT_JS1D2_IDX    6
#define BT_JS1D3_IDX    7
#define BT_JS2D0_IDX    8
#define BT_JS2D1_IDX    9 
#define BT_JS2D2_IDX    10
#define BT_JS2D3_IDX    11

typedef void (*BtnCallBack)(uint16_t);
typedef  struct _btn
{
  GPIO_TypeDef* Port;           // corresponding port  
  uint16_t      Pin;            // corresponding pin
  uint16_t      Notify;         // corresponding notify code
  uint16_t      ClosedState;    // corresponding state when button pressed,high level or low level
  uint16_t      ClosedCount;    // pressed counting timer
  uint16_t      OpenedCount;    // released ounting timer
  uint16_t      IsClosed;       // represent if button is valid pressed
  BtnCallBack   OnClosed;       // callback function called when valid pressed occured
  BtnCallBack   OnOpened;       // callback function called when valid released occured
}Button_t;


void Button_Init(Button_t* btn, GPIO_TypeDef* port,uint32_t pin, uint16_t notify,uint16_t closedstate,
  BtnCallBack CBclosed,BtnCallBack CBopened);

void Button_Fresh(Button_t *btn);
#endif
