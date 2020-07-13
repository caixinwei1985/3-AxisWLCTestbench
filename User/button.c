#include "button.h"

/**
  * @brife  Initialize button structure
  * @param  A pointer to button typedef strcuture
  * @param  GPIO port assigned to button
  * @param  Pin assigned to button
  * @param  Corresponding notify code,reference to comm_definition.h
  * @param  Button valid pressed state level at corresponding pin
  * @param  Callback function called when button pressed
  * @param  Callback function called when button released
  */
void Button_Init(Button_t* btn, GPIO_TypeDef* port,uint32_t pin, uint16_t notify,uint16_t closedstate,
  BtnCallBack CBclosed,BtnCallBack CBopened)
{
  btn->Port = port;
  btn->Pin = pin;
  btn->Notify = notify;
  btn->ClosedState = closedstate;
  btn->IsClosed = 0;
  btn->ClosedCount = 0;
  btn->OpenedCount = 0;
  btn->OnClosed = CBclosed;
  btn->OnOpened = CBopened;
}

void Button_Fresh(Button_t* btn)
{
  uint32_t bit = Get_GPIOPinBit(btn->Port,btn->Pin);
  if(bit == btn->ClosedState)
  {
    if(btn->ClosedCount < 0xffff)
    {
      btn->ClosedCount++;
      if(btn->ClosedCount == 20)
      {
        btn->OnClosed(btn->Notify);
      }
    }
    btn->OpenedCount = 0;
  }
  else
  {
    btn->ClosedCount = 0;
  }
}