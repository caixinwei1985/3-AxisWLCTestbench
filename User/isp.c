#include "isp.h"

void BootConfig_System()
{
  HAL_StatusTypeDef status = HAL_OK;
  uint16_t ob[8];
  int i = 0;

  // save option bytes data
  for(i = 0;i<8;i++)
  {
    ob[i] = *(uint16_t*)(0x1ffff800+i*2);
  }

  HAL_FLASH_Unlock();
  HAL_FLASH_OB_Unlock();

  status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
  if(HAL_OK == status)
  {
    /* erase option bytes */
    FLASH->CR |= FLASH_CR_OPTER;
    FLASH->CR |= FLASH_CR_STRT;
    status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
    FLASH->CR &= ~FLASH_CR_OPTER;
    
    /* set SEL = 0,nBoot1 = 1, nBoot0 = 0 */
    ob[1] = (ob[1] & ~0x9898) | 0x8810;
    FLASH->CR |= FLASH_CR_OPTPG;
      OB->RDP = ob[0];
      status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
      if(HAL_OK != status)
      {
        while(1);
      }
      FLASH->CR &= FLASH_CR_OPTPG;
      
      FLASH->CR |= FLASH_CR_OPTPG;
      OB->USER = ob[1];
      status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
      if(HAL_OK != status)
      {
        while(1);
      }
      FLASH->CR &= FLASH_CR_OPTPG;
      
      FLASH->CR |= FLASH_CR_OPTPG;
      OB->DATA0 = ob[2];
      status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
      if(HAL_OK != status)
      {
        while(1);
      }
      FLASH->CR &= FLASH_CR_OPTPG;
      
      FLASH->CR |= FLASH_CR_OPTPG;
      OB->DATA1 = ob[3];
      status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
      if(HAL_OK != status)
      {
        while(1);
      }
      FLASH->CR &= FLASH_CR_OPTPG;
      
      FLASH->CR |= FLASH_CR_OPTPG;
      OB->WRP0 = ob[4];
      status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
      if(HAL_OK != status)
      {
        while(1);
      }
      FLASH->CR &= FLASH_CR_OPTPG;
      
      FLASH->CR |= FLASH_CR_OPTPG;
      OB->WRP1 = ob[5];
      status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
      if(HAL_OK != status)
      {
        while(1);
      }
      FLASH->CR &= FLASH_CR_OPTPG;
      
      FLASH->CR |= FLASH_CR_OPTPG;
      OB->WRP2 = ob[6];
      status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
      if(HAL_OK != status)
      {
        while(1);
      }
      FLASH->CR &= FLASH_CR_OPTPG;
      
      FLASH->CR |= FLASH_CR_OPTPG;
      OB->WRP3 = ob[7];
      status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
      if(HAL_OK != status)
      {
        while(1);
      }
      FLASH->CR &= FLASH_CR_OPTPG;
    
  }
  else
  {
    
  }
  //	HAL_FLASH_OB_Lock();
  //	HAL_FLASH_Lock();

  HAL_FLASH_OB_Launch();
    
}

void BootConfig_User()
{
  HAL_StatusTypeDef status = HAL_OK;
  uint16_t ob[8];
  int i = 0;

  // save option bytes data
  for(i = 0;i<8;i++)
  {
    ob[i] = *(uint16_t*)(0x1ffff800+i*2);
  }

  // if not boot from main flash memory
  if((ob[1] & 0x0098) != 0x0018)
  {
    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();
    
    status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
    if(HAL_OK == status)
    {
      /* erase option bytes */
      FLASH->CR |= FLASH_CR_OPTER;
      FLASH->CR |= FLASH_CR_STRT;
      status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
      FLASH->CR &= ~FLASH_CR_OPTER;
      
      ob[1] = (ob[1] & ~0x9898) | 0x8018;
      
  //			for(i = 0; i < 8;i++)
  //			{
  //				FLASH->CR |= FLASH_CR_OPTPG;
  //				*((uint16_t*)(0x1fff800+i*2)) = ob[i];
  //				status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
  //				if(HAL_OK != status)
  //				{
  //					while(1);
  //				}
  //				FLASH->CR &= FLASH_CR_OPTPG;
  //			}
      
      FLASH->CR |= FLASH_CR_OPTPG;
      OB->RDP = ob[0];
      status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
      if(HAL_OK != status)
      {
        while(1);
      }
      FLASH->CR &= FLASH_CR_OPTPG;
      
      FLASH->CR |= FLASH_CR_OPTPG;
      OB->USER = ob[1];
      status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
      if(HAL_OK != status)
      {
        while(1);
      }
      FLASH->CR &= FLASH_CR_OPTPG;
      
      FLASH->CR |= FLASH_CR_OPTPG;
      OB->DATA0 = ob[2];
      status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
      if(HAL_OK != status)
      {
        while(1);
      }
      FLASH->CR &= FLASH_CR_OPTPG;
      
      FLASH->CR |= FLASH_CR_OPTPG;
      OB->DATA1 = ob[3];
      status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
      if(HAL_OK != status)
      {
        while(1);
      }
      FLASH->CR &= FLASH_CR_OPTPG;
      
      FLASH->CR |= FLASH_CR_OPTPG;
      OB->WRP0 = ob[4];
      status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
      if(HAL_OK != status)
      {
        while(1);
      }
      FLASH->CR &= FLASH_CR_OPTPG;
      
      FLASH->CR |= FLASH_CR_OPTPG;
      OB->WRP1 = ob[5];
      status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
      if(HAL_OK != status)
      {
        while(1);
      }
      FLASH->CR &= FLASH_CR_OPTPG;
      
      FLASH->CR |= FLASH_CR_OPTPG;
      OB->WRP2 = ob[6];
      status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
      if(HAL_OK != status)
      {
        while(1);
      }
      FLASH->CR &= FLASH_CR_OPTPG;
      
      FLASH->CR |= FLASH_CR_OPTPG;
      OB->WRP3 = ob[7];
      status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
      if(HAL_OK != status)
      {
        while(1);
      }
      FLASH->CR &= FLASH_CR_OPTPG;
      
    }
    else
    {
      
    }
  //		HAL_FLASH_OB_Lock();
  //		HAL_FLASH_Lock();
    
    HAL_FLASH_OB_Launch();
    
  }
}