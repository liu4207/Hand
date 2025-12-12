#include "myconfig.h"

//长按短按持续时间
#define SHORT_PRESS_TIM 100
#define LONG_PRESS_TIM  500

//1 按键初始化
void Key_Init(Key_Handle* key,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
  key->GPIOx = GPIOx;
  key->GPIO_Pin = GPIO_Pin;
  key->isPressed = 0;
  key->press_tick = 0;
  key->Short_cb = NULL;
  key->Long_cb = NULL;
}
//2 按键回调函数
void Key_CallBack(Key_Handle* key,CallBack Short,CallBack Long)
{
  key->Short_cb = Short;
  key->Long_cb = Long;
}
//3 按键定时器扫描
void Key_Scan(Key_Handle* key)
{
  uint16_t press_state = HAL_GPIO_ReadPin(key->GPIOx,key->GPIO_Pin); //获取按键状态
  static uint8_t cnt = 0;
  if(press_state == GPIO_PIN_RESET) //按键按下
  {
    if(cnt < 5)
    {
      cnt ++;
      if(cnt == 5)
      {
        key->isPressed = 1;
        key->press_tick = HAL_GetTick();        
      }      
    }    
  }
  else //按键松开
  {
    if(key->isPressed == 1)//判断按键松开前是否按下
    {
      uint32_t press_duration = HAL_GetTick() - key->press_tick; //按键按下持续时间
      if(press_duration > LONG_PRESS_TIM) //长按
      {
        if(key->Long_cb != NULL)
        {
          key->Long_cb();
        } 
      }
      else if(press_duration > SHORT_PRESS_TIM) //短按
      {
        if(key->Short_cb != NULL)
        {
          key->Short_cb();
        }
      }
      key->isPressed = 0;
    }
    cnt = 0;
  }  
}
