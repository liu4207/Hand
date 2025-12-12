#ifndef __KEY_H_
#define __KEY_H_

typedef void (*CallBack)(void);

typedef struct
{
  GPIO_TypeDef *GPIOx;
  uint16_t GPIO_Pin;
  uint8_t isPressed; //按键是否按下标志
  uint32_t press_tick; //获取时间
  
  CallBack Short_cb;
  CallBack Long_cb;
  
}Key_Handle;

//1 按键初始化
void Key_Init(Key_Handle* key,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);
//2 按键回调函数
void Key_CallBack(Key_Handle* key,CallBack Short,CallBack Long);
//3 按键定时器扫描
void Key_Scan(Key_Handle* key);

#endif

