#include "myconfig.h"

static void LED_ON(uint8_t LED);
static void LED_OFF(uint8_t LED);
static void LED_TOGGLE(uint8_t LED);

LED_t led = 
{
  LED_ON,
  LED_OFF,
  LED_TOGGLE,
};

static void LED_ON(uint8_t LED)
{
  switch(LED)
  {
    case LED1:HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);break;
    case LED2:HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);break;    
  }
  
}

static void LED_OFF(uint8_t LED)
{
  switch(LED)
  {
    case LED1:HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);break;
    case LED2:HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);break;    
  }  
  
}

static void LED_TOGGLE(uint8_t LED)
{
  switch(LED)
  {
    case LED1:HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);break;
    case LED2:HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);break;    
  }     
}

