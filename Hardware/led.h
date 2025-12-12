#ifndef __LED_H_
#define __LED_H_

typedef enum
{
  LED1 = (uint8_t)0x01,
  LED2 = (uint8_t)0x02,
}LED_NUM;


typedef struct
{
  void (*LED_ON)(uint8_t);
  void (*LED_OFF)(uint8_t);
  void (*LED_TOGGLE)(uint8_t);
}LED_t;

extern LED_t led;

#endif

