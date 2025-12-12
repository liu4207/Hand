#ifndef __MY_SYSTEM_H_
#define __MY_SYSTEM_H_

typedef struct
{
  void (*Per_Init)(void);
  void (*System_Run)(void);
}System_t;

extern System_t system;
extern uint8_t ch;

#endif

