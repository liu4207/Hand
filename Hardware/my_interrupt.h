#ifndef __MY_INTERRUPT_H_
#define __MY_INTERRUPT_H_

#define uart_len 100




typedef struct
{
  uint8_t myaddr;
  uint8_t modbus_buf[uart_len];
  uint16_t modbus_count;
  uint8_t rx_flag;
  uint8_t timeout;
  uint8_t time_run;
  uint8_t send_buf[uart_len];
}MODBUS;


extern MODBUS modbus;



#endif

