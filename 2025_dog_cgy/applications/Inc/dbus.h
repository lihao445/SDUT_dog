#ifndef _DBUS_H
#define _DBUS_H
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart3

#include <main.h>



typedef struct
{
  int16_t ch0;
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t roll;
  uint8_t sw1;
  uint8_t sw2;
} rc_info_t;
 
#define rc_Init   \
{                 \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
}

extern rc_info_t rc;
//static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);
void dbus_uart_init(void);
//void rc_callback_handler(rc_info_t *rc, uint8_t *buff);
//uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream);
void uart_receive_handler(UART_HandleTypeDef *huart);

#endif