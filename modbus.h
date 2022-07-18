#ifndef MODBUS_H_
#define MODBUS_H_

//#include "main.h"
#include "usart.h"
#include "stm32f1xx_hal.h"
//---------------user define------------------------------------
UART_HandleTypeDef *UARTx;
TIM_HandleTypeDef *TIMx;
//timer 0.0001 sec setup APB=36Mhz/(36*100) div 36 counter period 100
//------------------------------------------------------------------
#define OBJ_SZ 50
#define SETUP 4 //setup elements
#define MAX_HANDLER_CHANGE_REGISTER (uint8_t) 50 // max registers handler
#define CIRCULAR_BUFFER_QUEUE_REGISTERS 20
//buffer uart
#define BUF_SZ OBJ_SZ * 2
#define MODBUS_WRD_SZ (BUF_SZ-5)/2 //max quantity of words in responce

//uart structure
typedef struct {
uint8_t buffer[BUF_SZ];
uint32_t rxtimer;
uint32_t rxcnt;
uint32_t txcnt;
uint32_t txlen;
uint8_t rxgap;
uint32_t delay;
} UART_DATA;

uint8_t install_register(uint8_t reg_number, uint8_t number_of_regs, void (*handler)(uint16_t *Array_regs, uint8_t *index, uint8_t *r_length));
//------------------------------------
void ModBus_init(TIM_HandleTypeDef *my_TIM,UART_HandleTypeDef *my_UART, uint32_t delay_pack, uint8_t devId);
void set_devID(uint8_t devId);
void set_register_high_low(uint8_t value_high, uint8_t value_low, uint16_t index);
void set_register(uint16_t value, uint16_t index);
uint16_t get_register(uint16_t index);
void UART_IRQ_ProceedModBus();
void TIM_IRQ_ProceedModBus();
void proceedModBus();
//------------------------------------
void MODBUS_SLAVE(UART_DATA *MODBUS);

#endif /* MODBUS_H_ */
