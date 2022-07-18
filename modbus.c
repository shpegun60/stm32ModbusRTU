#include "modbus.h"

uint8_t Circular_buffer[CIRCULAR_BUFFER_QUEUE_REGISTERS]={0};
uint8_t queue_counter=0;

UART_DATA m_data;


uint8_t SET_PAR[SETUP];


uint16_t res_table[OBJ_SZ];


unsigned int Crc16(unsigned char *ptrByte, int byte_cnt);
void TX_03_04(UART_DATA *MODBUS);
void TX_06(UART_DATA *MODBUS);
void TX_16(UART_DATA *MODBUS);
void TX_EXCEPTION(UART_DATA *MODBUS,unsigned char error_type);


void proceed_registers(uint8_t reg_index);


uint8_t pop_queue();
void push_queue(uint8_t reg_index);



void ModBus_init(TIM_HandleTypeDef *my_TIM,UART_HandleTypeDef *my_UART, uint32_t delay_pack, uint8_t devId)
{
	if(my_UART==0 || my_TIM==0) return;
	UARTx=my_UART;
	TIMx=my_TIM;
	m_data.delay=delay_pack;
	SET_PAR[0]=devId;
	__HAL_UART_ENABLE_IT(UARTx, UART_IT_RXNE);
	__HAL_UART_DISABLE_IT(UARTx, UART_IT_TC);
	USART_DR_GPIO_Port->BSRR = (uint32_t)USART_DR_Pin << 16u;

	HAL_TIM_Base_Start (my_TIM);
	HAL_TIM_Base_Start_IT (my_TIM);


}

void set_devID(uint8_t devId) {SET_PAR[0]=devId;}
uint16_t get_register(uint16_t index)
{
	if(index>OBJ_SZ) return 0;
	uint16_t tmp=res_table[index];
	return tmp;
}
void set_register(uint16_t value, uint16_t index)
{
	if(index>OBJ_SZ) return;
	res_table[index]=value;
}
void set_register_high_low(uint8_t value_high, uint8_t value_low, uint16_t index)
{
	if(index>OBJ_SZ) return;
	res_table[index]=(uint16_t)((value_high<<8) + value_low);
}


void UART_IRQ_ProceedModBus()
{

	if (UARTx->Instance->SR & USART_SR_RXNE)
	{
		UARTx->Instance->SR &= ~(0x01<<USART_SR_RXNE_Pos);
		m_data.rxtimer=0;
		if(m_data.rxcnt>(BUF_SZ-2)) m_data.rxcnt=0;
		m_data.buffer[m_data.rxcnt++] = (uint8_t)(UARTx->Instance->DR & (uint8_t)0x00FF);
    }

	else if (UARTx->Instance->SR & USART_SR_TC)
	{
		if(m_data.txcnt < m_data.txlen) {UARTx->Instance->DR = (uint8_t)(m_data.buffer[m_data.txcnt++] & (uint8_t)0xFF);}
		else
		{
			m_data.txlen=0;
			USART_DR_GPIO_Port->BSRR = (uint32_t)USART_DR_Pin << 16u;
			__HAL_UART_ENABLE_IT(UARTx, UART_IT_RXNE);
			__HAL_UART_DISABLE_IT(UARTx, UART_IT_TC);
		}
	}
}


void TIM_IRQ_ProceedModBus()
{
	TIMx->Instance->SR &= ~TIM_SR_UIF;
	if((m_data.rxtimer++>m_data.delay) & (m_data.rxcnt>1)) {
		m_data.rxgap=1;
	}
	else {m_data.rxgap=0;}
}

void proceedModBus()
{
	if(m_data.rxgap==1)
	{
		MODBUS_SLAVE(&m_data);
		if((m_data.txlen>0)&(m_data.txcnt==0))
		{
			 USART_DR_GPIO_Port->BSRR = USART_DR_Pin;
			 UARTx->Instance->SR &= ~(0x01<<USART_SR_TC_Pos);
			 __HAL_UART_ENABLE_IT(UARTx, UART_IT_TC);
			 __HAL_UART_DISABLE_IT(UARTx, UART_IT_RXNE);
			 UARTx->Instance->DR = (uint8_t)(m_data.buffer[m_data.txcnt++] & (uint8_t)0xFF);
		}
	}
	if(queue_counter!=0) proceed_registers(pop_queue());
}






 void MODBUS_SLAVE(UART_DATA *MODBUS)
{
	 unsigned int tmp;


	if((MODBUS->buffer[0]!=0)&(MODBUS->rxcnt>5) & ((MODBUS->buffer[0]==SET_PAR[0])|(MODBUS->buffer[0]==255)))
	{
		tmp=Crc16(MODBUS->buffer,MODBUS->rxcnt-2);
		if((MODBUS->buffer[MODBUS->rxcnt-2]==(tmp&0x00FF)) & (MODBUS->buffer[MODBUS->rxcnt-1]==(tmp>>8)))
		{

			switch(MODBUS->buffer[1])
			{
			case 3:
				TX_03_04(MODBUS);
				break;
			case 4:
				TX_03_04(MODBUS);
				break;
			case 6:
				TX_06(MODBUS);
				break;
			case 16:
				TX_16(MODBUS);
				break;
			default:

				TX_EXCEPTION(MODBUS,0x01);
			}

			 tmp=Crc16(MODBUS->buffer,MODBUS->txlen-2);
			 MODBUS->buffer[MODBUS->txlen-2]=tmp;
			 MODBUS->buffer[MODBUS->txlen-1]=tmp>>8;
			 MODBUS->txcnt=0;
		}
	}
	MODBUS->rxgap=0;
	MODBUS->rxcnt=0;
	MODBUS->rxtimer=0xFFFF;
}




void TX_03_04(UART_DATA *MODBUS)
{
	uint16_t tmp_start_addr,tmp_number_of_regs;
	uint16_t m=0,n=0;
	uint16_t tmp_val;






	tmp_start_addr=((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]);


	tmp_number_of_regs=((MODBUS->buffer[4]<<8)+MODBUS->buffer[5]);


	n=3;

	if((((tmp_start_addr+tmp_number_of_regs)<OBJ_SZ)&(tmp_number_of_regs<MODBUS_WRD_SZ+1)))
	{
	   for(m=0; m<tmp_number_of_regs; m++)
	   {
		   tmp_val=res_table[m+tmp_start_addr];
		   MODBUS->buffer[n]=tmp_val>>8;
		   MODBUS->buffer[n+1]=tmp_val;
		   n=n+2;
	   }
	   MODBUS->buffer[2]=m*2;
	   MODBUS->txlen=m*2+5;
	}
	else
	{

	   TX_EXCEPTION(MODBUS,0x02);
	}
}



void TX_06(UART_DATA *MODBUS)
{
	uint16_t tmp_addr;



  
   

	tmp_addr=((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]);
	push_queue((uint8_t)tmp_addr);


   if(tmp_addr<OBJ_SZ)
   {
	   MODBUS->txlen=MODBUS->rxcnt;
	   res_table[tmp_addr]=(MODBUS->buffer[4]<<8)+MODBUS->buffer[5];
   }
   else
   {

	   TX_EXCEPTION(MODBUS,0x02);
   }
}




void TX_16(UART_DATA *MODBUS)
{
	uint16_t tmp_addr;
	uint16_t tmp_number_of_reg;
	uint16_t tmp_counter;





	tmp_addr=((MODBUS->buffer[2]<<8)+MODBUS->buffer[3]);
	tmp_number_of_reg=((MODBUS->buffer[4]<<8)+MODBUS->buffer[5]);
	tmp_counter=7;
	MODBUS->txlen=tmp_counter+1;

   if((((tmp_addr+tmp_number_of_reg)<OBJ_SZ)&(tmp_number_of_reg<MODBUS_WRD_SZ+1)))
   {
	   for(uint16_t i=0;i<tmp_number_of_reg;i++)
	   {
		   res_table[tmp_addr+i]=(MODBUS->buffer[tmp_counter]<<8)+MODBUS->buffer[tmp_counter+1];
		   push_queue((uint8_t)tmp_addr+i);
		   tmp_counter+=2;
	   }
   }
   else
   {

	   TX_EXCEPTION(MODBUS,0x02);
   }
}





void TX_EXCEPTION(UART_DATA *MODBUS,unsigned char error_type)
{

	  MODBUS->buffer[2]=error_type;
	  MODBUS->txlen=5;
}




unsigned int Crc16(unsigned char *ptrByte, int byte_cnt)
{
	unsigned int w=0;
	char shift_cnt;

	if(ptrByte)
	{
		w=0xffffU;
		for(; byte_cnt>0; byte_cnt--)
		{
			w=(unsigned int)((w/256U)*256U+((w%256U)^(*ptrByte++)));
			for(shift_cnt=0; shift_cnt<8; shift_cnt++)
			{
				 if((w&0x1)==1)
				 w=(unsigned int)((w>>1)^0xa001U);
				 else
				 w>>=1;
			}
		}
	}
	return w;
}




uint8_t handler_cnt=0;
uint8_t number_of_register=0;
typedef struct {
	uint8_t regAddr;
	uint8_t length;
	void (*handler)(uint16_t *Array_regs, uint8_t *reg_index, uint8_t *r_length);
} ModBus_RegChangedHandler;

ModBus_RegChangedHandler Registers_handler[MAX_HANDLER_CHANGE_REGISTER];

uint8_t install_register(uint8_t reg_number, uint8_t number_of_regs, void (*handler)(uint16_t *Array_regs, uint8_t *reg_index, uint8_t *r_length))
{
	if((number_of_register > OBJ_SZ) || (handler_cnt > MAX_HANDLER_CHANGE_REGISTER) || (number_of_regs > 4)) return 0;
	for (uint8_t i=0; i < handler_cnt; i++)
	{
		if (Registers_handler[i].regAddr == reg_number)
		{
			Registers_handler[i].handler = handler;
			Registers_handler[i].length = number_of_regs;
			return 0x01;
		}
	}
	Registers_handler[handler_cnt].regAddr = reg_number;
	Registers_handler[handler_cnt].length = number_of_regs;
	Registers_handler[handler_cnt].handler= handler;
	handler_cnt += 1;
	number_of_register += number_of_regs;
	return 0x01;
}
void proceed_registers(uint8_t reg_index)
{
	if(reg_index > OBJ_SZ) return;
	for (uint8_t i=0; i < handler_cnt; i++)
	{
		if (Registers_handler[i].regAddr == reg_index)
		{
			Registers_handler[i].handler(res_table,&(Registers_handler[i].regAddr),&(Registers_handler[i].length));
		}
	}
}




void push_queue(uint8_t reg_index)
{
	if(queue_counter>CIRCULAR_BUFFER_QUEUE_REGISTERS-1) return;
	Circular_buffer[queue_counter]=reg_index;
	queue_counter++;
}
uint8_t pop_queue()
{
	uint8_t tmp_val=Circular_buffer[0];
	for(uint8_t i=0;i<queue_counter-1;i++) Circular_buffer[i]=Circular_buffer[i+1];
	queue_counter--;
	return tmp_val;
}
