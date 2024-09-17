/*
 * uart_driver.c
 *
 *  Created on: Sep 16, 2024
 *      Author: Furka
 */


//static int32_t UART_is_buffer_empty(volatile UART_Buffer_t* buffer);
/*

void UART_send_byte(uint8_t data)
{
	UART_BufferTX.buffer[UART_BufferTX.head_pointer++] = data;
	if(UART_BufferTX.head_pointer == BUFFER_SIZE)
	{
		UART_BufferTX.head_pointer = 0;
	}
   Enable the UART Transmit Data Register Empty Interrupt
 SET_BIT(USART2->CR1, USART_CR1_TXEIE);
}

int32_t UART_is_buffer_empty(volatile UART_Buffer_t* buffer)
{
	return (buffer->head_pointer == buffer->tail_pointer?1:0);

	if(buffer->head_pointer == buffer->tail_pointer)
	{
		return 1; // buffer is empty
	}
	else
	{
		 return 0;
	}

}

int32_t UART_read_byte()
{
	int kar =  0;

	if(UART_is_buffer_empty(&UART_BufferRX) == 1 )
	{
		kar = -1;
	}
	else
	{
		kar = UART_BufferRX.buffer[UART_BufferRX.tail_pointer++];

		if ( UART_BufferRX.tail_pointer == BUFFER_SIZE)
		{
			UART_BufferRX.tail_pointer = 0;
		}
	}

	return kar;
}



void UART_send_byte_array(uint8_t* buffer, uint32_t size)
{
	int i;

	for( i=0; i<size; i++)
	{
		UART_send_byte(buffer[i]);
	}
}

uint32_t UART_bytes_to_read(void)
{
	if (UART_BufferRX.head_pointer >= UART_BufferRX.tail_pointer)
	{
		return UART_BufferRX.head_pointer - UART_BufferRX.tail_pointer;
	}
	else
	{
		return (BUFFER_SIZE + UART_BufferRX.head_pointer - UART_BufferRX.tail_pointer);
	}
}


*/
