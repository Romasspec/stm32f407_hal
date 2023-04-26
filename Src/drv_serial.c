#include "stm32f4xx_hal.h"
#include "drv_serial.h"


void serialInit (serialPort_t *serial)
{
	static uint8_t rxBuffer[UART4_RX_BUFFER_SIZE];
	static uint8_t txBuffer[UART4_TX_BUFFER_SIZE];
	
	serial->rx_bufferSize = UART4_RX_BUFFER_SIZE;
	serial->tx_bufferSize = UART4_TX_BUFFER_SIZE;
	serial->rxBuffer = rxBuffer;
	serial->txBuffer = txBuffer;
	serial->rxBufferHead = serial->rxBufferTail = 0;
	serial->txBufferHead = serial->txBufferTail = 0;
}

uint8_t serialTotalBytesWaiting(serialPort_t *serial)
{
    return serial->rxBufferTail != serial->rxBufferHead;
}

uint8_t serialRead(serialPort_t *serial)
{
	uint8_t readData;
	readData = serial->rxBuffer[serial->rxBufferTail];
	serial->rxBufferTail = (serial->rxBufferTail + 1) % serial->rx_bufferSize;	
    return readData;
}
