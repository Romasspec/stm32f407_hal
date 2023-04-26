#ifndef drv_serial_h
#define drv_serial_h

typedef struct	serialPort {
	uint32_t	rx_bufferSize;
	uint32_t	tx_bufferSize;
	uint32_t rxBufferHead;
    uint32_t rxBufferTail;
    uint32_t txBufferHead;
    uint32_t txBufferTail;
	volatile uint8_t *rxBuffer;
	volatile uint8_t *txBuffer;
}serialPort_t;

#define UART4_RX_BUFFER_SIZE    1024
#define UART4_TX_BUFFER_SIZE    256

void serialInit (serialPort_t* serial);
uint8_t serialTotalBytesWaiting(serialPort_t *serial);
uint8_t serialRead(serialPort_t *serial);

#endif
