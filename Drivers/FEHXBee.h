#ifndef FEHXBEE_H
#define FEHXBEE_H
#include "derivative.h"

void UART0_RX_TX_IRQHandler();

class FEHXBee
{
public:
	FEHXBee();

	bool Initialize();
	bool IsInitialized();

	unsigned int ReceiveData( char* data, unsigned int maxlength );
	unsigned int ReceiveDataSearch( char* data, unsigned int maxlength, unsigned char expectedfirstchar );
	unsigned int SendData( char* data, unsigned int length );

	void SetPacketCallBack( void (*packetcallbackfunction)( volatile unsigned char* data, unsigned char length ) );

//	void EnableInterrupt();
//	void DisableInterrupt();

private:
	bool _initialized;
};

#endif // FEHXBEE_H
