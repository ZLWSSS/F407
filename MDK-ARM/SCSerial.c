#include "UART_USART_FUN.h"

uint32_t IOTimeOut = 100;//���������ʱ
uint8_t wBuf[128];
uint8_t wLen = 0;

//UART �������ݽӿ�
int readSCS(unsigned char *nDat, int nLen)
{
	return Uart_Read(nDat, nLen, IOTimeOut);
}

//UART �������ݽӿ�
int writeSCS(unsigned char *nDat, int nLen)
{
	while(nLen--){
		if(wLen<sizeof(wBuf)){
			wBuf[wLen] = *nDat;
			wLen++;
			nDat++;
		}
	}
	return wLen;
}

//���ջ�����ˢ��
void rFlushSCS()
{
}

//���ͻ�����ˢ��
void wFlushSCS()
{
	if(wLen){
		Uart_Send(wBuf, wLen);
		wLen = 0;
	}
}

