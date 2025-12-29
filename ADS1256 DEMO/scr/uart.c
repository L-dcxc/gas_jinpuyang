#include "uart.h"

bit busy;


void Uart1_Init()
{
    SCON = 0x50;
    TMOD = 0x20;
    TL1 = BRT;
    TH1 = BRT;
    TR1 = 1;
    AUXR = 0x40;
    busy = 0;

	ES = 1;
	EA = 1;
}

void UartIsr() interrupt 4 using 1
{
    if (TI)
    {
        TI = 0;
        busy = 0;
    }
    if (RI)
    {
        RI = 0;
    }
}

void UartSend(char dat)
{
    while (busy);
    busy = 1;
    SBUF = dat;
}

void UartSendStr(char *p)
{
    while (*p)
    {
        UartSend(*p++);
    }
}

void Uart_send_hex_to_txt(uint8 dat)
{
	uint8 H_val,L_val;

	H_val = dat/16;
	if(H_val >= 10)
		 UartSend(H_val+0x37);
	else
		UartSend(H_val+0x30);

	L_val = dat%16;
	if(L_val >= 10)
		UartSend(L_val+0x37);
	else 
		UartSend(L_val+0x30);

	UartSend(' ');
}

void UART_Send_dat(uint32 dat)
{
	uint8 val[4];

	val[0] = dat%10000/1000;
	val[1] = dat%1000/100;
	val[2] = dat%100/10;
	val[3] = dat%10;

	UartSend(val[0]+'0');
	UartSend(val[1]+'0');
	UartSend(val[2]+'0');
	UartSend(val[3]+'0');
}

