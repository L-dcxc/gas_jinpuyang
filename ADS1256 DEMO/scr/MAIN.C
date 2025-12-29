/*                        -- 渡河蚂蚁电子工作室 --                        */
/*
*   说    明: STC8A8KS4A12 DEMO程序
*   开发平台: STC15W408S     
*   淘宝网店: 
*
//  文件名：main.c                                                              
//  说明：供客户测试模块通信使用程序                                                                  
//  编写人员：Duhemayi                                                                   
//  编写日期：2018-09-16                                                               
//  程序维护：
//  维护记录：
//	版    本: V1.0
//                                                          
// 免责声明：该程序仅用于学习与交流 
// (c) Duhemayi Corporation. All rights reserved.     
******************************************************************************/
#include "config.h"



void MAIN_CLK_Config(void)
{
	#define CKSEL           (*(unsigned char volatile xdata *)0xfe00)
	#define CKDIV           (*(unsigned char volatile xdata *)0xfe01)
	#define IRC24MCR        (*(unsigned char volatile xdata *)0xfe02)
	#define XOSCCR          (*(unsigned char volatile xdata *)0xfe03)
	#define IRC32KCR        (*(unsigned char volatile xdata *)0xfe04)

	P_SW2 = 0x80;								//访问外设寄存器之前，先将P_SW2 BIT7置1
    CKSEL = 0x00;                               //选择主时钟源内部IRC ( 默认 )
    P_SW2 = 0x00;								//访问完外设寄存器之后，再将P_SW2 BIT7置0

    
//    P_SW2 = 0x80;
//    XOSCCR = 0xc0;                              //启动外部晶振
//    while (!(XOSCCR & 1));                      //等待时钟稳定
//    CKDIV = 0x00;                               //时钟不分频
//    CKSEL = 0x01;                               //选择外部晶振
//    P_SW2 = 0x00;
    

    /*
    P_SW2 = 0x80;
    IRC32KCR = 0x80;                            //启动内部32K IRC
    while (!(IRC32KCR & 1));                    //等待时钟稳定
    CKDIV = 0x00;                               //时钟不分频
    CKSEL = 0x03;                               //选择内部32K
    P_SW2 = 0x00;
    */
}

/******************************************************************************/
// 函数名称：TestMode1(void) 
// 输入参数： 
// 输出参数： 
// 函数功能： 测试单通道方式
/******************************************************************************/
void TestMode1(void)
{
	unsigned long Adc;
	uint32 dianya=0;
	
	Adc = ADS1256ReadData( ADS1256_MUXP_AIN0|ADS1256_MUXN_AINCOM);//选择AIN0--AINCOM
//	Adc = ADS1256ReadData( ADS1256_MUXP_AIN1|ADS1256_MUXN_AINCOM);//选择AIN1--AINCOM
//	Adc = ADS1256ReadData( ADS1256_MUXP_AIN2|ADS1256_MUXN_AINCOM);//选择AIN2--AINCOM
//	Adc = ADS1256ReadData( ADS1256_MUXP_AIN3|ADS1256_MUXN_AINCOM);//选择AIN3--AINCOM
//	Adc = ADS1256ReadData( ADS1256_MUXP_AIN4|ADS1256_MUXN_AINCOM);//选择AIN4--AINCOM
//	Adc = ADS1256ReadData( ADS1256_MUXP_AIN5|ADS1256_MUXN_AINCOM);//选择AIN5--AINCOM
//	Adc = ADS1256ReadData( ADS1256_MUXP_AIN6|ADS1256_MUXN_AINCOM);//选择AIN6--AINCOM
//	Adc = ADS1256ReadData( ADS1256_MUXP_AIN7|ADS1256_MUXN_AINCOM);//选择AIN7--AINCOM
	Uart_send_hex_to_txt(Adc);
	Uart_send_hex_to_txt(Adc>>8);
	Uart_send_hex_to_txt(Adc>>16);
	dianya = Adc*250/4194304*10;
	UART_Send_dat(dianya);
	UartSend(0x6d);
	UartSend(0x56);
	UartSend(' ');
	UartSend('0');
	UartSend('\r');
	UartSend('\n');
	delay_nms(100);
	
	Adc = ADS1256ReadData( ADS1256_MUXP_AIN1|ADS1256_MUXN_AINCOM);//选择AIN1--AINCOM
	Uart_send_hex_to_txt(Adc);
	Uart_send_hex_to_txt(Adc>>8);
	Uart_send_hex_to_txt(Adc>>16);
	dianya = Adc*250/4194304*10;
	UART_Send_dat(dianya);
	UartSend(0x6d);
	UartSend(0x56);
	UartSend(' ');
	UartSend('1');
	UartSend('\r');
	UartSend('\n');
	delay_nms(100);
	
	Adc = ADS1256ReadData( ADS1256_MUXP_AIN2|ADS1256_MUXN_AINCOM);//选择AIN2--AINCOM
	Uart_send_hex_to_txt(Adc);
	Uart_send_hex_to_txt(Adc>>8);
	Uart_send_hex_to_txt(Adc>>16);
	dianya = Adc*250/4194304*10;
	UART_Send_dat(dianya);
	UartSend(0x6d);
	UartSend(0x56);
	UartSend(' ');
	UartSend('2');
	UartSend('\r');
	UartSend('\n');
	delay_nms(100);
	
	Adc = ADS1256ReadData( ADS1256_MUXP_AIN3|ADS1256_MUXN_AINCOM);//选择AIN3--AINCOM
	Uart_send_hex_to_txt(Adc);
	Uart_send_hex_to_txt(Adc>>8);
	Uart_send_hex_to_txt(Adc>>16);
	dianya = Adc*250/4194304*10;
	UART_Send_dat(dianya);
	UartSend(0x6d);
	UartSend(0x56);
	UartSend(' ');
	UartSend('3');
	UartSend('\r');
	UartSend('\n');
	delay_nms(100);
	
}

/******************************************************************************/
// 函数名称：TestMode1(void) 
// 输入参数： 
// 输出参数： 
// 函数功能： 测试差分模式
/******************************************************************************/
void TestMode2(void)
{
	unsigned long Adc;
	uint32 dianya=0;
	
	Adc = ADS1256ReadData(ADS1256_MUXP_AIN0|ADS1256_MUXP_AIN1);//选择AIN0--AINCOM
//	Adc = ADS1256ReadData(ADS1256_MUXP_AIN2|ADS1256_MUXP_AIN3);//选择AIN0--AINCOM
//	Adc = ADS1256ReadData(ADS1256_MUXP_AIN4|ADS1256_MUXP_AIN5);//选择AIN0--AINCOM
//	Adc = ADS1256ReadData(ADS1256_MUXP_AIN6|ADS1256_MUXP_AIN7);//选择AIN0--AINCOM
	Uart_send_hex_to_txt(Adc);
	Uart_send_hex_to_txt(Adc>>8);
	Uart_send_hex_to_txt(Adc>>16);
	dianya = Adc*250/4194304*10;
	UART_Send_dat(dianya);
	UartSend(0x6d);
	UartSend(0x56);
	UartSend('\r');
	UartSend('\n');
	
}

/******************************************************************************/
// 函数名称：main 
// 输入参数： 
// 输出参数： 
// 函数功能： 打开定时器1，开定时器1中断，定时1MS，100MS LED闪烁一次
/******************************************************************************/
void main(void)
{
	uint16 ch=0;
//	unsigned long Adc;
//	float  Volts;
//	char str[20];
	
	MAIN_CLK_Config();	//设置主时钟
	Uart1_Init();       //串口初始化
//	Init_ADS1256_GPIO();//ADS1256 IO初始化
	Delay100ms();
	
	ADS1256_Init();
	while(1)
	{
		TestMode1();
				
		Delay100ms();
	}
}

