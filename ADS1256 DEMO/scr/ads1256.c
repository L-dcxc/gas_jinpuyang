
#include "ADS1256.h"


	/*端口定义*/ 

	sbit SCK  = P1^5;
	sbit DIN  = P1^4;
	sbit DOUT = P1^3;
	sbit DRDY = P1^7;
	sbit CS   = P1^6;

	#define CS_0()		CS = 0
	#define CS_1()		CS = 1
	#define SCK_0()		SCK = 0
	#define SCK_1()		SCK = 1

	#define ADS1256_DIN_0()	DIN = 0
	#define ADS1256_DIN_1()	DIN = 1

	#define ADS1256_DRDY   DRDY
	#define ADS1256_DOUT   DOUT

void delay_ms(u16);

/*void Init_ADS1256_GPIO(void)
{
	********************************************
	 	提示：写单片机程序配置I/O口时要注意方向，
		I/O口配置输出时推荐配置为推挽模式，有些老
		的51单片机没有推挽模式，推荐加 1K上拉电阻.
 	********************************************
	P1M1 = 0;
	P1M0 =0x4C; //配置 I O
	CS_1();
}*/



static void ADS1256_DelaySCLK(void)
{
	unsigned short i;

	/*
		取 5 时，实测高电平200ns, 低电平250ns <-- 不稳定
		取 10 以上，可以正常工作， 低电平400ns 高定400ns <--- 稳定
	*/
	for (i = 0; i < 20; i++);
}
/*
*********************************************************************************************************
*	函 数 名: SPI_WriteByte
*	功能说明: 向SPI总线发送8个bit数据。 不带CS控制。
*	形    参: _data : 数据
*	返 回 值: 无
*********************************************************************************************************
*/
void SPI_WriteByte(unsigned char TxData)
{
	unsigned char i;
	/*　ADS1256 要求 SCL高电平和低电平持续时间最小 200ns  */
	for(i = 0; i < 8; i++)
	{
		if (TxData & 0x80)
			ADS1256_DIN_1();
		else
			ADS1256_DIN_0();
		
		SCK_1();
		//ADS1256_DelaySCLK();
		TxData <<= 1;
		SCK_0();			/* <----  ADS1256 是在SCK下降沿采样DIN数据, 数据必须维持 50nS */
		//ADS1256_DelaySCLK();
	}
	
} 

/*
*********************************************************************************************************
*	函 数 名: SPI_ReadByte
*	功能说明: 从SPI总线接收8个bit数据。 不带CS控制。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
unsigned char SPI_ReadByte(void)
{
	unsigned char i;
	unsigned char read = 0;

	//ADS1256_DelaySCLK();

	/*　ADS1256 要求 SCL高电平和低电平持续时间最小 200ns  */
	for (i = 0; i < 8; i++)
	{
		SCK_1();
		ADS1256_DelaySCLK();
		read = read<<1;
		SCK_0();
		if (ADS1256_DOUT)
		{
			read++;
		}
		ADS1256_DelaySCLK();
	}
	return read;
}

//-----------------------------------------------------------------//
//	功    能：ADS1256 写数据
//	入口参数: /
//	出口参数: /
//	全局变量: /
//	备    注: 向ADS1256中地址为regaddr的寄存器写入一个字节databyte
//-----------------------------------------------------------------//
void ADS1256WREG(unsigned char regaddr,unsigned char databyte)
{
    
	CS_0();
	while(ADS1256_DRDY);//当ADS1256_DRDY为低时才能写寄存器
	//向寄存器写入数据地址
    SPI_WriteByte(ADS1256_CMD_WREG | (regaddr & 0x0F));
    //写入数据的个数n-1
    SPI_WriteByte(0x00);
    //向regaddr地址指向的寄存器写入数据databyte
    SPI_WriteByte(databyte);
	CS_1();
}


//初始化ADS1256
void ADS1256_Init(void)
{

	//*************自校准****************
   	while(ADS1256_DRDY);
	CS_0();
	SPI_WriteByte(ADS1256_CMD_SELFCAL);
	while(ADS1256_DRDY);
	CS_1();
	//**********************************

	ADS1256WREG(ADS1256_STATUS,0x06);               // 高位在前、校准、使用缓冲
//	ADS1256WREG(ADS1256_STATUS,0x04);               // 高位在前、不使用缓冲
//	ADS1256WREG(ADS1256_MUX,0x08);                  // 初始化端口A0为‘+’，AINCOM位‘-’
	ADS1256WREG(ADS1256_ADCON,ADS1256_GAIN_1);      // 放大倍数1
	ADS1256WREG(ADS1256_DRATE,ADS1256_DRATE_10SPS); // 数据5sps
	ADS1256WREG(ADS1256_IO,0x00);  
	             
	//*************自校准****************
	while(ADS1256_DRDY);
	CS_0();
	SPI_WriteByte(ADS1256_CMD_SELFCAL);
	while(ADS1256_DRDY);
	CS_1(); 
	//**********************************

}


//读取AD值
unsigned long ADS1256ReadData(unsigned char channel)  
{

    unsigned long sum=0;
	char i;
	unsigned long r=0;
	
	while(ADS1256_DRDY);//当ADS1256_DRDY为低时才能写寄存器 
	ADS1256WREG(ADS1256_MUX,channel);		//设置通道
	CS_0();
	SPI_WriteByte(ADS1256_CMD_SYNC);//外同步AD信号，启动转换
	SPI_WriteByte(ADS1256_CMD_WAKEUP);//完成同步并退出待机模式	  
	while(ADS1256_DRDY);	
	SPI_WriteByte(ADS1256_CMD_RDATA);
	delay_nms(1);
	
	for(i=0;i<3;i++)
    {
    	sum = sum << 8;
		r = SPI_ReadByte();  
	  	sum |= r;
	}	

	CS_1();

	if (sum>0x7FFFFF)           // if MSB=1, 
	{
		sum -= 0x1000000;       // do 2's complement
	}

    return sum;
}




