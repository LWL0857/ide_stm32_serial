#include "../Inc/mbotLinuxUsart.h"

#include "usart.h"         //包含printf

/*--------------------------------发送协议-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
//注意：这里数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/

/*--------------------------------接收协议-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
//注意：这里数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/
/**************************************************************************
通信的发送函数和接收函数必须的一些常量、变量、共用体对象
**************************************************************************/

//数据接收暂存区

unsigned char  receiveBuff[RX_LEN] = {0};
//通信协议常量
const unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};

//发送数据（飞机目前的位姿给树莓派）共用体（-32767 - +32768）
unsigned char buf[62] = {0};//待修改
union sendData
{
	double d;
	unsigned char data[8];
}positionX_stm32,positionY_stm32,positionZ_stm32,
orientationW_stm32,orientationX_stm32,
orientationY_stm32,orientationZ_stm32;

//从树莓派接收到的动捕发布的位姿 共用体
union receiveData
{
	double d;
	unsigned char data[8];
}positionX_rec_mocap,positionY_rec_mocap,positionZ_rec_mocap,
orientationX_rec_mocap,orientationY_rec_mocap,
orientationZ_rec_mocap,orientationW_rec_mocap;


/**************************************************************************
函数功能：通过串口中断服务函数，获取上位机发送的mocap中无人机的位姿，分别存入参数中
入口参数：pose from mocap
返回  值：无特殊意义
**************************************************************************/
int usartReceiveOneData(int *p_positionX_rec_mocap, int *p_positionY_rec_mocap, int *p_positionZ_rec_mocap,
						int *p_orientationX_rec_mocap, int *p_orientationY_rec_mocap,
						int *p_orientationZ_rec_mocap, int *p_orientationW_rec_mocap)
{
	unsigned char USART_Receiver              = 0;          //接收数据
	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //一帧数据传送开始标志位
	static short dataLength                   = 0;

	USART_Receiver = LL_USART_ReceiveData8(USART1);   //@@@@@#####如果你使用不是USART1更改成相应的，比如USART3
	//接收消息头
	if(Start_Flag == START)
	{
		if(USART_Receiver == 0xaa)                             //buf[1]
		{
			if(USARTReceiverFront == 0x55)        //数据头两位 //buf[0]
			{
				Start_Flag = !START;              //收到数据头，开始接收数据
				//printf("header ok\n");
				receiveBuff[0]=header[0];         //buf[0]
				receiveBuff[1]=header[1];         //buf[1]
				USARTBufferIndex = 0;             //缓冲区初始化
				checkSum = 0x00;				  //校验和初始化
			}
		}
		else
		{
			USARTReceiverFront = USART_Receiver;
		}
	}
	else
    {
		switch(USARTBufferIndex)
		{
			case 0://接收动捕发送的位姿数据的长度
				receiveBuff[2] = USART_Receiver;
				dataLength     = receiveBuff[2];            //buf[2]
				USARTBufferIndex++;
				break;
			case 1://接收所有数据，并赋值处理
				receiveBuff[j + 3] = USART_Receiver;        //buf[3] - buf[7]
				j++;
				if(j >= dataLength)
				{
					j = 0;
					USARTBufferIndex++;
				}
				break;
			case 2://接收校验值信息
				receiveBuff[3 + dataLength] = USART_Receiver;
				checkSum = getCrc8(receiveBuff, 3 + dataLength);
				  // 检查信息校验值
				if (checkSum != receiveBuff[3 + dataLength]) //buf[8]
				{
					//printf("Received data check sum error!");
					return 0;
				}
				USARTBufferIndex++;
				break;

			case 3://接收信息尾
				if(k==0)
				{
					//数据0d     buf[60]  无需判断
					k++;
				}
				else if (k==1)
				{
					//数据0a     buf[61] 无需判断
					memcpy(&positionX_rec_mocap.data, &receiveBuff[3], 8);
					memcpy(&positionY_rec_mocap.data, &receiveBuff[11], 8);
					memcpy(&positionZ_rec_mocap.data, &receiveBuff[19], 8);
					memcpy(&orientationX_rec_mocap.data, &receiveBuff[27], 8);
					memcpy(&orientationY_rec_mocap.data, &receiveBuff[35], 8);
					memcpy(&orientationZ_rec_mocap.data, &receiveBuff[43], 8);
					memcpy(&orientationW_rec_mocap.data, &receiveBuff[51], 8);
					/*//进行速度赋值操作
					 for(k = 0; k < 7; k++)
					{
						positionX_rec_mocap.data[k]  = receiveBuff[k + 3]; //buf[3]  buf[4]
						positionY_rec_mocap.data[k] = receiveBuff[k + 11]; //buf[5]  buf[6]
						positionZ_rec_mocap.data[k] = receiveBuff[k + 19];
						orientationX_rec_mocap.data[k] = receiveBuff[k + 11];
						orientationY_rec_mocap.data[k] = receiveBuff[k + 11];
						orientationZ_rec_mocap.data[k] = receiveBuff[k + 11];
						orientationW_rec_mocap.data[k] = receiveBuff[k + 11];

					}*/

					//位姿赋值操作
					*p_positionX_rec_mocap = (int)positionX_rec_mocap.d;
					*p_positionY_rec_mocap = (int)positionY_rec_mocap.d;
					*p_positionZ_rec_mocap = (int)positionZ_rec_mocap.d;
					*p_orientationX_rec_mocap = (int)orientationX_rec_mocap.d;
					*p_orientationY_rec_mocap = (int)orientationY_rec_mocap.d;
					*p_orientationZ_rec_mocap = (int)orientationZ_rec_mocap.d;
					*p_orientationW_rec_mocap = (int)orientationW_rec_mocap.d;

					//-----------------------------------------------------------------
					//完成一个数据包的接收，相关变量清零，等待下一字节数据
					USARTBufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					//-----------------------------------------------------------------
				}
				break;
			 default:break;
		}
	}
	return 0;
}
/**************************************************************************
函数功能：将无人机的位姿进行打包，通过串口发送给Linux
入口参数：实时位姿
返回  值：无
5000 2000 1000 0x05
**************************************************************************/
void usartSendData(double positionX, double positionY,double positionZ,
double orientationZ,double orientationX,
double orientationW,double orientationY)
{
	// 协议数据缓存数组

	int i, length = 0;

	// 计算位姿
	positionX_stm32.d = positionX;
	positionY_stm32.d = positionY;
	positionZ_stm32.d = positionZ;
	orientationX_stm32.d = orientationX;
	orientationY_stm32.d = orientationY;
	orientationZ_stm32.d = orientationZ;
	orientationW_stm32.d = orientationW;

	// 设置消息头
	for(i = 0; i < 2; i++)
		buf[i] = header[i];                      // buf[0] buf[1]

	// 设置机器人左右轮速度、角度
	length =56 ;
	buf[2] = length;                             // buf[2]
	 memcpy(&buf[3], &positionX_stm32.data, 8);
    memcpy(&buf[11], &positionY_stm32.data, 8);
    memcpy(&buf[19], &positionZ_stm32.data, 8);
    memcpy(&buf[27], &orientationX_stm32.data, 8);
    memcpy(&buf[35], &orientationY_stm32.data, 8);
    memcpy(&buf[43], &orientationZ_stm32.data, 8);
    memcpy(&buf[51], &orientationW_stm32.data, 8);
	/*for(i = 0; i < 2; i++)
	{
		buf[i + 3] = positionX_stm32.data[i];         // buf[3] buf[4]
		buf[i + 5] = positionY_stm32.data[i];        // buf[5] buf[6]
		buf[i + 7] = positionZ_stm32.data[i];           // buf[7] buf[8]
	}*/
	

	// 设置校验值、消息尾
	buf[3 + length] = getCrc8(buf, 3 + length);  // buf[10]
	buf[3 + length + 1] = ender[0];              // buf[11]
	buf[3 + length + 2] = ender[1];              // buf[12]

	//发送字符串数据
	USART_Send_String(buf,sizeof(buf));
}
/**************************************************************************
函数功能：发送指定大小的字符数组，被usartSendData函数调用
入口参数：数组地址、数组大小
返回  值：无
**************************************************************************/
void USART_Send_String(uint8_t *p,uint16_t sendSize)
{
	static int length =0;
	while(length<sendSize)
	{
		//@@@@@#####如果你使用不是USART1更改成相应的，比如USART3，这里有两处修改
		while( !(USART1->ISR&(0x01<<7)) );//发送缓冲区为空
		USART1->TDR=*p;
		p++;
		length++;
	}
	length =0;
}
/**************************************************************************
函数功能：计算八位循环冗余校验，被usartSendData和usartReceiveOneData函数调用
入口参数：数组地址、数组大小
返回  值：无
**************************************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else
                crc >>= 1;
		}
	}
	return crc;
}
/**********************************END***************************************/







