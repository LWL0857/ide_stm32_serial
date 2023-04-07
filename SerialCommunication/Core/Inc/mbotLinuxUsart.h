#ifndef __MBOTLINUXUSART__
#define __MBOTLINUXUSART__
#include "main.h"

#include<string.h>

#define START   0X11

#ifdef __cplusplus
extern "C" {
#endif

//从linux接收并解析数据到参数地址中
extern int usartReceiveOneData(int *p_positionX_rec_mocap, int *p_positionY_rec_mocap, int *p_positionZ_rec_mocap,
						int *p_orientationX_rec_mocap, int *p_orientationY_rec_mocap,
						int *p_orientationZ_rec_mocap, int *p_orientationW_rec_mocap);
//封装数据，调用USART1_Send_String将数据发送给linux
extern void usartSendData(double positionX, double positionY,double positionZ,
double orientationZ,double orientationX,
double orientationW,double orientationY);
//发送指定字符数组的函数
void USART_Send_String(unsigned char *p,unsigned short sendSize);
//计算八位循环冗余校验，得到校验值，一定程度上验证数据的正确性
unsigned char getCrc8(unsigned char *ptr, unsigned short len);

#endif
#ifdef __cplusplus
}
#endif



