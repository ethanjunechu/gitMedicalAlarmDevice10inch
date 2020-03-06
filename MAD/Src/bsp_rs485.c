/*
 *********************************************************************************************************
 *
 * 模块名称 : 串口驱动模块
 * 文件名称 : bsp_usart.c
 * 版   本 : V1.0
 * 说   明 : 实现printf和scanf函数重定向到串口1，即支持printf信息到USART1
 *       实现重定向，只需要添加2个函数:
 *       int fputc(int ch, FILE *f);
 *       int fgetc(FILE *f);
 *       对于KEIL MDK编译器，编译选项中需要在MicorLib前面打钩，否则不会有数据打印到USART1。
 * 修改记录 :
 *   版本号  日期       作者    说明
 *   v0.1    2009-12-27 armfly  创建该文件，ST固件库版本为V3.1.2
 *   v1.0    2011-01-11 armfly  ST固件库升级到V3.4.0版本。
 *
 * Copyright (C), 2010-2011, 安富莱电子 www.armfly.com
 *
 *********************************************************************************************************
 */

#include "bsp_rs485.h"
#include <stdio.h>

short ad_data[8];
unsigned char real_modal_addr = 0x01;

static unsigned char auchCRCHi[] = { 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80,
		0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 };
// CRC 低位字节值表
static char auchCRCLo[] = { 0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2,
		0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
		0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
		0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F,
		0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6,
		0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1,
		0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
		0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB,
		0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA,
		0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5,
		0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
		0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
		0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE,
		0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79,
		0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
		0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73,
		0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
		0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D,
		0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
		0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F,
		0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86,
		0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40 };

unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen) {
	unsigned char uchCRCHi = 0xFF; /* 高CRC字节初始化 */
	unsigned char uchCRCLo = 0xFF; /* 低CRC 字节初始化 */
	unsigned uIndex; /* CRC循环中的索引 */
	while (usDataLen--) /* 传输消息缓冲区 */
	{
		uIndex = uchCRCHi ^ *puchMsg++; /* 计算CRC */
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
		uchCRCLo = auchCRCLo[uIndex];
	}
	return (uchCRCHi << 8 | uchCRCLo);
}

//设置把8路AD值读到ad_data[8]中
//unsigned char Read485_ADData_All(unsigned char modal_addr)
//{
//  unsigned char buffer[8];
//  unsigned short crc;
//  int i=0;
//  buffer[0]=modal_addr;
//  buffer[1]=0x03;//读数据
//  buffer[2]=0;
//  buffer[3]=0;
//  buffer[4]=0;
//  buffer[5]=0x08;
//  crc=CRC16(buffer, 6);
//  buffer[6]=crc&0xff;
//  buffer[7]=(crc&0xff00)>>8;
//  //ex. 01 03 00 00 00 08 44 0C
//  RS485_Send_Data(buffer,8);
//  //ex.01 03 10 09 CE 00 00 00 00 00 00 00 00 00 00 00 00 00 00 6C 5B
////  后期修改
////  while(RS485_RX_CNT!=21)
////  {
////    i++;
////    if(i>1000000000)
////    return 1;
////  }
//  crc =CRC16(RS485_RX_BUF,19);
//  if(((crc&0xff)!=RS485_RX_BUF[19])||(((crc&0xff00)>>8)!=RS485_RX_BUF[20]))
//    return 1;
//  for(i=0;i<8;i++)
//  {
//    ad_data[i]=(RS485_RX_BUF[3+2*i]<<8)|(RS485_RX_BUF[4+2*i]);
//  }
//  return 0;

//}

//设置485模块地址
//unsigned char SetModal485Addr(unsigned char modal_addr)
//{
//  unsigned char buffer[8];
//  unsigned short crc;
//  int i=0;
//  buffer[0]=0;//广播发送
//  buffer[1]=0x06;//写单个寄存器
//  buffer[2]=0;
//  buffer[3]=0x64;
//  buffer[4]=0;
//  buffer[5]=modal_addr;
//  crc=CRC16(buffer, 6);
//  buffer[6]=crc&0xff;
//  buffer[7]=(crc&0xff00)>>8;
//  //ex.  00 06 00 64 00 01 08 04
//  RS485_Send_Data(buffer,8);
//  //ex.  00 06 00 64 00 01 08 04
//// 后期修改
////  while(RS485_RX_CNT!=8)
////  {
////    i++;
////    if(i>1000000000)
////    return 1;
////
////  }
//  crc =CRC16(RS485_RX_BUF,6);
//  if(((crc&0xff)!=RS485_RX_BUF[6])||(((crc&0xff00)>>8)!=RS485_RX_BUF[7]))
//    return 1;

//  real_modal_addr=RS485_RX_BUF[5];
//  return 0;

//}

/**********************************************************
 * 功能：发送数据
 * 参数：buf 发送缓冲区首地址
 *     len 待发送的字节数
 **********************************************************/
void RS485_Send_Data(uint8_t *buf, uint8_t len) {
	RS485_TX_ON()
	;     //设置为发送模式
	HAL_UART_Transmit(&huart3, (uint8_t *) buf, len, 0xFFFF);
	RS485_TX_OFF()
	;    //设置为接收模式
}
