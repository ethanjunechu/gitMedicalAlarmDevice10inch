#ifndef __BSP_USART_LCD_H__
#define __BSP_USART_LCD_H__

#include "stm32f1xx_hal.h"

#define NOTIFY_TOUCH_PRESS      	0X01  //触摸屏按下通知
#define NOTIFY_TOUCH_RELEASE  		0X03  //触摸屏松开通知
#define NOTIFY_WRITE_FLASH_OK  		0X0C  //写FLASH成功
#define NOTIFY_WRITE_FLASH_FAILD  	0X0D  //写FLASH失败
#define NOTIFY_READ_FLASH_OK  		0X0B  //读FLASH成功
#define NOTIFY_READ_FLASH_FAILD  	0X0F  //读FLASH失败
#define NOTIFY_MENU                 0X14  //菜单事件通知
#define NOTIFY_TIMER                0X43  //定时器超时通知
#define NOTIFY_CONTROL              0XB1  //控件更新通知
#define NOTIFY_READ_RTC             0XF7  //读取RTC时间
#define MSG_GET_CURRENT_SCREEN 		0X01  //画面ID变化通知
#define MSG_GET_DATA                0X11  //控件数据通知

#define PAGE_START 		0	 // start界面
#define PAGE_MAIN3 		1	 // main界面
#define PAGE_PASSWORD 	2	 // password界面
#define PAGE_SET1 		3	 // set1界面
#define PAGE_SET2 		4	 // set2界面
#define PAGE_SET3 		5	 // set3界面
#define PAGE_SET4 		6	 // set4界面
#define PAGE_MAIN11		8 	 // main界面
#define PAGE_MAIN12		9 	 // main界面
#define PAGE_MAIN13		10 	 // main界面
#define PAGE_MAIN14		16 	 // main界面
#define PAGE_MAIN15		17 	 // main界面
#define PAGE_MAIN16		18 	 // main界面
#define PAGE_MAIN212	11 	 // main界面
#define PAGE_MAIN213	12 	 // main界面
#define PAGE_MAIN214	19 	 // main界面
#define PAGE_MAIN215	20 	 // main界面
#define PAGE_MAIN216	21 	 // main界面
#define PAGE_MAIN223	13 	 // main界面
#define PAGE_MAIN224	22 	 // main界面
#define PAGE_MAIN225	23 	 // main界面
#define PAGE_MAIN226	24 	 // main界面
#define PAGE_MAIN234	25 	 // main界面
#define PAGE_MAIN235	26 	 // main界面
#define PAGE_MAIN236	27 	 // main界面
#define PAGE_MAIN245	28 	 // main界面
#define PAGE_MAIN246	62 	 // main界面
#define PAGE_MAIN256	29 	 // main界面
#define PAGE_MAIN3123	30 	 // main界面
#define PAGE_MAIN3124	31 	 // main界面
#define PAGE_MAIN3125	32 	 // main界面
#define PAGE_MAIN3126	33 	 // main界面
#define PAGE_MAIN3134	34 	 // main界面
#define PAGE_MAIN3135	35 	 // main界面
#define PAGE_MAIN3136	36 	 // main界面
#define PAGE_MAIN3145	37 	 // main界面
#define PAGE_MAIN3146	38 	 // main界面
#define PAGE_MAIN3156	63 	 // main界面
#define PAGE_MAIN3234	39 	 // main界面
#define PAGE_MAIN3235	40 	 // main界面
#define PAGE_MAIN3236	41 	 // main界面
#define PAGE_MAIN3245	42 	 // main界面
#define PAGE_MAIN3246	43 	 // main界面
#define PAGE_MAIN3256	44 	 // main界面
#define PAGE_MAIN3345	45 	 // main界面
#define PAGE_MAIN3346	46 	 // main界面
#define PAGE_MAIN3356	64 	 // main界面
#define PAGE_MAIN3456	47 	 // main界面
#define PAGE_MAIN41234	48 	 // main界面
#define PAGE_MAIN41235	49 	 // main界面
#define PAGE_MAIN41236	50 	 // main界面
#define PAGE_MAIN41245	65 	 // main界面
#define PAGE_MAIN41246	66 	 // main界面
#define PAGE_MAIN41256	71 	 // main界面
#define PAGE_MAIN41345	51 	 // main界面
#define PAGE_MAIN41346	52 	 // main界面
#define PAGE_MAIN41356	53 	 // main界面
#define PAGE_MAIN41456	54 	 // main界面
#define PAGE_MAIN42345	55 	 // main界面
#define PAGE_MAIN42346	56 	 // main界面
#define PAGE_MAIN42356	67 	 // main界面
#define PAGE_MAIN42456	57 	 // main界面
#define PAGE_MAIN43456	58 	 // main界面
#define PAGE_MAIN512345	59 	 // main界面
#define PAGE_MAIN512346	60 	 // main界面
#define PAGE_MAIN512356	68 	 // main界面
#define PAGE_MAIN512456	70 	 // main界面
#define PAGE_MAIN513456	61 	 // main界面
#define PAGE_MAIN523456	69 	 // main界面

#define PAGE_BLUETOOTH	14	 // 蓝牙界面
#define PAGE_LIC	15	 // 授权到期界面

#define PTR2U16(PTR) ((((uint8_t *)(PTR))[0]<<8)|((uint8_t *)(PTR))[1])  //从缓冲区取16位数据
#define PTR2U32(PTR) ((((uint8_t *)(PTR))[0]<<24)|(((uint8_t *)(PTR))[1]<<16)|(((uint8_t *)(PTR))[2]<<8)|((uint8_t *)(PTR))[3])  //从缓冲区取32位数据

enum CtrlType {
	kCtrlUnknown = 0x00, kCtrlButton = 0x10,		//按钮
	kCtrlText = 0x11,		//文本
	kCtrlProgress,			//进度条
	kCtrlSlider,    		//滑动条
	kCtrlMeter,  			//仪表
	kCtrlDropList, 			//下拉列表
	kCtrlAnimation = 0x27,	//动画
	kCtrlRTC, 				//时间显示
	kCtrlGraph, 			//曲线图控件
	kCtrlTable, 			//表格控件
	kCtrlMenu = 0x1a,				//菜单控件
	kCtrlSelector = 0x1B,	//选择控件
	kCtrlQRCode,			//二维码
};

typedef struct {
	uint8_t cmd_head;  		//帧头

	uint8_t cmd_type;  		//命令类型(UPDATE_CONTROL)
	uint8_t ctrl_msg;   		//CtrlMsgType-指示消息的类型
	uint8_t screen_id_high;  //产生消息的画面ID
	uint8_t screen_id_low;
	uint8_t control_id_high;	//产生消息的控件ID
	uint8_t control_id_low;
	uint8_t control_type; 	//控件类型

	uint8_t param[256];		//可变长度参数，最多256个字节

	uint8_t cmd_tail[4];   	//帧尾
} CTRL_MSG, *PCTRL_MSG;

void ProcessUIMessage(PCTRL_MSG msg, uint16_t size);
void NotifyScreen(uint16_t screen_id);
void NotifyTouchXY(uint8_t press, uint16_t x, uint16_t y);
void NotifyButton(uint16_t screen_id, uint16_t control_id, uint8_t state);
void NotifyText(uint16_t screen_id, uint16_t control_id, uint8_t *str);
void NotifyProgress(uint16_t screen_id, uint16_t control_id, uint32_t value);
void NotifySlider(uint16_t screen_id, uint16_t control_id, uint32_t value);
void NotifyMeter(uint16_t screen_id, uint16_t control_id, uint32_t value);
void NotifyMenu(uint16_t screen_id, uint16_t control_id, uint8_t item,
		uint8_t state);
void NotifySelector(uint16_t screen_id, uint16_t control_id, uint8_t item);
void NotifyTimer(uint16_t screen_id, uint16_t control_id);
void NotifyReadFlash(uint8_t status, uint8_t *_data, uint16_t length);
void NotifyWriteFlash(uint8_t status);
void NotifyReadRTC(uint8_t year, uint8_t month, uint8_t week, uint8_t day,
		uint8_t hour, uint8_t minute, uint8_t second);
void NotifyAnimation(uint16_t screen_id, uint16_t control_id);
#endif
