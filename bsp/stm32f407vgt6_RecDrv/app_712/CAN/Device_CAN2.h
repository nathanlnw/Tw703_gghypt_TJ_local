#ifndef   _CAN2_DEV
#define    _CAN2_DEV
#include <rtthread.h>
#include <rthw.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

#include  <stdlib.h>//����ת�����ַ���
#include  <stdio.h>
#include  <string.h>
#include "App_moduleConfig.h"
#include "Device_CAN.h"
#include <finsh.h>


extern  struct rt_device  Device_CAN2;

extern void  CAN2_RxHandler(unsigned char rx_data);   
extern void  Device_CAN2_regist(void );    



#endif 
