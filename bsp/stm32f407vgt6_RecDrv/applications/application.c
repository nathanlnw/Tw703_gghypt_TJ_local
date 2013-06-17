/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <stdio.h>

#include "stm32f4xx.h"
#include <board.h>
#include <rtthread.h>
#include "App_moduleConfig.h"

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#include "stm32_eth.h"
#endif

void rt_init_thread_entry(void* parameter)
{
    /* LwIP Initialization */
#ifdef RT_USING_LWIP
    {
        extern void lwip_sys_init(void);

        /* register ethernetif device */
        eth_system_device_init();

        rt_hw_stm32_eth_init();
        /* re-init device driver */
        rt_device_init_all();

        /* init lwip system */
        lwip_sys_init();
        rt_kprintf("TCP/IP initialized!\n");
    }
#endif

//FS

//GUI
}



ALIGN(RT_ALIGN_SIZE)
static char thread_led_stack[1024];
struct rt_thread thread_led;



static void rt_thread_entry_led(void* parameter)
{
    u8 LowPowerCounter=0,CutPowerCounter=0,Battery_Flag=0;;	

      	   CAN_App_Init();   // CAN初始化   

  while (1)
  {
              //-------  CAN query ---------------     
                                         
                              
	    if(CAN_initOver==1)
	    {
				   TestRx=(TestStatus)CANRXStr();    
				  if (TestRx == PASSED) 
				   	   rt_kprintf("\r\n CAN1-RxData\r\n");	 
	     }
		 //---------------------------------------------
          				//----------------------
				//------------- 电源电压AD显示 ----------------------- 
				 ADC_ConvertedValue=ADC_GetConversionValue(ADC1);               
				 AD_Volte=((ADC_ConvertedValue*543)>>12);   
					//rt_kprintf ("\r\n  获取到的电池AD数值为:	%d	 AD电压为: %d V  电源电压: %d V\r\n",ADC_ConvertedValue,a,a+11);	 
					 //  ---电源欠压报警---- 
				 AD_Volte=AD_Volte+11;   


                          //----------------------
                          if(ADC_ConvertedValue<500)  //  小于500 认为是外部断电
				{
				      CutPowerCounter++;
				       if(CutPowerCounter>15)
				       	{
                                                  CutPowerCounter=0;
							 LowPowerCounter=0;					  
    
							   //------ 超级电容  为高则 启动了 
							    if(Battery_Flag==0)
							    	{
				                       rt_kprintf("\r\n   主电源掉电! \r\n");   
									   Battery_Flag=1;	 
									    PositionSD_Enable();
									    Current_UDP_sd=1;	
							    	}
						     //--------------------------------					   
				       	}
 
                          	}
			     else
				{      //    电源正常情况下
				       CutPowerCounter=0;
                                   Powercut_Status=0x01;
					  if(Battery_Flag==1)
			    	         {
                                         rt_kprintf("\r\n   主电源正常! \r\n");   
					       Battery_Flag=1;	  
					       PositionSD_Enable();
						Current_UDP_sd=1;    
			               }	
				         Battery_Flag=0;
                                   //------------判断欠压和正常-----
					 if(AD_Volte<160)	// 16V      
					  {
							if((Warn_Status[3]&0x80)==0x00)     
							{
								  LowPowerCounter++;
								  if(LowPowerCounter>15) 
								  {
								    LowPowerCounter=0;						
									Warn_Status[3]|=0x80;  //欠压报警
								     PositionSD_Enable();
								     Current_UDP_sd=1;	 
								    rt_kprintf("\r\n 欠压报警! \r\n");
								  }	
							}  				   
					  }
					 else
					 {
						    if((Warn_Status[3]&0x80)==0x80) 
						   	 rt_kprintf("\r\n 从欠压中还原正常! \r\n");  
						   
						     LowPowerCounter=0;	
							 Warn_Status[3]&=~0x80; //取消欠压报警 
					 }

					//---------------------------------------			   
			     	}	           

		//-----------------------------------------------
                        rt_thread_delay(RT_TICK_PER_SECOND/10);
	
  }
}


int rt_application_init()
{
    rt_thread_t init_thread;


       Device_CAN2_regist();    //  Device CAN2 Init

#if (RT_THREAD_PRIORITY_MAX == 32)
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 8, 20);
#else
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL, 
                                   2048, 80, 20);
#endif

    if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);

    //------- init led1 thread
    rt_thread_init(&thread_led,
                   "led1",
                   rt_thread_entry_led,
                   RT_NULL,
                   &thread_led_stack[0],
                   sizeof(thread_led_stack),Prio_Demo,5); 
    rt_thread_startup(&thread_led);
	
    return 0; 

}
/*@}*/
