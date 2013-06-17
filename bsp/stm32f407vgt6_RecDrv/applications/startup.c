/*
 * File      : startup.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://openlab.rt-thread.com/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2006-08-31     Bernard      first implementation
 * 2011-06-05     Bernard      modify for STM32F107 version
 */

#include <rthw.h>
#include <rtthread.h>

#include "stm32f4xx.h"
#include "board.h"
#include "App_moduleConfig.h"/**
 * @addtogroup STM32
 */

/*@{*/

extern int  rt_application_init(void);
#ifdef RT_USING_FINSH
extern void finsh_system_init(void);
extern void finsh_set_device(const char* device);
#endif

#ifdef __CC_ARM
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define STM32_SRAM_BEGIN    (&Image$$RW_IRAM1$$ZI$$Limit)
#elif __ICCARM__
#pragma section="HEAP"
#define STM32_SRAM_BEGIN    (__segment_end("HEAP"))
#else
extern int __bss_end;
#define STM32_SRAM_BEGIN    (&__bss_end)
#endif

/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{
	rt_kprintf("\n\r Wrong parameter value detected on\r\n");
	rt_kprintf("       file  %s\r\n", file);
	rt_kprintf("       line  %d\r\n", line);

	while (1) ;
}

/**
 * This function will startup RT-Thread RTOS.
 */
void rtthread_startup(void)
{
	/* init board */
	rt_hw_board_init(); 
	

          //------------------------------------------
      #if 0    
       rt_kprintf("\n\r   starting...--------->>>>>>\r\n ");   

	//WatchDogInit();    
		 while(1)
	   	{      
	   	/* Insert 40 ms delay */
              DF_delay_us(1000);
		//IWDG_ReloadCounter();  
		}
     #endif     	


     #if  0
      rt_kprintf("\n\r   starting...\r\n ");   
	 


  while (1)
  {
     DF_delay_us(1000);
    /* Reload IWDG counter */
    //IWDG_ReloadCounter();  
  }
	 
 
	 
    #endif 	 
         rt_kprintf("\n\r   ���ͨ���� ���������ն� ˫IP ȫ������ƽ̨ TW703-BD--13-6-11 - Version 1.3 !\r\n ");    
	/* show version */
	rt_show_version();

	/* init tick */
	rt_system_tick_init();

	/* init kernel object */
	rt_system_object_init();

	/* init timer system */
	rt_system_timer_init();

      rt_system_heap_init((void*)STM32_SRAM_BEGIN, (void*)STM32_SRAM_END);

	/* init scheduler system */
	rt_system_scheduler_init();

	/* init all device */
	rt_device_init_all();   

//------------------- Device  thread ----------------------------------
       


#ifdef RT_USING_FINSH
	/* init finsh */
	finsh_system_init();
	finsh_set_device( FINSH_DEVICE_NAME ); // mount on
#endif

 #ifdef  GSM_UART
      _gsm_startup();
#endif

  #ifdef _485_DEVICE
      //_485_app_init();
      //_485_set_device(_485_DEVICE); // mount  on
      _485_startup();
  #endif

  #ifdef GPS_UART
      // gps_app_init(); 
      // gps_set_device(GPS_DEVICE);  // mount on
	gps_init();     
       mma8451_driver_init(); 
  #endif

     printer_driver_init();   

   //---  RTC  device Register---------------
    if( rt_hw_rtc_init()==1)
	{
	    RTC_Demo_init();   // Debug  Test  use  
	    rt_kprintf("\n\r   RTC -first conifg\r\n "); 
    	}
  //---------------App Thread    -----------------------
  #ifdef HMI
        HMI_app_init();
  #endif
  

	/* init application_ demo */
	rt_application_init();

      #ifdef APP808
       Protocol_app_init();
      #endif
// -------------------------------------------------------------------------

    /* init timer thread */
      rt_system_timer_thread_init(); 

	/* init idle thread */
	rt_thread_idle_init();

	/* start scheduler */
	rt_system_scheduler_start();

	/* never reach here */
	return ;
}

int main(void)
{
	/* disable interrupt first */
	rt_hw_interrupt_disable();

	/* startup RT-Thread RTOS */
	rtthread_startup();

	return 0;
}

/*@}*/
