/*
       IC  card 
*/


#include <rtthread.h> 
#include <rthw.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

#include  <stdlib.h>//����ת�����ַ���
#include  <stdio.h>
#include  <string.h>
#include  <ctype.h>
#include  <stdlib.h>
#include  <stdarg.h>
#include "App_moduleConfig.h" 
#include "Vdr.h"




unsigned char IC_CardInsert=0;//1:IC��������ȷ  2:IC���������
unsigned char IC_Check_Count=0;
unsigned int      read_counter=0,flag_8024off=1;    
unsigned char   Init8024Flag=0; 
unsigned int      DelayCheckIc=0;
unsigned char   institution[45];
unsigned char administrator_card=0;
u8        powerOn_first=0;   //    �״��ϵ���жϰο�  



void KeyBuzzer(unsigned char num)
{

if(num==1)
	{
	BuzzerFlag++;
	if(BuzzerFlag==2)
		GPIO_SetBits(Buzzer_IO_Group,Buzzer_Group_Num);	
	if(BuzzerFlag==4)
		{
			GPIO_ResetBits(Buzzer_IO_Group,Buzzer_Group_Num);
			BuzzerFlag=0;
			IC_CardInsert=0;
		}
	  
	}
else if(num==2)
	{
	BuzzerFlag++;
	if((BuzzerFlag==12)||(BuzzerFlag==16))
		GPIO_SetBits(Buzzer_IO_Group,Buzzer_Group_Num);	
	if((BuzzerFlag==14)||(BuzzerFlag==18))
		GPIO_ResetBits(Buzzer_IO_Group,Buzzer_Group_Num);
	if(BuzzerFlag==18)
		{BuzzerFlag=0;IC_CardInsert=0;}  
	}
}



void CheckICInsert(void)
{
unsigned char write_flag=0;
u8 result0=0,result1=0,result2=0,result3=0,result4=0,result5=0;//i=0;
u8 FLagx=0;//,len=0;
unsigned char reg_record[32];
u32 DriveCode32=0;


//===================����IC����д==================================================
if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7))
	{
	IC_Check_Count++;
	if(IC_Check_Count>=10)
		{
		IC_Check_Count=0;
	//�����ϵ翪8024�ĵ�
/*	if(flag_8024off==1)       //  �ϵ粻��ȡIC ��
		{
			R_Flag|=b_CardEdge;
			Init8024Flag=2;
			flag_8024off=0;
		}
 */
	//8024��off�ӵͱ��
	if(Init8024Flag==1)
		{
			Init8024Flag=2;
			R_Flag|=b_CardEdge;
			//rt_kprintf("pc7  Ϊ �ߣ�R_Flag��1\r\n");
		}
	//��⵽�����ʼ��ic��
	if((R_Flag&b_CardEdge)&&(Init8024Flag==2))
		{
		Init8024Flag=3;
		_CardCMDVCC_LOW;
		for(DelayCheckIc=0;DelayCheckIc<500;DelayCheckIc++)
		DELAY5us();
		_CardSetPower_HIGH;
		_CardSetRST_LOW;
		for(DelayCheckIc=0;DelayCheckIc<15;DelayCheckIc++)
			{
			_CardSetCLK_LOW;
			DELAY5us();DELAY5us();DELAY5us();
			_CardSetCLK_HIGH;
			DELAY5us();DELAY5us();DELAY5us();
			_CardSetCLK_LOW;
			}			 
		R_Flag&=~b_CardEdge;
		write_flag=1;
	     	rt_kprintf("  �忨  ִֻ��1��\r\n");
			//----------------------------------------------------
            VDR_product_12H(0x01);  //  ��¼  
			//----------------------------------------------------
		}
		}
	}
else 
	{
	
	IC_Check_Count=0;
	_CardSetRST_HIGH;
	_CardSetPower_LOW;
	_CardCMDVCC_HIGH;
	if(Init8024Flag==0)
		{
			Init8024Flag=1;
			if(powerOn_first==0)
				 powerOn_first=1;
			else
			 {
			    //--------------------------------------------------- 
			     rt_kprintf("   �ο� pc7  Ϊ ��---����\r\n");  
                 VDR_product_12H(0x02);  //  �ǳ� 
				//---------------------------------------------------
				 
			 }		 
		}
	}

if(write_flag==1)
	{
	write_flag=0;
	Rx_4442(241,13,reg_record);	//����Ա��
	if(strncmp((char *)reg_record,"administrator",13)==0)
		{
		  rt_kprintf("\r\n����Ա��");
		  administrator_card=1;
		}
	else
		{
	     memset(DriverCardNUM,0,sizeof(DriverCardNUM));
	     memset(DriverName,0,sizeof(DriverName));
	     //memset(JT808Conf_struct.Driver_Info,0,sizeof(JT808Conf_struct.Driver_Info));

           result0=Rx_4442(70,10,(unsigned char *)DriverName);	//����ʻԱ����
	    rt_kprintf("\r\n��ʻԱ����:%s,result0=%d",DriverName,result0);

		
		result1=Rx_4442(52,18,(unsigned char *)DriverCardNUM);	//����ʻ֤����
		rt_kprintf("\r\n��ʻ֤����:%s,result1=%d",DriverCardNUM,result1);

              memset(JT808Conf_struct.Driver_Info.DriveCode,0,sizeof(JT808Conf_struct.Driver_Info.DriveCode));
		result2=Rx_4442(49,3,(unsigned char *)JT808Conf_struct.Driver_Info.DriveCode);	//����ʻԱ����
		DriveCode32=(JT808Conf_struct.Driver_Info.DriveCode[0]<<16)+(JT808Conf_struct.Driver_Info.DriveCode[1]<<8)+JT808Conf_struct.Driver_Info.DriveCode[2];
		rt_kprintf("\r\n��ʻԱ����:%d,result2=%d",DriveCode32,result2);   

		memset(JT808Conf_struct.Driver_Info.Driver_ID,0,sizeof(JT808Conf_struct.Driver_Info.Driver_ID));
		result3=Rx_4442(80,20,(unsigned char *)JT808Conf_struct.Driver_Info.Driver_ID);	//���֤����
		rt_kprintf("\r\n���֤����:%s,result3=%d",JT808Conf_struct.Driver_Info.Driver_ID,result3);

		memset(JT808Conf_struct.Driver_Info.Drv_CareerID,0,sizeof(JT808Conf_struct.Driver_Info.Drv_CareerID));
		result4=Rx_4442(100,40,(unsigned char *)JT808Conf_struct.Driver_Info.Drv_CareerID);	//��ҵ�ʸ�֤
		rt_kprintf("\r\n��ҵ�ʸ�֤:%s,result4=%d",JT808Conf_struct.Driver_Info.Drv_CareerID,result4);

		memset(JT808Conf_struct.Driver_Info.Comfirm_agentID,0,sizeof(JT808Conf_struct.Driver_Info.Comfirm_agentID));
		result5=Rx_4442(140,41,(unsigned char *)institution);	//��֤����
		memcpy(JT808Conf_struct.Driver_Info.Comfirm_agentID,&institution[1],40);
		rt_kprintf("\r\n��֤����:%s,result5=%d",JT808Conf_struct.Driver_Info.Comfirm_agentID,result5);
		
		if((result0==0)&&(result1==0)&&(result2==0)&&(result3==0)&&(result4==0)&&(result5==0))//�������ȷ
		{
		IC_CardInsert=1;//IC	��������ȷ
		FLagx=0;
 
		//ǰ18λΪ��ʻ֤����-------------ƣ�ͼ�ʻ���------���Ƿ�����˿� --------------
		if(strncmp((char*)DriverCardNUM,(char*)JT808Conf_struct.Driver_Info.DriverCard_ID,18)!=0)
			{	
			memset(JT808Conf_struct.Driver_Info.DriverCard_ID,0,sizeof(JT808Conf_struct.Driver_Info.DriverCard_ID));  
			memcpy(JT808Conf_struct.Driver_Info.DriverCard_ID,DriverCardNUM,18);  
			FLagx=1;
			}
		//��20λΪ��ʻԱ����-------------ƣ�ͼ�ʻ���------���Ƿ�����˿� --------------
		if(strncmp((char*)DriverName,(char*)JT808Conf_struct.Driver_Info.DriveName,20)!=0)
			{							   
			memset(JT808Conf_struct.Driver_Info.DriveName,0,sizeof(JT808Conf_struct.Driver_Info.DriveName));  
			memcpy((u8*)JT808Conf_struct.Driver_Info.DriveName,DriverName,strlen((const char*)DriverName));    		
			FLagx=2;
			}
		if(FLagx)//������IC ��    ���ƣ�ͼ�ʻ���
			{
			TIRED_Drive_Init();  //���ƣ�ͼ�ʻ��״̬	  
			GPIO_ResetBits(Buzzer_IO_Group,Buzzer_Group_Num); // �رշ����� 		
			Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
			FLagx=0;//clear
			} 
		  BuzzerFlag=1;//��һ����ʾ 
		  pMenuItem=&Menu_2_5_DriverInfor;
	        pMenuItem->show();
		}
	else
		{
		         BuzzerFlag=11;//��һ����ʾ
		         IC_CardInsert=2;//IC	��������� 
		}
		}
	Init8024Flag=0;
	GpsIo_Init();
	} 

//===================����IC����д���==================================================

}


