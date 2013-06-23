#include  <string.h>
#include "Menu_Include.h"

u8 comfirmation_flag=0;
u8 col_screen=0;
u8 CarBrandCol_Cou=1;

unsigned char car_col[13]={"������ɫ:��ɫ"}; 

void car_col_fun(u8 par)
{                                                      
                                           //������ɫ�����
if(par==1)
	memcpy(Menu_VecLogoColor,"��ɫ",4);     //   1
else if(par==2)
	memcpy(Menu_VecLogoColor,"��ɫ",4);    //   2
else if(par==3)
	memcpy(Menu_VecLogoColor,"��ɫ",4);     //   3
else if(par==4)
	memcpy(Menu_VecLogoColor,"��ɫ",4);    //   4
else if(par==5)
   {	memcpy(Menu_VecLogoColor,"����",4);  par=9; } //   9
   
Menu_color_num=par; 

memcpy(car_col+9,Menu_VecLogoColor,4);  
lcd_fill(0);
lcd_text12(20,10,(char *)car_col,13,LCD_MODE_SET);
lcd_update_all();
}
static void msg( void *p)
{

}
static void show(void)
{
CounterBack=0;
col_screen=1;
car_col_fun(2);// old 1
}


static void keypress(unsigned int key)
{
u8 error=0;
	switch(KeyValue)
		{
		case KeyValueMenu:
			if(comfirmation_flag==4)
				{
				pMenuItem=&Menu_1_Idle;
				pMenuItem->show();
				}
			else
				{
				pMenuItem=&Menu_0_loggingin;
				pMenuItem->show();
				}
			col_screen=0;
			CarBrandCol_Cou=1;
			comfirmation_flag=0;
			break;
		case KeyValueOk:
                  if(col_screen==1)
				{
				col_screen=2;
				CarSet_0_counter=1;//
				menu_color_flag=1;//������ɫ�������
                         lcd_fill(0);
				lcd_text12(20,3,(char *)car_col,13,LCD_MODE_SET);
				lcd_text12(12,18,"��ȷ�ϼ��鿴��Ϣ",16,LCD_MODE_SET);
				lcd_update_all();
				}
			else if(col_screen==2)
				{
				menu_color_flag=0;
				
				col_screen=3;
				comfirmation_flag=1;//����������Ϣ��־
				lcd_fill(0);
				lcd_text12(0,0,(char *)Menu_Car_license,8,LCD_MODE_SET);
				lcd_text12(54,0,(char *)Menu_VechileType,6,LCD_MODE_SET);
				lcd_text12(96,0,(char *)Menu_VecLogoColor,4,LCD_MODE_SET);
				lcd_text12(0,12,"SIM������",9,LCD_MODE_SET);
				lcd_text12(55,12,(char *)Menu_sim_Code,12,LCD_MODE_SET);
				lcd_text12(24,23,"ȷ��",4,LCD_MODE_INVERT);
				lcd_text12(72,23,"ȡ��",4,LCD_MODE_SET);
				lcd_update_all();
				}
			else if(comfirmation_flag==1)
				{
				col_screen=0;
				comfirmation_flag=4;
				//�������õ���Ϣ
				lcd_fill(0);
				lcd_text12(18,3,"������������Ϣ",14,LCD_MODE_SET);
				lcd_text12(0,18,"���˵��������������",20,LCD_MODE_SET);
				lcd_update_all();

                            //���ƺ�
                //rt_kprintf("\r\n(������Ϣ)Menu_Car_license=%s",Menu_Car_license);
				memset(JT808Conf_struct.Vechicle_Info.Vech_Num,0,sizeof(JT808Conf_struct.Vechicle_Info.Vech_Num));
				memcpy(JT808Conf_struct.Vechicle_Info.Vech_Num,Menu_Car_license,strlen(Menu_Car_license));
                //rt_kprintf("\r\n(������Ϣ)JT808Conf_struct.Vechicle_Info.Vech_Num=%s",JT808Conf_struct.Vechicle_Info.Vech_Num);
				// ��������
				memset(JT808Conf_struct.Vechicle_Info.Vech_Type,0,sizeof(JT808Conf_struct.Vechicle_Info.Vech_Type));
				memcpy(JT808Conf_struct.Vechicle_Info.Vech_Type,Menu_VechileType,10);
                //rt_kprintf("\r\n(����1   )JT808Conf_struct.Vechicle_Info.Vech_Num=%s",JT808Conf_struct.Vechicle_Info.Vech_Num);
                                
///////
				 //����VIN
				memset(JT808Conf_struct.Vechicle_Info.Vech_VIN,0,sizeof(JT808Conf_struct.Vechicle_Info.Vech_VIN));
				memcpy(JT808Conf_struct.Vechicle_Info.Vech_VIN,Menu_Vin_Code,17);
				//rt_kprintf("\r\n(����2   )JT808Conf_struct.Vechicle_Info.Vech_Num=%s",JT808Conf_struct.Vechicle_Info.Vech_Num);
                 
				// SIM������
				//memset(JT808Conf_struct.Vech_sim,0,sizeof(JT808Conf_struct.Vech_sim));
				//memcpy(JT808Conf_struct.Vech_sim,Menu_sim_Code,11);
				//-----------------------------------------------------------------------------
							 memset(DeviceNumberID,0,sizeof(DeviceNumberID));
							  memcpy(DeviceNumberID,Menu_sim_Code,12);								 
							  DF_WriteFlashSector(DF_DeviceID_offset,0,DeviceNumberID,13); 
							  delay_ms(80); 		  
							  rt_kprintf("\r\n �ֶ��豸ID����Ϊ : %s", DeviceNumberID);  
							  DF_ReadFlash(DF_DeviceID_offset,0,DeviceNumberID,13);     
							  DeviceID_Convert_SIMCODE();  // ת�� 
		              //----------------------------------------------------------------------------
                //rt_kprintf("\r\n(����3   )JT808Conf_struct.Vechicle_Info.Vech_Num=%s",JT808Conf_struct.Vechicle_Info.Vech_Num);
                               
				// ������ɫ
				JT808Conf_struct.Vechicle_Info.Dev_Color=Menu_color_num;
				//�����������
				JT808Conf_struct.password_flag=1; 
				//  �洢
				//rt_kprintf("\r\n(����4   )JT808Conf_struct.Vechicle_Info.Vech_Num=%s",JT808Conf_struct.Vechicle_Info.Vech_Num);
				        
				error=Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
				//rt_kprintf("\r\n write  error=%d",error);
				delay_ms(3);
				error=Api_Config_read(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   //  ��ȡJT808   ������Ϣ
                //rt_kprintf("\r\n read   error=%d",error);
                //rt_kprintf("\r\n(read    )JT808Conf_struct.Vechicle_Info.Vech_Num=%s",JT808Conf_struct.Vechicle_Info.Vech_Num);
				
				}
			else if(comfirmation_flag==2)
				{
				col_screen=0;
				comfirmation_flag=3;
				lcd_fill(0);
				lcd_text12(6, 3,"��ȷ���Ƿ���������",18,LCD_MODE_SET);
				lcd_text12(12,18,"��ȷ�ϼ���������",16,LCD_MODE_SET);
				lcd_update_all();
				}
			else if(comfirmation_flag==3)
				{
				col_screen=0;
				comfirmation_flag=0;
				//��������
				pMenuItem=&Menu_0_loggingin;
				pMenuItem->show();
				
				comfirmation_flag=0;
				col_screen=0;
				CarBrandCol_Cou=1;
				}

			break;
		case KeyValueUP:
			if(col_screen==1)
				{
				CarBrandCol_Cou--;
				if(CarBrandCol_Cou<1)
					CarBrandCol_Cou=5;
				car_col_fun(CarBrandCol_Cou);
				}
			else if(col_screen==3)
				{
				comfirmation_flag=1;
				lcd_fill(0);
			       lcd_text12(0,3,Menu_Car_license,8,LCD_MODE_SET);
				lcd_text12(54,3,Menu_VechileType,6,LCD_MODE_SET);
				lcd_text12(96,3,(char *)Menu_VecLogoColor,4,LCD_MODE_SET);
				lcd_text12(24,23,"ȷ��",4,LCD_MODE_INVERT);
				lcd_text12(72,23,"ȡ��",4,LCD_MODE_SET);
				lcd_update_all();
				}

			break;
		case KeyValueDown:
			if(col_screen==1)
				{
				CarBrandCol_Cou++;
				if(CarBrandCol_Cou>5)
					CarBrandCol_Cou=1;
				car_col_fun(CarBrandCol_Cou);
				}
			else if(col_screen==3)
				{
				comfirmation_flag=2;
				lcd_fill(0);
				lcd_text12(0,3,Menu_Car_license,8,LCD_MODE_SET);
				lcd_text12(54,3,Menu_VechileType,6,LCD_MODE_SET);
				lcd_text12(96,3,(char *)Menu_VecLogoColor,4,LCD_MODE_SET);
				lcd_text12(24,23,"ȷ��",4,LCD_MODE_SET);
				lcd_text12(72,23,"ȡ��",4,LCD_MODE_INVERT);
				lcd_update_all();
				}

			break;
		}
	KeyValue=0;
}


static void timetick(unsigned int systick)
{

	/*CounterBack++;
	if(CounterBack!=MaxBankIdleTime*5)
		return;
	CounterBack=0;
	pMenuItem=&Menu_0_loggingin;
	pMenuItem->show();


	col_screen=0;
	CarBrandCol_Cou=1;
	comfirmation_flag=0;*/
}

ALIGN(RT_ALIGN_SIZE)
MENUITEM	Menu_0_5_Colour=
{
"������ɫ����",
	12,
	&show,
	&keypress,
	&timetick,
	&msg,
	(void*)0
};


