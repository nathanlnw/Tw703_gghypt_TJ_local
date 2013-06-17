#include  <string.h>
#include "Menu_Include.h"

#define  Sim_width1  6

u8 Sim_SetFlag=1,Sim_SetCounter=0;

unsigned char select_Sim[]={0x0C,0x06,0xFF,0x06,0x0C};

DECL_BMP(8,5,select_Sim);


void Sim_Set(u8 par)
{
	lcd_fill(0);
	lcd_text12(0,3,(char *)Menu_sim_Code,Sim_SetFlag-1,LCD_MODE_SET);
	
	lcd_bitmap(par*Sim_width1, 14, &BMP_select_Sim, LCD_MODE_SET);
	lcd_text12(0,19,"0123456789",10,LCD_MODE_SET);

	lcd_update_all();
}


static void show(void)
{
Sim_Set(Sim_SetCounter*Sim_width1);
}


static void keypress(unsigned int key)
{
	switch(KeyValue)
		{
		case KeyValueMenu:
			pMenuItem=&Menu_0_loggingin;
			pMenuItem->show();

		       memset(Menu_sim_Code,0,sizeof(Menu_sim_Code));
                    Sim_SetFlag=1;
		       Sim_SetCounter=0;

			break;
		case KeyValueOk:
			if((Sim_SetFlag>=1)&&(Sim_SetFlag<=12))
				{
				if(Sim_SetCounter<=9)
					Menu_sim_Code[Sim_SetFlag-1]=Sim_SetCounter+'0';
				
				Sim_SetFlag++;	
				Sim_SetCounter=0;
				Sim_Set(0);
				}		
			if(Sim_SetFlag==13)
				{
				lcd_fill(0);
				Sim_SetFlag=14;
				lcd_text12(0,5,(char *)Menu_sim_Code,12,LCD_MODE_SET);
				lcd_text12(13,19,"SIM卡号设置完成",15,LCD_MODE_SET);
				lcd_update_all();		
				//写入车辆 ID 号码
                         CarSet_0_counter=4;
				}
			else if(Sim_SetFlag==14)
				{
				Sim_SetFlag=1;
				pMenuItem=&Menu_0_loggingin;
				pMenuItem->show();
				}
			
			break;
		case KeyValueUP:
                     if((Sim_SetFlag>=1)&&(Sim_SetFlag<=12))
                     	{
                     	if(Sim_SetCounter==0)
					Sim_SetCounter=9;
				else if(Sim_SetCounter>=1)
					Sim_SetCounter--;
				Sim_Set(Sim_SetCounter);
                     	}
			break;
		case KeyValueDown:
                    if((Sim_SetFlag>=1)&&(Sim_SetFlag<=12)) 
                     	{
	                     	Sim_SetCounter++;
	                     	if(Sim_SetCounter>9)
						        Sim_SetCounter=0;
					         Sim_Set(Sim_SetCounter);	 
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

     Sim_SetFlag=1;
     Sim_SetCounter=0;*/
}


MENUITEM	Menu_0_3_Sim=
{
       "设置手机号码",
	12,
	&show,
	&keypress,
	&timetick,
	(void*)0
};


