
#include "App_moduleConfig.h"
#include "DF_Oper.h"
#include "DF_Function.h"  



void Write_IPandPort_Setting(void)
{
   u8 reg[20];
   
            memset(reg,0,sizeof(reg));
	  
   	        reg[0]=RemoteIP_main[0];
			reg[1]=RemoteIP_main[1];
			reg[2]=RemoteIP_main[2];
			reg[3]=RemoteIP_main[3];
			reg[4]=(u8)(RemotePort_main>>8);	
			reg[5]=(u8)(RemotePort_main);  
					
			reg[6]=RemoteIP_aux[0];
			reg[7]=RemoteIP_aux[1];
			reg[8]=RemoteIP_aux[2];
			reg[9]=RemoteIP_aux[3];
			reg[10]=(u8)(RemotePort_aux>>8);	
			reg[11]=(u8)(RemotePort_aux);			

			   
			
		    DF_WriteFlashSector(DF_socket_all,0,reg,12);   
}

void  Read_IPandPort_Setting(void)
{
    u8 reg[20];

   memset(reg,0,sizeof(reg)); 
   
   DF_ReadFlash(DF_socket_all,0,reg,20);
   RemoteIP_main[0]=reg[0];
   RemoteIP_main[1]=reg[1];
   RemoteIP_main[2]=reg[2]; 
   RemoteIP_main[3]=reg[3];	
   RemotePort_main=((u16)reg[4]<<8)+(u16)reg[5];  
   
   RemoteIP_aux[0]=reg[6];
   RemoteIP_aux[1]=reg[7];
   RemoteIP_aux[2]=reg[8]; 
   RemoteIP_aux[3]=reg[9];	
   RemotePort_aux=((u16)reg[10]<<8)+(u16)reg[11];  


}

//===============================================================
u8  DF_Write_RecordAdd(u32 Wr_Address,u32 Rd_Address, u8 Type) 
{
  u8     head[448];
  u16    offset=0;  
  u16    Add_offset=0;  //  page offset  
  u16    Savepage=0;      // �洢page ҳ
  u16    InpageOffset=0;  // ҳ��ƫ�� 
  u8     reg[9];  
  u8     Flag_used=0x01;
  
  //  1.   Classify
    switch(Type)
    {
		case TYPE_CycleAdd:
							 Add_offset=DF_CycleAdd_Page;
							 break;
		case TYPE_PhotoAdd:
							 Add_offset=DF_PhotoAdd_Page;
							 break;
		case TYPE_TiredDrvAdd:
							 Add_offset=DF_TiredDrvAdd_Page;
							 break;
		case TYPE_ExpSpdAdd:
							 Add_offset=DF_ExpSpdAdd_Page;
							 break; 					 
		case TYPE_AccFireAdd:
							 Add_offset=DF_AccFireRecAdd_Page;
							 break;
		case TYPE_AvrgSpdAdd:
							 Add_offset=DF_AvrgSpdPerMinAdd_Page;
							 break;
		case TYPE_ErrorLogAdd:
							 Add_offset=DF_AbnormalLogAdd_Page;
							 break;
		case TYPE_VechRecordAdd:
							 Add_offset=DF_RecordAdd_Page;	
							 break;
		case TYPE_DoubtAdd:
                             Add_offset=DF_DoubtAdd_Page; 
							 break;
		case TYPE_AvrgSpdSecAdd:
                             Add_offset=DF_AvrgSpdSec_Page;
			                 break;
		case TYPE_LogInAdd:
                             Add_offset=DF_Login_Page;
			                 break;
		case TYPE_PowerCutAdd:
                             Add_offset=DF_Powercut_Page;
			                 break;	
		case TYPE_SettingChgAdd:
                             Add_offset=DF_Settingchg_Page;
			                 break;		
		case TYPE_MintPosAdd:
			                 Add_offset=DF_Minpos_Page;   
			                 break; 
		case TYPE_DayDistancAdd:
			                 Add_offset=DF_DayDistance_Page;
							 break;
		case TYPE_ACCONFFcounterAdd:
			                 Add_offset=DF_ACCONFFcounter_Page;
							 break;
							 
		default :
							 return false;							 
    }
  //  2 .  Excute 
    
     DF_ReadFlash(Add_offset,0,(unsigned char*)head,448);   
     DF_delay_us(100);
     //    debug 
	/* rt_kprintf("\r\n ��ȡ״̬��448 :\r\n");
	 for(offset=0;offset<448;offset++)
     {
       rt_kprintf("%x ",head[offset]);     
	 }
	*/	
	
	/*
	
      ͨ����ѯBlock ��1��Page��ǰ448�ֽ��Ƿ�Ϊ0xFF �жϣ������Ҫд�����ݵ�ƫ�Ƶ�ַ����448����ʶʹ����󣬲�����Block��
      Ȼ���ͷ��ʼ��
	 ����ÿ��page�ܴ�64�����ݣ�����Ҫ�ȼ���洢��PageȻ���ټ���ƫ�Ƶ�ַ
	  �洢page�ļ��㷽��Ϊ �� 
	        Savepage=Startpage+n/64;
	 �洢ҳ�ڵ�ƫ�Ƶ�ַ���㷽��Ϊ��
		   InpageOffset=��n%64��*8��

	*/ 		
     for(offset=0;offset<448;offset++)
     {
       if(0xFF==head[offset])
	   	  break;
     }

	 if(offset==448)
	 	{     
		   SST25V_SectorErase_4KByte((8*((u32)Add_offset/8))*PageSIZE); // Erase block
		   offset=0; 		   
		   DF_delay_ms(50);
	    }   
	 Savepage=Add_offset+1+(offset>>6);   //Add_offset+offset/64  
	 InpageOffset=((offset%64)<<3);      //(offset%64��*8;    

     
	 memcpy(reg,&Wr_Address,4);  
	 memcpy(reg+4,&Rd_Address,4);   
	 
	 // rt_kprintf("\r\n write=%d ,read=%d\r\n",Wr_Address,Rd_Address);   
	 DF_WriteFlashDirect(Add_offset,offset,&Flag_used,1); //  ����״̬λ
	 DF_delay_us(100); 
	 DF_WriteFlashDirect(Savepage,InpageOffset,reg,8);    //  ���¼�¼����
	 
	 return true;
   //                 The  End     	     
}


u8  DF_Read_RecordAdd(u32 Wr_Address,u32 Rd_Address, u8 Type)
{
  u8	   head[448];
  u16    offset=0;  
  u32   Add_offset=0;  //  page offset  
  u32   Reg_wrAdd=0,Reg_rdAdd=0;  
  u16    Savepage=0;      // �洢page ҳ
  u16    InpageOffset=0;  // ҳ��ƫ�� 

   //Wr_Address, Rd_Address  , û��ʲô��ֻ�Ǳ�ʾ���룬����д�۲����    
  //  1.   Classify
    switch(Type)
    {
       case TYPE_CycleAdd:
                            Add_offset=DF_CycleAdd_Page;
	                        break;
	   case TYPE_PhotoAdd:
	   	                    Add_offset=DF_PhotoAdd_Page;
	                        break;
	   case TYPE_TiredDrvAdd:
	   	                    Add_offset=DF_TiredDrvAdd_Page;
	                        break;
	   case TYPE_ExpSpdAdd:
	   	                    Add_offset=DF_ExpSpdAdd_Page;
	                        break;						
	   case TYPE_AccFireAdd:
	   	                    Add_offset=DF_AccFireRecAdd_Page;
	                        break;
	   case TYPE_AvrgSpdAdd:
	   	                    Add_offset=DF_AvrgSpdPerMinAdd_Page;
	                        break;
       case TYPE_ErrorLogAdd:
	   	                    Add_offset=DF_AbnormalLogAdd_Page;
	                        break;
       case TYPE_VechRecordAdd:
	   	                    Add_offset=DF_RecordAdd_Page;  
	                        break;
	   case TYPE_DoubtAdd:
							Add_offset=DF_DoubtAdd_Page; 
							break;	
		case TYPE_AvrgSpdSecAdd:
                             Add_offset=DF_AvrgSpdSec_Page;
			                 break;
		case TYPE_LogInAdd:
                             Add_offset=DF_Login_Page;
			                 break;
		case TYPE_PowerCutAdd:
                             Add_offset=DF_Powercut_Page;
			                 break;	
		case TYPE_SettingChgAdd:
                             Add_offset=DF_Settingchg_Page;
			                 break;	 	
		case TYPE_MintPosAdd:
		 				     Add_offset=DF_Minpos_Page;  
							 break;
		case TYPE_DayDistancAdd:
			                 Add_offset=DF_DayDistance_Page;
							 break;
		case TYPE_ACCONFFcounterAdd:
			                 Add_offset=DF_ACCONFFcounter_Page;
							 break;	  	 			 
						 
							 
	   default :
	   	                    return false;							

    }
  //  2 .  Excute 
    
     DF_ReadFlash(Add_offset,0,(unsigned char*)head,448); //   ������Ϣ  
		
		/*
		
		  ͨ����ѯBlock ��1��Page��ǰ448�ֽ��Ƿ�Ϊ0xFF �жϣ������Ҫд�����ݵ�ƫ�Ƶ�ַ����448����ʶʹ����󣬲�����Block��
		  Ȼ���ͷ��ʼ��
		 ����ÿ��page�ܴ�64�����ݣ�����Ҫ�ȼ���洢��PageȻ���ټ���ƫ�Ƶ�ַ
		  �洢page�ļ��㷽��Ϊ �� 
				Savepage=Startpage+n/64;
		 �洢ҳ�ڵ�ƫ�Ƶ�ַ���㷽��Ϊ��
			   InpageOffset=��n%64��*8��
		
		*/

   
		for(offset=0;offset<448;offset++)
		{
		  if(0xFF==head[offset])
			 break;
		}

		offset--;  // ��һ������Ϊ0xFF

		
		Savepage=Add_offset+1+(offset>>6);	 //Add_offset+offset/64  
		InpageOffset=((offset%64)<<3); 	 //(offset%64��*8;	  
		//rt_kprintf("\r\n Read	offset=%d\r\n", offset);
		DF_ReadFlash(Savepage,InpageOffset,(u8*)&Reg_wrAdd,4);
		DF_ReadFlash(Savepage,InpageOffset+4,(u8*)&Reg_rdAdd,4);    

	
	    // rt_kprintf("\r\n  RecordAddress  READ-1   write=%d , read=%d \r\n",Reg_wrAdd,Reg_rdAdd);    

	
  //  3. Get reasult 
    switch(Type)
    {
       case TYPE_CycleAdd:
                            cycle_write=Reg_wrAdd;
							cycle_read=Reg_rdAdd;
	                        break;
	   case TYPE_PhotoAdd:
	   	                    pic_write=Reg_wrAdd;
							pic_read=Reg_rdAdd;
	                        break;
	   case TYPE_TiredDrvAdd:
	   	                    TiredDrv_write=Reg_wrAdd;
							TiredDrv_read=Reg_rdAdd;
	                        break;
	   case TYPE_ExpSpdAdd:
	   	                    ExpSpdRec_write=Reg_wrAdd;
							ExpSpdRec_read=Reg_rdAdd;
	                        break;						
	   case TYPE_AccFireAdd:
	   	                    OnFireRec_write=Reg_wrAdd;
							OnFireRec_read=Reg_rdAdd;
	                        break;
	   case TYPE_AvrgSpdAdd:
	   	                    AvrgSpdPerMin_write=Reg_wrAdd;
							AvrgSpdPerMin_Read=Reg_rdAdd;
	                        break;
       case TYPE_ErrorLogAdd:
	   	                    ErrorLog_write=Reg_wrAdd;
							ErrorLog_Read=Reg_rdAdd;
	                        break;
       case TYPE_VechRecordAdd:
	   	                    Recorder_write=Reg_wrAdd;
							Recorder_Read=Reg_rdAdd;           
	                        break;
	 	case TYPE_AvrgSpdSecAdd:
			                 AvrgSpdPerSec_write=Reg_wrAdd;
	 						 AvrgSpdPerSec_Read=Reg_rdAdd;
	 						 break;
	 	case TYPE_LogInAdd:
			                 Login_write=Reg_wrAdd;
	 						 Login_Read=Reg_rdAdd;
	 						 break;
	 	case TYPE_PowerCutAdd:
	 						 Powercut_write=Reg_wrAdd;
	 						 Powercut_read=Reg_rdAdd;
	 						 break; 
	 	case TYPE_SettingChgAdd:
	 						 Settingchg_write=Reg_wrAdd;
	 						 Settingchg_read=Reg_rdAdd;
	 						 break; 	   						
	    case TYPE_MintPosAdd: 
						    AvrgMintPosit_write=Reg_wrAdd;
						    AvrgMintPosit_Read=Reg_rdAdd;   
						    break;
		case TYPE_DayDistancAdd:
			                Distance_m_u32=Reg_wrAdd;
						    DayStartDistance_32=Reg_rdAdd;         
							break;
		case TYPE_ACCONFFcounterAdd:
			                //Tired_drive.ACC_ONstate_counter=Reg_wrAdd;
						//	Tired_drive.ACC_Offstate_counter=Reg_rdAdd;      
							break;					
						
	   						
	   default :
	   	                    return false;							

    } 
   //                 The  End     	     
    return true;
}






