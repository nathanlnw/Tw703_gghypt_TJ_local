/*
     SMS.C 
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
#include  "App_moduleConfig.h"
#include  "App_gsm.h"
#include  "SMS.h"


SMS_Style   SMS_Service;    //  ��Ϣ���    


/*********************************************************************************
*��������:void SMS_timer(u8 *instr,u16 len)
*��������:���Ŵ����������������Ҫ��һ��1��Ķ�ʱ��������ã����ں���"SMS_Process"�Ķ�ʱ�����
*��    ��:none
*��    ��:none 
*�� �� ֵ:none
*��    ��:������
*��������:2013-05-29
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
void SMS_timer(void)
{
	if(SMS_Service.SMS_waitCounter)
		SMS_Service.SMS_waitCounter--;
	/*
	if(SMS_Service.SMS_delALL>1)
		{
		SMS_Service.SMS_delALL--;
		}
		*/
	 //-------- ������� ------------------
	 /*
	 if(SMS_Service.SMS_come==1)
	 {
		 SMS_Service.SMS_delayCounter++;
		 if(SMS_Service.SMS_delayCounter)
		   {
			 SMS_Service.SMS_delayCounter=0;
			 SMS_Service.SMS_come=0;
			 SMS_Service.SMS_read=3;	  // ʹ�ܶ�ȡ
		   }
	 }
	 */
}


/*********************************************************************************
*��������:void SMS_protocol(u8 *instr,u16 len)
*��������:���Ŵ����������������Ҫ��һ���߳�������ã�������ش���(���Ŷ�ȡ��ɾ�����Զ��������)
*��    ��:none
*��    ��:none 
*�� �� ֵ:none
*��    ��:������
*��������:2013-05-29
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
void SMS_Process(void) 
{
	u16   	ContentLen=0;
	u16 		i,j,k;
	char *pstrTemp;
	if(SMS_Service.SMS_waitCounter)
		return;
	//-----------  ���Ŵ������ -------------------------------------------------------- 
	//---------------------------------
	if(SMS_Service.SMS_read)	   // ��ȡ����
	{
		memset(SMS_Service.SMSAtSend,0,sizeof(SMS_Service.SMSAtSend));
		/*
		strcpy( ( char * ) SMS_Service.SMSAtSend, "AT+CMGR=" );	  
		if ( SMS_Service.SMIndex > 9 )
		{
			SMS_Service.SMSAtSend[8] = ( SMS_Service.SMIndex >> 4 ) + 0x30;
			SMS_Service.SMSAtSend[9] = ( SMS_Service.SMIndex & 0x0f ) + 0x30;
			SMS_Service.SMSAtSend[10] = 0x0d;
			SMS_Service.SMSAtSend[11] = 0x0a;
		}
		else
		{
			SMS_Service.SMSAtSend[8] = ( SMS_Service.SMIndex & 0x0f ) + 0x30;
			SMS_Service.SMSAtSend[9] = 0x0d;
			SMS_Service.SMSAtSend[10] = 0x0a;
		}
		rt_kprintf("\r\n%s",SMS_Service.SMSAtSend); 
		*/
		///
		sprintf(SMS_Service.SMSAtSend,"AT+CMGR=%d\r\n",SMS_Service.SMIndex);
		rt_kprintf("%s",SMS_Service.SMSAtSend); 
		///
		rt_hw_gsm_output( ( char * ) SMS_Service.SMSAtSend );   		
		SMS_Service.SMS_read--;  
		SMS_Service.SMS_waitCounter=3;
	}
	//-------------------------------
	//       ���Ͷ�Ϣȷ��
	else
	if(SMS_Service.SMS_sendFlag==1)
	{
		//#ifdef SMS_TYPE_PDU
		if(SMS_Service.SMS_come==1)
		{
			memset(SMS_Service.SMSAtSend,0,sizeof(SMS_Service.SMSAtSend));
			///����400�ֽڿռ�
			pstrTemp=rt_malloc(400);
			memset(pstrTemp,0,400);
			///���ַ�����ʽ��Ŀ�ĵ绰��������ΪPDU��ʽ�ĺ���
			SetPhoneNumToPDU(SMS_Service.Sms_Info.TPA, SMS_Service.SMS_destNum, sizeof(SMS_Service.Sms_Info.TPA));
			///����PDU��ʽ��������
			ContentLen=AnySmsEncode_NoCenter(SMS_Service.Sms_Info.TPA,GSM_UCS2,SMS_Service.SMS_sd_Content,strlen(SMS_Service.SMS_sd_Content),pstrTemp);
			//ContentLen=strlen(pstrTemp);
			///��Ӷ���β�����"esc"
			pstrTemp[ContentLen]=0x1A;      // message  end  	
			//////
			sprintf( ( char * ) SMS_Service.SMSAtSend, "AT+CMGS=%d\r\n", (ContentLen-2)/2); 
			rt_kprintf("%s",SMS_Service.SMSAtSend); 
			rt_hw_gsm_output( ( char * ) SMS_Service.SMSAtSend );
			rt_thread_delay(50);
			//////	
			//rt_kprintf("%s",pstrTemp); 
			rt_device_write( &dev_vuart, 0, pstrTemp, strlen(pstrTemp) );
			rt_hw_gsm_output_Data( ( char * ) pstrTemp, ContentLen+1); 
			rt_free( pstrTemp );
			pstrTemp=RT_NULL;
		}
		//#else
		else
		{
			memset(SMS_Service.SMSAtSend,0,sizeof(SMS_Service.SMSAtSend));
			strcpy( ( char * ) SMS_Service.SMSAtSend, "AT+CMGS=\"" ); 
			//strcat(SMS_Service.SMSAtSend,"8613820554863");// Debug
			strcat(SMS_Service.SMSAtSend,SMS_Service.SMS_destNum);
			strcat(SMS_Service.SMSAtSend,"\"\r\n");

			rt_kprintf("\r\n%s",SMS_Service.SMSAtSend); 
			rt_hw_gsm_output( ( char * ) SMS_Service.SMSAtSend );

			rt_thread_delay(50);
			ContentLen=strlen(SMS_Service.SMS_sd_Content);
			SMS_Service.SMS_sd_Content	[ContentLen]=0x1A;      // message  end  		
			rt_kprintf("%s",SMS_Service.SMS_sd_Content); 
			rt_hw_gsm_output_Data( ( char * ) SMS_Service.SMS_sd_Content, ContentLen+1); 
		}
		//#endif
		SMS_Service.SMS_sendFlag=0;  // clear 
		SMS_Service.SMS_waitCounter=3;
	} 
	/*
	else if(SMS_Service.SMS_delALL==1)	  //ɾ������
	{	   
		memset(SMS_Service.SMSAtSend,0,sizeof(SMS_Service.SMSAtSend));
		///
		//sprintf(SMS_Service.SMSAtSend,"AT+CMGD=%d\r\n",SMS_Service.SMIndex);
		sprintf(SMS_Service.SMSAtSend,"AT+CMGD=0,4\r\n",SMS_Service.SMIndex);
		rt_kprintf("%s",SMS_Service.SMSAtSend); 
		///
		rt_hw_gsm_output( ( char * )SMS_Service.SMSAtSend ); 
		SMS_Service.SMS_delALL=0; 
		SMS_Service.SMS_waitCounter=3;
	}
	*/
}


///���ӷ��Ͷ�����������ݣ�����λ���Ͷ�Ϣ��ǣ��ɹ�����true��ʧ�ܷ���false
u8 Add_SMS_Ack_Content(char * instr,u8 ACKflag)
{
    if(ACKflag==0)
		 return false;  
     
	if(strlen(instr)+strlen(SMS_Service.SMS_sd_Content) < sizeof(SMS_Service.SMS_sd_Content))
		{
		 	strcat((char *)SMS_Service.SMS_sd_Content,instr);
			//rt_kprintf(" \r\n  enable  send msg! \r\n"); 
		 	SMS_Service.SMS_sendFlag=1;
			return true;
		}
	return false;
}

/*********************************************************************************
*��������:void SMS_protocol(u8 *instr,u16 len)
*��������:���յ����ź����������
*��    ��:instrԭʼ�������ݣ�len����
*��    ��:none 
*�� �� ֵ:none
*��    ��:������
*��������:2013-05-29
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
void   SMS_protocol (u8 *instr,u16 len, u8  ACKstate)   //  ACKstate 
{
	char	sms_content[60];		///����������"()"֮�������
	char	sms_ack_data[60];		///����ÿ���������'#'������������
	u8	u8TempBuf[6];
	u16	i=0,j=0;
	u16  cmdLen,u16Temp;
	char *p_Instr;
	char *pstrTemp,*pstrTempStart,*pstrTempEnd;

	//SYSID		///�޸ĸ�ֵ������flash
	///Ӧ����Ű�ͷ����
	memset(SMS_Service.SMS_sd_Content,0,sizeof(SMS_Service.SMS_sd_Content));
	strcpy(SMS_Service.SMS_sd_Content,JT808Conf_struct.Vechicle_Info.Vech_Num);
	strcat(SMS_Service.SMS_sd_Content,"#");// Debug
	strcat(SMS_Service.SMS_sd_Content,DeviceNumberID);// Debug
	/*************************������Ϣ****************************/
	p_Instr=(char *)instr;
	for(i=0;i<len;i++)
		{
		pstrTemp=strchr(p_Instr,'#');					///���������Ƿ����
		//instr++;
		if(pstrTemp)
			{
			p_Instr=pstrTemp+1;
			pstrTempStart=strchr((char *)pstrTemp,'(');			///�����������ݿ�ʼλ��
			pstrTempEnd=strchr((char *)pstrTemp,')');			///�����������ݽ���λ��
			if((NULL==pstrTempStart)||(NULL==pstrTempEnd))
				{
				break;
				}
			rt_kprintf("\r\n���������ʽ��Ч !");
			///��ȡ��������
			memset(sms_ack_data,0,sizeof(sms_ack_data));
			memcpy(sms_ack_data,pstrTemp,pstrTempEnd-pstrTemp+1);

			///��ȡ�����������������Լ���������
			pstrTempStart++;
			pstrTemp++;
			cmdLen=pstrTempEnd-pstrTempStart;
			memset(sms_content,0,sizeof(sms_content));
			rt_memcpy(sms_content,pstrTempStart,cmdLen);

			///��������ƥ��
			if(strncmp(pstrTemp,"DNSR",4)==0)				///  1. ��������
				{
				if(cmdLen<=sizeof(DomainNameStr))
					{
					if(pstrTemp[4]=='1')		///������
						{
						rt_kprintf("\r\n���������� !");
						memset(DomainNameStr,0,sizeof(DomainNameStr));					  
						memset(SysConf_struct.DNSR,0,sizeof(DomainNameStr));  
						memcpy(DomainNameStr,(char*)pstrTempStart,cmdLen);
						memcpy(SysConf_struct.DNSR,(char*)pstrTempStart,cmdLen);
						Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
						//----- ���� GSM ģ��------
						DataLink_DNSR_Set(SysConf_struct.DNSR,1); 
						
						///
						Add_SMS_Ack_Content(sms_ack_data,ACKstate);

						//------- add on 2013-6-6
						if(ACKstate==SMS_ACK_none)
						     SD_ACKflag.f_CentreCMDack_0001H=2 ;//DataLink_EndFlag=1; //AT_End(); �ȷ��ؽ���ٹҶ�  
						else
							   DataLink_EndFlag=1; //AT_End();  
							   
                         //--------    �����Ȩ�� -------------------
					     idip("clear");		   

						}
					else if(pstrTemp[4]=='2')	///��������
						{
						rt_kprintf("\r\n���ñ������� !");
						memset(DomainNameStr_aux,0,sizeof(DomainNameStr_aux));					  
						memset(SysConf_struct.DNSR_Aux,0,sizeof(DomainNameStr_aux));
						memcpy(DomainNameStr_aux,(char*)pstrTempStart,cmdLen);
						memcpy(SysConf_struct.DNSR_Aux,(char*)pstrTempStart,cmdLen);
						Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
						//----- ���� GSM ģ��------
						DataLink_DNSR2_Set(SysConf_struct.DNSR_Aux,1);
						
						///
						Add_SMS_Ack_Content(sms_ack_data,ACKstate);
						}
					else
						{
						continue;
						}
					}
				}
			else if(strncmp(pstrTemp,"PORT",4)==0)			///2. ���ö˿�
				{
				j=sscanf(sms_content,"%u",&u16Temp);
				if(j)
					{
					if(pstrTemp[4]=='1')		///���˿�
						{
						rt_kprintf("\r\n�������˿�=%d!",u16Temp);
						RemotePort_main=u16Temp;
						SysConf_struct.Port_main=u16Temp;
						Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
						//----- ���� GSM ģ��------
						DataLink_MainSocket_set(RemoteIP_main,RemotePort_main,1);
						///
						Add_SMS_Ack_Content(sms_ack_data,ACKstate);

						//------- add on 2013-6-6
						if(ACKstate==SMS_ACK_none)
						     SD_ACKflag.f_CentreCMDack_0001H=2 ;//DataLink_EndFlag=1; //AT_End(); �ȷ��ؽ���ٹҶ�  
						else
							  DataLink_EndFlag=1; //AT_End();  

						 //--------    �����Ȩ�� -------------------
					     idip("clear");		   	  

						}
					else if(pstrTemp[4]=='2')	///���ö˿�
						{
						rt_kprintf("\r\n���ñ��ö˿�=%d!",u16Temp);
						RemotePort_aux=u16Temp;
						SysConf_struct.Port_Aux=u16Temp;
						Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
						//----- ���� GSM ģ��------
						DataLink_AuxSocket_set(RemoteIP_aux,RemotePort_aux,1);
						///
						Add_SMS_Ack_Content(sms_ack_data,ACKstate);
						}
					else
						{
						continue;
						}
					}
				
				}
			else if(strncmp(pstrTemp,"DUR",3)==0)				///3. �޸ķ��ͼ��
				{
				j=sscanf(sms_content,"%u",&u16Temp);
				if(j)
					{
					
					rt_kprintf("\r\n�޸ķ��ͼ��! %d",u16Temp);
					dur(sms_content);
					/*
					JT808Conf_struct.DURATION.Default_Dur=u16Temp;
					Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					*/

					///
					Add_SMS_Ack_Content(sms_ack_data,ACKstate);
					}
				}
			else if(strncmp(pstrTemp,"DEVICEID",8)==0)			///4. �޸��ն�ID
				{
				if(cmdLen<=sizeof(DeviceNumberID))
					{
					rt_kprintf("\r\n�޸��ն�ID  !");
					memset(DeviceNumberID,0,sizeof(DeviceNumberID));
					memcpy(DeviceNumberID,pstrTempStart,cmdLen);
					DF_WriteFlashSector(DF_DeviceID_offset,0,DeviceNumberID,13); 
					///
					Add_SMS_Ack_Content(sms_ack_data,ACKstate);

					 //--------    �����Ȩ�� -------------------
					     idip("clear");		     
					
					}
				else
					{
					       continue;
					}
				}    //DeviceNumberID			
			else if(strncmp(pstrTemp,"IP",2)==0)				///5.����IP��ַ
				{
				j = sscanf(sms_content, "%u.%u.%u.%u", (u32*)&u8TempBuf[0], (u32*)&u8TempBuf[1], (u32*)&u8TempBuf[2], (u32*)&u8TempBuf[3]);
				//j=str2ip(sms_content, u8TempBuf);
				if(j==4)
				 	{
				 	rt_kprintf("\r\n����IP��ַ!");
					if(pstrTemp[2]=='1')
						{ 
						memcpy(SysConf_struct.IP_Main,u8TempBuf,4);
						memcpy(RemoteIP_main,u8TempBuf,4);
						SysConf_struct.Port_main=RemotePort_main;
						Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
						rt_kprintf("\r\n���������������� IP: %d.%d.%d.%d : %d ",RemoteIP_main[0],RemoteIP_main[1],RemoteIP_main[2],RemoteIP_main[3],RemotePort_main);
						//-----------  Below add by Nathan  ----------------------------
						DataLink_MainSocket_set(RemoteIP_main,RemotePort_main,1);
						///
						Add_SMS_Ack_Content(sms_ack_data,ACKstate);

					     //------- add on 2013-6-6
						if(ACKstate==SMS_ACK_none)
						      SD_ACKflag.f_CentreCMDack_0001H=2 ;//DataLink_EndFlag=1; //AT_End(); �ȷ��ؽ���ٹҶ�  
						else
							   DataLink_EndFlag=1;

						 //--------    �����Ȩ�� -------------------
					     idip("clear");		   

						}
					else if(pstrTemp[2]=='2')
						{
						memcpy(SysConf_struct.IP_Aux,u8TempBuf,4);
						memcpy(RemoteIP_aux,u8TempBuf,4);
						SysConf_struct.Port_Aux=RemotePort_aux;
						Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
						rt_kprintf("\r\n�������ñ��÷����� IP: %d.%d.%d.%d : %d ",RemoteIP_aux[0],RemoteIP_aux[1],RemoteIP_aux[2],RemoteIP_aux[3],RemotePort_aux);   
						//-----------  Below add by Nathan  ----------------------------
						DataLink_AuxSocket_set(RemoteIP_aux,RemotePort_aux,1);
						///
						Add_SMS_Ack_Content(sms_ack_data,ACKstate);
						}
					}
				}
			else if(strncmp(pstrTemp,"MODE",4)==0)			///6. ���ö�λģʽ
				{
				        if(strncmp(sms_content,"BD",2)==0)
				        {
                                                 gps_mode("1");
				        }
					  if(strncmp(sms_content,"GP",2)==0)
				        {
                                                 gps_mode("2");
				        }	
					   if(strncmp(sms_content,"GN",2)==0)
				        {
                                                 gps_mode("3");  
				        }
					   Add_SMS_Ack_Content(sms_ack_data,ACKstate);
				}
			else if(strncmp(pstrTemp,"VIN",3)==0)				///7.���ó���VIN
				{
				     vin_set(sms_content);
				    Add_SMS_Ack_Content(sms_ack_data,ACKstate);
				}
			else if(strncmp(pstrTemp,"TIREDCLEAR",10)==0)		///8.���ƣ�ͼ�ʻ��¼
				{
				      TiredDrv_write=0;
				      TiredDrv_read=0;	   
				      DF_Write_RecordAdd(TiredDrv_write,TiredDrv_read,TYPE_TiredDrvAdd);      
				      Add_SMS_Ack_Content(sms_ack_data,ACKstate);	  
				}
			else if(strncmp(pstrTemp,"DISCLEAR",8)==0)			///9������
				{
				   	  JT808Conf_struct.DayStartDistance_32=0;
					  JT808Conf_struct.Distance_m_u32=0;
                                     Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
					   Add_SMS_Ack_Content(sms_ack_data,ACKstate);				 
				}
			else if(strncmp(pstrTemp,"RESET",5)==0)			///10.�ն�����
				{
				      reset();
				}
			else if(strncmp(pstrTemp,"RELAY",5)==0)			///11.�̵�������
				{
				       if(sms_content[0]=='0')
				              debug_relay("0");
				       if(sms_content[0]=='1')
					   	debug_relay("1"); 

					Add_SMS_Ack_Content(sms_ack_data,ACKstate);	    
				}
			else if(strncmp(pstrTemp,"TAKE",4)==0)				//12./����
				{
				    switch(sms_content[0])
				   {
                                    case '1':
                                                 takephoto("1");
							break;		
					case '2':
                                                  takephoto("2"); 
							break;		
				       case '3':
                                                takephoto("3");
							break;	
				       case '4':
                                                  takephoto("4");
							break;
				   }
				    Add_SMS_Ack_Content(sms_ack_data,ACKstate);	
				}
			else if(strncmp(pstrTemp,"PLAY",4)==0)				///13.��������
				{
				      TTS_Get_Data(sms_content,strlen(sms_content));
				     Add_SMS_Ack_Content(sms_ack_data,ACKstate);		  
				}
			else if(strncmp(pstrTemp,"QUERY",5)==0)			///14.����״̬��ѯ
				{
				       switch(sms_content[0])
				     	{
				     	    case 0:  

								     break;
							case 1:

								     break;
                            case 2:

								     break;
				     	}				
				        Add_SMS_Ack_Content(sms_ack_data,ACKstate);	  
				}
			else if(strncmp(pstrTemp,"ISP",3)==0)				///15.Զ������IP �˿�
				{
				     ;
				    Add_SMS_Ack_Content(sms_ack_data,ACKstate);		 
				}
			else if(strncmp(pstrTemp,"PLATENUM",8)==0)
				{
				    rt_kprintf("Vech_Num is %s", sms_content);
					memset((u8*)&JT808Conf_struct.Vechicle_Info.Vech_Num,0,sizeof(JT808Conf_struct.Vechicle_Info.Vech_Num));	//clear	
				    rt_memcpy(JT808Conf_struct.Vechicle_Info.Vech_Num,sms_content,strlen(sms_content));
					Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					Add_SMS_Ack_Content(sms_ack_data,ACKstate);

					 //--------    �����Ȩ�� -------------------
					     idip("clear");		   
				}
			else if(strncmp(pstrTemp,"COLOUR",6)==0)
				{
				     j=sscanf(sms_content,"%d",&u16Temp);
					if(j)
					{
						
					JT808Conf_struct.Vechicle_Info.Dev_Color=u16Temp; 
	        		rt_kprintf("\r\n ������ɫ: %s ,%d \r\n",sms_content,JT808Conf_struct.Vechicle_Info.Dev_Color);          
	        		Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
				  	Add_SMS_Ack_Content(sms_ack_data,ACKstate);
					}
				}
			else if(strncmp(pstrTemp,"CONNECT",6)==0)
				{
				    j=sscanf(sms_content,"%d",&u16Temp);
					if(j)
					{
						
						JT808Conf_struct.Link_Frist_Mode=u16Temp;  
		        		rt_kprintf("\r\n �״����ӷ�ʽ %s ,%d \r\n",sms_content,JT808Conf_struct.Link_Frist_Mode);          
		        		Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
					  	Add_SMS_Ack_Content(sms_ack_data,ACKstate);  
						 //--------    �����Ȩ�� -------------------
					     idip("clear");		   
					} 
				}
           else if(strncmp(pstrTemp,"CLEARREGIST",11)==0)
           	    {
                     //--------    �����Ȩ�� -------------------
					     idip("clear");		
                     DEV_regist.Enable_sd=1; // set ����ע���־λ
			         DataLink_EndFlag=1; 
           	    }
			else												
				{
				;
				}
			}
		else
			{
			break;
			}
		}
}



/*********************************************************************************
*��������:u8 SMS_Rx_Text(char *instr,char *strDestNum)
*��������:���յ�TEXT��ʽ�Ķ��Ŵ�����
*��    ��:instr ԭʼ�������ݣ�strDestNum���յ�����Ϣ�ķ��ͷ�����
*��    ��:none 
*�� �� ֵ:	1:������ɣ�
			0:��ʾʧ��
*��    ��:������
*��������:2013-05-29
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
u8 SMS_Rx_Text(char *instr,char *strDestNum)
{
	u16 len;
	u8 ret=0;
	len=strlen(strDestNum);
	memset( SMS_Service.SMS_destNum, 0, sizeof( SMS_Service.SMS_destNum ) );
	if(len>sizeof( SMS_Service.SMS_destNum ))
		{
		len=sizeof( SMS_Service.SMS_destNum );
		}
	memcpy(SMS_Service.SMS_destNum,strDestNum,len);
	rt_kprintf( "\r\n  ��Ϣ��Դ����:%s", SMS_Service.SMS_destNum );
	
	len=strlen(instr);
	rt_kprintf( "\r\n �����յ���Ϣ: " );
	rt_device_write( &dev_vuart, 0, instr, len);
	
	if( strncmp( (char*)instr, "TW703#", 6 ) == 0 )                                                //�����޸�UDP��IP�Ͷ˿�
	{
		//-----------  �Զ��� ��Ϣ�����޸� Э�� ----------------------------------
		SMS_protocol( instr + 5, len - 5 ,SMS_ACK_msg);
		ret=1;
	}
	SMS_Service.SMS_read		= 0;
	SMS_Service.SMS_waitCounter = 0;
	SMS_Service.SMS_come		= 0;
	//SMS_Service.SMS_delALL		= 1;
	return ret;
}


/*********************************************************************************
*��������:u8 SMS_Rx_PDU(char *instr,u16 len)
*��������:���յ�PDU��ʽ�Ķ��Ŵ�����
*��    ��:instr ԭʼ�������ݣ�len���յ�����Ϣ���ȣ���λΪ�ֽ�
*��    ��:none 
*�� �� ֵ:	1:������ɣ�
			0:��ʾʧ��
*��    ��:������
*��������:2013-05-29
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
u8 SMS_Rx_PDU(char *instr,u16 len)
{
	char *pstrTemp;
	u8 ret=0;
	
	//////
	memset( SMS_Service.SMS_destNum, 0, sizeof( SMS_Service.SMS_destNum ) );
	pstrTemp=(char *)rt_malloc(200);	///���Ž������������ݣ��������ΪGB��
	memset(pstrTemp,0,200);
	rt_kprintf( "\r\n ����ԭʼ��Ϣ: " );
	rt_device_write( &dev_vuart, 0, GSM_rx, len );
	
	len=GsmDecodePdu(GSM_rx,len,&SMS_Service.Sms_Info,pstrTemp);
	GetPhoneNumFromPDU( SMS_Service.SMS_destNum,  SMS_Service.Sms_Info.TPA, sizeof(SMS_Service.Sms_Info.TPA));

	//memcpy( SMS_Service.SMS_destNum, SMS_Service.Sms_Info.TPA,sizeof( SMS_Service.SMS_destNum ) );
	rt_kprintf( "\r\n  ��Ϣ��Դ����:%s \r\n", SMS_Service.SMS_destNum );
	rt_kprintf( "\r\n ������Ϣ: " ); 
	rt_device_write( &dev_vuart, 0, pstrTemp, len );
	//rt_hw_console_output(GSM_rx);
	if( strncmp( (char*)pstrTemp, "TW703#", 6 ) == 0 )                                                //�����޸�UDP��IP�Ͷ˿�
	{
		//-----------  �Զ��� ��Ϣ�����޸� Э�� ----------------------------------
		SMS_protocol( pstrTemp + 5, len - 5 ,SMS_ACK_msg);
		ret=1;
	}
	SMS_Service.SMS_read		= 0;
	SMS_Service.SMS_waitCounter = 3;
	SMS_Service.SMS_come		= 1;
	//SMS_Service.SMS_delALL		= 1;
	rt_free( pstrTemp );
	pstrTemp = RT_NULL;
	//////
	return ret;
}




/*********************************************************************************
*��������:u8 SMS_Tx_Text(char *strDestNum,char *s)
*��������:����TEXT��ʽ�Ķ��ź���
*��    ��:s ԭʼ�������ݣ�strDestNum���շ�����
*��    ��:none 
*�� �� ֵ:	1:������ɣ�
			0:��ʾʧ��
*��    ��:������
*��������:2013-05-29
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
u8 SMS_Tx_Text(char *strDestNum,char *s) 
{
	u16 len;
	char *pstrTemp;
	
	memset(SMS_Service.SMSAtSend,0,sizeof(SMS_Service.SMSAtSend));
	strcpy( ( char * ) SMS_Service.SMSAtSend, "AT+CMGS=\"" ); 
	//strcat(SMS_Service.SMSAtSend,"8613820554863");// Debug
	strcat(SMS_Service.SMSAtSend,strDestNum);
	strcat(SMS_Service.SMSAtSend,"\"\r\n");

	rt_kprintf("\r\n%s",SMS_Service.SMSAtSend); 
	rt_hw_gsm_output( ( char * ) SMS_Service.SMSAtSend );

	rt_thread_delay(50);
	
	pstrTemp=rt_malloc(150);
	memset(pstrTemp,0,150);
	len=strlen(s);
	memcpy(pstrTemp,s,len);
	pstrTemp[len++]=0x1A;      // message  end  

	///���͵�����Ϣ
	rt_device_write( &dev_vuart, 0, pstrTemp, len);
	///���͵�GSMģ��
	rt_hw_gsm_output_Data(pstrTemp, len); 
	rt_free( pstrTemp );
	pstrTemp=RT_NULL; 
	return 1;
}
FINSH_FUNCTION_EXPORT(SMS_Tx_Text, SMS_Tx_Text);



/*********************************************************************************
*��������:u8 SMS_Tx_PDU(char *strDestNum,char *s)
*��������:����PDU��ʽ�Ķ��ź���
*��    ��:s ԭʼ�������ݣ�strDestNum���շ�����
*��    ��:none 
*�� �� ֵ:	1:������ɣ�
			0:��ʾʧ��
*��    ��:������
*��������:2013-05-29
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
u8 SMS_Tx_PDU(char *strDestNum,char *s)
{
	u16 len;
	u16 i;
	char *pstrTemp;
	memset(SMS_Service.SMSAtSend,0,sizeof(SMS_Service.SMSAtSend));
	pstrTemp=rt_malloc(400);
	memset(pstrTemp,0,400);
	i=0;
	SetPhoneNumToPDU(SMS_Service.Sms_Info.TPA, strDestNum, sizeof(SMS_Service.Sms_Info.TPA));
	len=AnySmsEncode_NoCenter(SMS_Service.Sms_Info.TPA,GSM_UCS2,s,strlen(s),pstrTemp);
	//len=strlen(pstrTemp);
	pstrTemp[len++]=0x1A;      // message  end  	
	//////
	sprintf( ( char * ) SMS_Service.SMSAtSend, "AT+CMGS=%d\r\n", (len-2)/2); 
	rt_kprintf("%s",SMS_Service.SMSAtSend); 
	rt_hw_gsm_output( ( char * ) SMS_Service.SMSAtSend );
	rt_thread_delay(50);
	//////	
	rt_device_write( &dev_vuart, 0, pstrTemp, strlen(pstrTemp) );
	//rt_hw_console_output(pstrTemp);
	rt_hw_gsm_output_Data(pstrTemp, len); 
	rt_free( pstrTemp );
	pstrTemp=RT_NULL;
	return 1;
}
FINSH_FUNCTION_EXPORT(SMS_Tx_PDU, SMS_Tx_PDU);


/*********************************************************************************
*��������:u8 SMS_Rx_Notice(u16 indexNum)
*��������:ģ���յ��¶���֪ͨ
*��    ��:�¶��ŵ�������
*��    ��:none 
*�� �� ֵ:	1:������ɣ�
			0:��ʾʧ��
*��    ��:������
*��������:2013-05-29
*---------------------------------------------------------------------------------
*�� �� ��:
*�޸�����:
*�޸�����:
*********************************************************************************/
u8 SMS_Rx_Notice(u16 indexNum)
{
	SMS_Service.SMIndex=indexNum;
	rt_kprintf( " index=%d", SMS_Service.SMIndex );
	SMS_Service.SMS_read		= 3;
	SMS_Service.SMS_waitCounter = 1;
	return 1;
}


///���Ժ���������SMS���Ž��մ��������������ȷ��
void SMS_Test(char * s)
{
	SMS_protocol(s,strlen(s),SMS_ACK_none);
}
FINSH_FUNCTION_EXPORT(SMS_Test, SMS_Test);

///���Ժ���������PDU���ݰ��Ľ��ս�������
void SMS_PDU(char *s)
{
	u16 len;
	u16 i,j;
	char *pstrTemp;
	pstrTemp=(char *)rt_malloc(160);	///���Ž������������ݣ��������ΪGB��
	len=GsmDecodePdu(s,strlen(s),&SMS_Service.Sms_Info,pstrTemp);
	GetPhoneNumFromPDU( SMS_Service.SMS_destNum,  SMS_Service.Sms_Info.SCA, sizeof(SMS_Service.Sms_Info.SCA));
	rt_kprintf( "\r\n  ��Ϣ���ĺ���:%s \r\n", SMS_Service.SMS_destNum );
	GetPhoneNumFromPDU( SMS_Service.SMS_destNum,  SMS_Service.Sms_Info.TPA, sizeof(SMS_Service.Sms_Info.TPA));
	rt_kprintf( "\r\n  ��Ϣ��Դ����:%s \r\n", SMS_Service.SMS_destNum );
	rt_kprintf( "\r\n ������Ϣ:\"%s\"\r\n",pstrTemp);
	rt_free( pstrTemp );
	pstrTemp = RT_NULL;
}
FINSH_FUNCTION_EXPORT(SMS_PDU, SMS_PDU);
