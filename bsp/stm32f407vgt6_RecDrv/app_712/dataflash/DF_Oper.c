/*****************************************************************
 MSP430 AT45DB041B 函数
*****************************************************************/
/*
#include <string.h>
*/
#include "App_moduleConfig.h"
#include "DF_Oper.h"

#define  DFBakSize   150//50
 
u8 SectorBuf_save[8][DFBakSize];//512bytes 一个单位              只用来存储补报信息 
static u8 SST25Temp[PageSIZE];// 
u8   DF_LOCK=0;    //Dataflash  Lock      



void DF_init(void)
{
     u32 device_id=0;	

      SST25V_Init();    

     SST25V_CS_LOW();
     SPI_Flash_SendByte(WriteDisable);
     SST25V_CS_HIGH();

     SST25V_CS_LOW();

    //-----erase chip-------------------
    //	SST25V_ChipErase();  
    //	DF_delay_ms(350);      
    //--------------------------------	
	
     SPI_Flash_SendByte( ReadJedec_ID  );
    device_id  = SPI_Flash_SendByte(0xFF)<<16;
    device_id = device_id | SPI_Flash_SendByte(0xFF)<<8;
    device_id = device_id | SPI_Flash_SendByte(0xFF);	 
    SST25V_CS_HIGH();

    if(device_id == 0xBF254A)//SST25VF032B = 0xBF254A,
    {
        rt_kprintf("FLASH TYPE : SST25VF032B\r\n");
        //rt_kprintf("FLASH:SST25VF032B\r\n"); 
        SST25V_CS_LOW();
        SPI_Flash_SendByte( DBSY );
        SST25V_CS_HIGH();

        SST25V_CS_LOW();
        SPI_Flash_SendByte( EnableWriteStatusRegister );
        SST25V_CS_HIGH();

        SST25V_CS_LOW();
        SPI_Flash_SendByte( WriteStatusRegister );
        SPI_Flash_SendByte( 0 );
        SST25V_CS_HIGH();
    }



}

void	mDelaymS( u8 ms )
{
	 unsigned int i;
  for(;ms>0;ms--)
     for(i=0;i<1200;i++);   //940 
}


void DF_delay_us(u16 j)
{
u8 i;
  while(j--)
  	{
  	i=3;
  	while(i--);
  	}  
}

void DF_delay_ms(u16 j)
{
  while(j--)
  	{
           DF_delay_us(2000);// 1000
  	} 
   
}

u8 DF_Spi_Tranbyte(u8 BYTE)
{
//USART_TxStr(USART1,"\r\nSpi_Tranbyte\r\n");
   /*
   BSPI1->TXR=(u16)(BYTE<<8);
   while(((BSPI1->CSR2)&0x10)!=0x10);
   return (BSPI1->RXR)>>8;
   */
//Set_AT45_CLK_1;

   while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
//USART_TxStr(USART1,"\r\nSpi_Tranbyte111111111\r\n");
//GPIO_SetBits(GPIOB,GPIO_Pin_3);
   SPI_I2S_SendData(SPI3,(u16)BYTE);
//USART_TxStr(USART1,"\r\nSpi_Tranbyte2222222222\r\n");
//   DF_delay_us(400);

//Set_AT45_CLK_0;
//GPIO_ResetBits(GPIOB,GPIO_Pin_3);
   while ( (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET ) );
   return SPI_I2S_ReceiveData(SPI3);
//   DF_delay_us(400);
//USART_TxStr(USART1,"\r\nSpi_Tranbyte_over\r\n");
}


void DF_Read_zk(u32 address,u8 *p,u16 length)//480 bytes 直接读取
{
    u16 i=0;
    
	for(i=0;i<length;i++)
	{
		*p=SST25V_ByteRead(address+i);
		p++;
	} 
}
 
void DF_ReadFlash(u16 page_counter,u16 page_offset,u8 *p,u16 length)
{   
	u16 i=0;
   
	
	for(i=0;i<length;i++)
	{
		 *p=SST25V_ByteRead(((u32)page_counter*PageSIZE)+(u32)(page_offset+i));//512bytes 一个单位
		 p++;
	}	
    //DF_delay_us(200);   
    DF_delay_ms(5);  
}

void DF_WriteFlash(u16 page_counter,u16 page_offset,u8 *p,u16 length)//512bytes 一个单位 
{
   u16 i=0,j=0,k=0;   //写补报

	for(i=0;i<8;i++)
	{
	    DF_ReadFlash((8*(page_counter/8))+i,0,SectorBuf_save[i],DFBakSize);//PageSIZE 
	}	
	
	SST25V_SectorErase_4KByte((8*((u32)page_counter/8))*PageSIZE);
	DF_delay_ms(80);          
	for(j=0;j<8;j++)
	{
	    if(j==(page_counter%8))
	    {
			for(i=0;i<length;i++)
			{			 
				SectorBuf_save[j][page_offset+i]=*p;    
				p++; 
			}			
	    } 	    	
		for(k=0;k<DFBakSize;k++)              
		{
			SST25V_ByteWrite(SectorBuf_save[j][k],((8*(page_counter/8))+j)*PageSIZE+k);  
		}		
	}  
	DF_delay_ms(20);  
} 
void DF_WriteFlashSector(u16 page_counter,u16 page_offset,u8 *p,u16 length) //512bytes 直接存储
{
	u16 i=0;
    
    // 要擦除所在的sector(要求pagecounter 为所在sector的第一个page) ，然后该sector的第一个page写
	SST25V_SectorErase_4KByte((8*((u32)page_counter/8))*PageSIZE);	
	DF_delay_ms(5); 
	
	for(i=0;i<length;i++)
	{
		SST25V_ByteWrite(*p,page_counter*512+i+page_offset);
		p++;
	}
	DF_delay_ms(5); 
	
}
void DF_EraseAppFile_Area(void)
{
    u16 i=0;
      /*
           1.先擦除0扇区   
           2.读出page48内容并将其写到第0扇区的page 0 
           3.读出page49内容并将其写到第0扇区的page 0
           4.擦除 6-38扇区 即 48page 到 304 page
           5.以后有数据过来就直接写入，不需要再擦除了 
        */
	
        SST25V_SectorErase_4KByte(0x0);
 	 DF_delay_ms(5);   
	 WatchDog_Feed();
       DF_ReadFlash(48,0,SST25Temp,PageSIZE);
	//DF_delay_ms(1);	 
        DF_WriteFlashDirect(0,0,SST25Temp,PageSIZE);
	 DF_delay_ms(1);	
	 WatchDog_Feed();
        DF_ReadFlash(49,0,SST25Temp,PageSIZE);
        DF_WriteFlashDirect(1,0,SST25Temp,PageSIZE);
	 DF_delay_ms(1);		
        for(i=6;i<134;i++)    // 要求是512K -> 128  erase sector 6-134      128K ->  32   
        {
            WatchDog_Feed();
            SST25V_SectorErase_4KByte(((u32)i*4096));
	     DF_delay_ms(5); 
	     WatchDog_Feed();		
        }
        DF_ReadFlash(0,0,SST25Temp,PageSIZE);
        DF_WriteFlashDirect(48,0,SST25Temp,PageSIZE); 
	 WatchDog_Feed();	
	 DF_delay_ms(1);		
        DF_ReadFlash(1,0,SST25Temp,PageSIZE);
	 WatchDog_Feed(); 	
        DF_WriteFlashDirect(49,0,SST25Temp,PageSIZE);
        rt_kprintf("\r\nSST25Ready\r\n"); 

}

void DF_WriteFlashRemote(u16 page_counter,u16 page_offset,u8 *p,u16 length)//512bytes 直接存储
{
	u16 i=0;
    if(51==page_counter) 
    {
       /*
           1.先擦除0扇区   
           2.读出page48内容并将其写到第0扇区的page 0 
           3.读出page49内容并将其写到第0扇区的page 0
           4.擦除 6-38扇区 即 48page 到 304 page
           5.以后有数据过来就直接写入，不需要再擦除了 
        */
	   DF_EraseAppFile_Area(); 
    }
	for(i=0;i<length;i++)
	{
		SST25V_ByteWrite(*p,(u32)page_counter*PageSIZE+(u32)(page_offset+i));
		p++;
	}
	DF_delay_ms(5);
}
void DF_WriteFlashDirect(u16 page_counter,u16 page_offset,u8 *p,u16 length)//512bytes 直接存储
{
    u16 i=0;
  
    
	for(i=0;i<length;i++)
	{
		SST25V_ByteWrite(*p,(u32)page_counter*PageSIZE+(u32)(page_offset+i));
		p++;
	}
	DF_delay_ms(5);  
}

void DF_ClearUpdate_Area(void)   // 清除远程升级区域 
{
    u8  updata_flag=0;
   
	updata_flag=0;	 // 不更新下载的程序
	DF_WriteFlash(DF_APP1_PageNo,0,&updata_flag,1); 
	DF_delay_ms(2);
    DF_EraseAppFile_Area();   // 清除  

}
	


