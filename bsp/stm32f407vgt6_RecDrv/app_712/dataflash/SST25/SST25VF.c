//#include <includes.h>

#include  "App_moduleConfig.h"


void SST25V_DBSY(void); 


void SST25V_DBSY(void)
{ 
  SST25V_CS_LOW();
  SPI_Flash_SendByte(DBSY);
  SST25V_CS_HIGH();
}

 void SST25V_Init(void)
 {

        GPIO_InitTypeDef GPIO_InitStructure;
        SPI_InitTypeDef SPI_InitStructure;

        /* Enable DF_SPI Periph clock */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
	 GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2  );
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2  );
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2  );
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	

        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;


         /*!< SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*!< SPI MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*!< SPI MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);			



    //-------  CS _pin  	
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14;   
	GPIO_InitStructure.GPIO_Mode  =  GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);


      //-------  WP  _pin  	
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;   
    GPIO_InitStructure.GPIO_Mode  =  GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);	

        /*------------------------ DF_SPI configuration ------------------------*/
        SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//SPI_Direction_1Line_Tx;
        SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
        SPI_InitStructure.SPI_NSS  = SPI_NSS_Soft;
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;/* 72M/64=1.125M */
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
        SPI_InitStructure.SPI_CRCPolynomial = 7; 

	  //SPI1->CR2=0x04; 									//NSS ---SSOE
    //   SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
       //SPI_InitStructure.SPI_Mode = SPI_Mode_Master; 	  //MSTR	

        SPI_I2S_DeInit(DF_SPI);
        SPI_Init(DF_SPI, &SPI_InitStructure);

        /* Enable SPI_MASTER */
        SPI_Cmd(DF_SPI, ENABLE);
        SPI_CalculateCRC(DF_SPI, DISABLE); 

         SST25V_CS_HIGH();
	  SST25V_WP_HIGH();
	  //SST25V_HOLD_HIGH();
	//  SST25V_EnableWriteStatusRegister();
	 // SST25V_WriteStatusRegister(0x02); 
	   SST25V_DBSY(); 
}

u8 SPI_Flash_SendByte(u8 data)
{
    //Wait until the transmit buffer is empty
    while (SPI_I2S_GetFlagStatus(DF_SPI, SPI_I2S_FLAG_TXE) == RESET);
    // Send the byte
    SPI_I2S_SendData(DF_SPI, data);

    //Wait until a data is received
    while (SPI_I2S_GetFlagStatus(DF_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    // Get the received data
    data = SPI_I2S_ReceiveData(DF_SPI);

    // Return the shifted data
    return data;
}

u8 SPI_Flash_ReceiveByte(void)
{
  return (SPI_Flash_SendByte(Dummy_Byte));
}


u8 SST25V_ByteRead(u32 ReadAddr)
{
  u8 Temp = 0;
  SST25V_CS_LOW();
  SPI_Flash_SendByte(Read_Data);
  SPI_Flash_SendByte((ReadAddr & 0xFF0000) >> 16);
  SPI_Flash_SendByte((ReadAddr& 0xFF00) >> 8);
  SPI_Flash_SendByte(ReadAddr & 0xFF);
  
  Temp = SPI_Flash_ReceiveByte();
  SST25V_CS_HIGH();
  return Temp;
}

void SST25V_BufferRead(u8* pBuffer, u32 ReadAddr, u16 NumByteToRead)
{
  SST25V_CS_LOW();
  SPI_Flash_SendByte(Read_Data);
  SPI_Flash_SendByte((ReadAddr & 0xFF0000) >> 16);
  SPI_Flash_SendByte((ReadAddr& 0xFF00) >> 8);
  SPI_Flash_SendByte(ReadAddr & 0xFF);

  while(NumByteToRead--)
  {
    *pBuffer = SPI_Flash_ReceiveByte();
    pBuffer++;
  }
  SST25V_CS_HIGH();
}

void SST25V_HighSpeedBufferRead(u8* pBuffer, u32 ReadAddr, u16 NumByteToRead)
{
  SST25V_CS_LOW();
  SPI_Flash_SendByte(HighSpeedReadData);
  SPI_Flash_SendByte((ReadAddr & 0xFF0000) >> 16);
  SPI_Flash_SendByte((ReadAddr& 0xFF00) >> 8);
  SPI_Flash_SendByte(ReadAddr & 0xFF);
  SPI_Flash_SendByte(Dummy_Byte);

  while(NumByteToRead--)
  {
    *pBuffer = SPI_Flash_ReceiveByte();
    pBuffer++;
  }
  SST25V_CS_HIGH();
}

u8 SST25V_HighSpeedRead(u32 ReadAddr)
{
  u32 Temp = 0;
  SST25V_CS_LOW();
  SPI_Flash_SendByte(HighSpeedReadData);
  SPI_Flash_SendByte((ReadAddr & 0xFF0000) >> 16);
  SPI_Flash_SendByte((ReadAddr& 0xFF00) >> 8);
  SPI_Flash_SendByte(ReadAddr & 0xFF);
  SPI_Flash_SendByte(Dummy_Byte);
  Temp = SPI_Flash_ReceiveByte();
  SST25V_CS_HIGH();
  return Temp;
}


// u8 SPI_Flash_SendByte(u8 byte)
// {
//   while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
//   SPI_I2S_SendData(SPI3, byte);

//   while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET);
//   return SPI_I2S_ReceiveData(SPI3);
// }

// u8 SPI_Flash_ReceiveByte(void)
// {
//   return (SPI_Flash_SendByte(Dummy_Byte));
// }

/************************************************************************/
/* 名称: Byte_Program						*/
/* 功能: 写一个字节数据,被写的地址必须被擦除及未被保护		*/				
/* 输入:								*/
/*		Dst:		(目标地址 000000H - 1FFFFFH)	*/
/*		byte:		数据			*/
/* 返回:								*/
/*		Nothing							*/
/************************************************************************/
void SST25V_ByteWrite(u8 Byte, u32 WriteAddr)
{
  SST25V_WriteEnable();
  SST25V_CS_LOW();
  SPI_Flash_SendByte(Byte_Program);
  SPI_Flash_SendByte((WriteAddr & 0xFF0000) >> 16);
  SPI_Flash_SendByte((WriteAddr & 0xFF00) >> 8);
  SPI_Flash_SendByte(WriteAddr & 0xFF);
  
  SPI_Flash_SendByte(Byte);
  SST25V_CS_HIGH();  
  SST25V_WaitForWriteEnd(); 
}

void SST25V_strWrite(u8 *p, u32 WriteAddr,u16 length)
{
  SST25V_WriteEnable();
  SST25V_CS_LOW();
  SPI_Flash_SendByte(Byte_Program);
  SPI_Flash_SendByte((WriteAddr & 0xFF0000) >> 16);
  SPI_Flash_SendByte((WriteAddr & 0xFF00) >> 8);
  SPI_Flash_SendByte(WriteAddr & 0xFF);
  while(length--)
    {  
       SPI_Flash_SendByte(*p);
        p++;
    }
  
  
  SST25V_CS_HIGH();  
  SST25V_WaitForWriteEnd(); 
}



void AutoAddressIncrement_WordProgramA(u8 Byte1, u8 Byte2, u32 Addr)
{
  SST25V_WriteEnable();
  SST25V_CS_LOW();
  SPI_Flash_SendByte(AAI_WordProgram);
  SPI_Flash_SendByte((Addr & 0xFF0000) >> 16);
  SPI_Flash_SendByte((Addr & 0xFF00) >> 8);
  SPI_Flash_SendByte(Addr & 0xFF);   

  SPI_Flash_SendByte(Byte1);
  SPI_Flash_SendByte(Byte2);  

  SST25V_CS_HIGH();
  SST25V_Wait_Busy_AAI();
  //SPI_FLASH_WaitForWriteEnd();
}


void AutoAddressIncrement_WordProgramB(u8 state,u8 Byte1, u8 Byte2) 
{ 
  SST25V_WriteEnable();
  SST25V_CS_LOW();
  SPI_Flash_SendByte(AAI_WordProgram);

  SPI_Flash_SendByte(Byte1);
  SPI_Flash_SendByte(Byte2);

  SST25V_CS_HIGH();
  SST25V_Wait_Busy_AAI();
  
  if(state==1)
  {
    SST25V_WriteDisable();
  }
  SST25V_Wait_Busy_AAI();
}


void SST25V_Wait_Busy_AAI(void) 
{ 
  while (SST25V_ReadStatusRegister() == 0x43) /* 等待空闲 */
  SST25V_ReadStatusRegister(); 
}


void SST25V_SectorErase_4KByte(u32 Addr)
{
  SST25V_WriteEnable();
  SST25V_CS_LOW();
  SPI_Flash_SendByte(SectorErace_4KB);
  SPI_Flash_SendByte((Addr & 0xFF0000) >> 16); 
  SPI_Flash_SendByte((Addr & 0xFF00) >> 8);
  SPI_Flash_SendByte(Addr & 0xFF);
  
  SST25V_CS_HIGH();
  SST25V_WaitForWriteEnd();
}

void SST25V_BlockErase_32KByte(u32 Addr)
{
  SST25V_WriteEnable();
  SST25V_CS_LOW();
  SPI_Flash_SendByte(BlockErace_32KB);
  SPI_Flash_SendByte((Addr & 0xFF0000) >> 16);
  SPI_Flash_SendByte((Addr & 0xFF00) >> 8);
  SPI_Flash_SendByte(Addr & 0xFF);
  
  SST25V_CS_HIGH();
  SST25V_WaitForWriteEnd();
}

void SST25V_BlockErase_64KByte(u32 Addr)
{
  SST25V_WriteEnable();
  SST25V_CS_LOW();
  SPI_Flash_SendByte(BlockErace_64KB);
  SPI_Flash_SendByte((Addr & 0xFF0000) >> 16);
  SPI_Flash_SendByte((Addr & 0xFF00) >> 8);
  SPI_Flash_SendByte(Addr & 0xFF);
  
  SST25V_CS_HIGH();
  SST25V_WaitForWriteEnd();
}

void SST25V_ChipErase(void)
{
  SST25V_WriteEnable();
  SST25V_CS_LOW();
  SPI_Flash_SendByte(ChipErace);
  SST25V_CS_HIGH();
  SST25V_WaitForWriteEnd();
}

u8 SST25V_ReadStatusRegister(void)
{
  u8 StatusRegister = 0;
  SST25V_CS_LOW();
  SPI_Flash_SendByte(ReadStatusRegister);
  StatusRegister = SPI_Flash_ReceiveByte();
  SST25V_CS_HIGH();
  return StatusRegister;
}

void SST25V_WriteEnable(void)
{
  SST25V_CS_LOW();
  SPI_Flash_SendByte(WriteEnable);
  SST25V_CS_HIGH();
}

void SST25V_WriteDisable(void)
{
  SST25V_CS_LOW();
  SPI_Flash_SendByte(WriteDisable);
  SST25V_CS_HIGH();
}

void SST25V_EnableWriteStatusRegister(void)
{
  SST25V_CS_LOW();
  SPI_Flash_SendByte(EnableWriteStatusRegister);
  SST25V_CS_HIGH();
}

void SST25V_WriteStatusRegister(u8 Byte)
{
  SST25V_EnableWriteStatusRegister();
  SST25V_CS_LOW();
  SPI_Flash_SendByte(WriteStatusRegister);
  SPI_Flash_SendByte(Byte);
  SST25V_CS_HIGH();
}

void SST25V_WaitForWriteEnd(void)
{
  u8 FLASH_Status = 0;
  SST25V_CS_LOW();
  SPI_Flash_SendByte(ReadStatusRegister);
  do
  {
    FLASH_Status = SPI_Flash_SendByte(Dummy_Byte);

  } while((FLASH_Status & WriteStatusRegister) == SET);

  SST25V_CS_HIGH();
}

u32 SST25V_ReadJedecID(void)
{
  u32 JEDECID = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;
  SST25V_CS_LOW();
  SPI_Flash_SendByte(ReadJedec_ID);
  Temp0 = SPI_Flash_ReceiveByte();
  Temp1 = SPI_Flash_ReceiveByte();
  Temp2 = SPI_Flash_ReceiveByte();
  SST25V_CS_HIGH();  
  JEDECID = (Temp0 << 16) | (Temp1 << 8) | Temp2;
  return JEDECID;
}

u16 SST25V_ReadManuID_DeviceID(u32 ReadManu_DeviceID_Addr)
{
  u16 ManuID_DeviceID = 0;
  u8 ManufacturerID = 0,  DeviceID = 0;
  SST25V_CS_LOW();
  SPI_Flash_SendByte(ReadDeviceID);
  
  SPI_Flash_SendByte((ReadManu_DeviceID_Addr & 0xFF0000) >> 16);
  SPI_Flash_SendByte((ReadManu_DeviceID_Addr & 0xFF00) >> 8);
  SPI_Flash_SendByte(ReadManu_DeviceID_Addr & 0xFF);
  
  if(ReadManu_DeviceID_Addr==1)
  {
    DeviceID = SPI_Flash_ReceiveByte();
    ManufacturerID = SPI_Flash_ReceiveByte();
  }
  else
  {
    ManufacturerID = SPI_Flash_ReceiveByte();
    DeviceID = SPI_Flash_ReceiveByte();
  }
  
  ManuID_DeviceID = ((ManufacturerID<<8) | DeviceID);
  SST25V_CS_HIGH();
  
  return ManuID_DeviceID;
}

/*void SST25V_EBSY()
{ 
  SST25V_CS_LOW();
  SPI_Flash_SendByte(EBSY);
  SST25V_CS_HIGH();
} */


/////////////////////////////////////////////////////////////////////////////////////////////////////

