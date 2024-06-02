#include "ST7735.h"

void LCD_Init() {
      pinMode(_ss_pin, OUTPUT);
      digitalWrite(_ss_pin, LOW);
      pinMode(DC, OUTPUT);
      pinMode(BLK, OUTPUT);
      // pinMode(MOSI, OUTPUT);
      // pinMode(SCLK, OUTPUT);
      digitalWrite(BLK, HIGH);
      SPI.begin();
      //SPI.setClockDivider(SPI_CLOCK_DIV4);
    }

  void  SPI_WriteData(uint8_t Data)
  {
	// unsigned char i=0;
	// for(i=8;i>0;i--)
	// {
	// 	if(Data&0x80)	
	//   digitalWrite(MOSI, HIGH); 
  //     else digitalWrite(MOSI, LOW);
	   
  //     digitalWrite(SCLK, LOW);       
  //     digitalWrite(SCLK, HIGH); ;
  //     Data<<=1; 
	//   }
    SPI.transfer(Data);
  }
    void Lcd_WriteIndex(uint8_t index)
    {
      
      digitalWrite(DC, LOW);
      SPI_WriteData(index);
    }

    void Lcd_WriteData(uint8_t data)
    {
      
      digitalWrite(DC, HIGH);
      SPI_WriteData(data);
    }

    void Lcd_WriteReg(uint8_t index, uint8_t data)
    {
        Lcd_WriteIndex(index);
        Lcd_WriteData(data);
    }

    void LCD_WriteData_16Bit(uint16_t Data)
    {
      
      digitalWrite(DC, HIGH);
      SPI_WriteData(Data>>8); 	
      SPI_WriteData(Data); 			
       
    }

    void Lcd_Init(void)
    {	
    
#ifdef  ST7735
    //LCD Init For 1.44Inch LCD Panel with ST7735R.
    Lcd_WriteIndex(0x11);//Sleep exit 
    delay(120);
      
    //ST7735R Frame Rate
    Lcd_WriteIndex(0xB1); 
    Lcd_WriteData(0x01); 
    Lcd_WriteData(0x2C); 
    Lcd_WriteData(0x2D); 

    Lcd_WriteIndex(0xB2); 
    Lcd_WriteData(0x01); 
    Lcd_WriteData(0x2C); 
    Lcd_WriteData(0x2D); 

    Lcd_WriteIndex(0xB3); 
    Lcd_WriteData(0x01); 
    Lcd_WriteData(0x2C); 
    Lcd_WriteData(0x2D); 
    Lcd_WriteData(0x01); 
    Lcd_WriteData(0x2C); 
    Lcd_WriteData(0x2D); 
    
    Lcd_WriteIndex(0xB4); //Column inversion 
    Lcd_WriteData(0x07); 
    
    //ST7735R Power Sequence
    Lcd_WriteIndex(0xC0); 
    Lcd_WriteData(0xA2); 
    Lcd_WriteData(0x02); 
    Lcd_WriteData(0x84); 
    Lcd_WriteIndex(0xC1); 
    Lcd_WriteData(0xC5); 

    Lcd_WriteIndex(0xC2); 
    Lcd_WriteData(0x0A); 
    Lcd_WriteData(0x00); 

    Lcd_WriteIndex(0xC3); 
    Lcd_WriteData(0x8A); 
    Lcd_WriteData(0x2A); 
    Lcd_WriteIndex(0xC4); 
    Lcd_WriteData(0x8A); 
    Lcd_WriteData(0xEE); 
    
    Lcd_WriteIndex(0xC5); //VCOM 
    Lcd_WriteData(0x0E); 
    
    Lcd_WriteIndex(0x36); //MX, MY, RGB mode 
    Lcd_WriteData(0xC0); 
    
    //ST7735R Gamma Sequence
    Lcd_WriteIndex(0xe0); 
    Lcd_WriteData(0x0f); 
    Lcd_WriteData(0x1a); 
    Lcd_WriteData(0x0f); 
    Lcd_WriteData(0x18); 
    Lcd_WriteData(0x2f); 
    Lcd_WriteData(0x28); 
    Lcd_WriteData(0x20); 
    Lcd_WriteData(0x22); 
    Lcd_WriteData(0x1f); 
    Lcd_WriteData(0x1b); 
    Lcd_WriteData(0x23); 
    Lcd_WriteData(0x37); 
    Lcd_WriteData(0x00); 	
    Lcd_WriteData(0x07); 
    Lcd_WriteData(0x02); 
    Lcd_WriteData(0x10); 

    Lcd_WriteIndex(0xe1); 
    Lcd_WriteData(0x0f); 
    Lcd_WriteData(0x1b); 
    Lcd_WriteData(0x0f); 
    Lcd_WriteData(0x17); 
    Lcd_WriteData(0x33); 
    Lcd_WriteData(0x2c); 
    Lcd_WriteData(0x29); 
    Lcd_WriteData(0x2e); 
    Lcd_WriteData(0x30); 
    Lcd_WriteData(0x30); 
    Lcd_WriteData(0x39); 
    Lcd_WriteData(0x3f); 
    Lcd_WriteData(0x00); 
    Lcd_WriteData(0x07); 
    Lcd_WriteData(0x03); 
    Lcd_WriteData(0x10);  
    
    Lcd_WriteIndex(0x2a);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x7f);

    Lcd_WriteIndex(0x2b);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x00);
    Lcd_WriteData(0x9f);
    
    Lcd_WriteIndex(0xF0); //Enable test command  
    Lcd_WriteData(0x01); 
    Lcd_WriteIndex(0xF6); //Disable ram power save mode 
    Lcd_WriteData(0x00); 
    
    Lcd_WriteIndex(0x3A); //65k mode 
    Lcd_WriteData(0x05); 
    
    Lcd_WriteIndex(0x29);//Display on

#endif
#ifdef ST7789

	Lcd_WriteIndex(0x36);
	if(USE_HORIZONTAL==0)Lcd_WriteData(0x00);
	else if(USE_HORIZONTAL==1)Lcd_WriteData(0xC0);
	else if(USE_HORIZONTAL==2)Lcd_WriteData(0x70);
	else Lcd_WriteData(0xA0); 

	Lcd_WriteIndex( 0x3A);     
	Lcd_WriteData( 0x05);   

	Lcd_WriteIndex( 0xB2);     
	Lcd_WriteData( 0x0B);   
	Lcd_WriteData( 0x0B);   
	Lcd_WriteData( 0x00);   
	Lcd_WriteData( 0x33);   
	Lcd_WriteData( 0x33);   

	Lcd_WriteIndex( 0xB7);     
	Lcd_WriteData( 0x11);   

	Lcd_WriteIndex( 0xBB);     
	Lcd_WriteData( 0x2F);   

	Lcd_WriteIndex( 0xC0);     
	Lcd_WriteData( 0x2C);   

	Lcd_WriteIndex( 0xC2);     
	Lcd_WriteData( 0x01);   

	Lcd_WriteIndex( 0xC3);     
	Lcd_WriteData( 0x0D);   

	Lcd_WriteIndex( 0xC4);     
	Lcd_WriteData( 0x20);   //VDV, 0x20:0v

	Lcd_WriteIndex( 0xC6);     
	Lcd_WriteData( 0x18);   //0x13:60Hz   

	Lcd_WriteIndex( 0xD0);     
	Lcd_WriteData( 0xA7);   
	Lcd_WriteData( 0xA1); 

	Lcd_WriteIndex( 0xD0);     
	Lcd_WriteData( 0xA4);   
	Lcd_WriteData( 0xA1);   

	Lcd_WriteIndex( 0xD6);     
	Lcd_WriteData( 0xA1);   //sleep in后，gate输出为GND

	Lcd_WriteIndex( 0xE0);     
	Lcd_WriteData( 0xF0);   
	Lcd_WriteData( 0x06);   
	Lcd_WriteData( 0x0B);   
	Lcd_WriteData( 0x0A);   
	Lcd_WriteData( 0x09);   
	Lcd_WriteData( 0x26);   
	Lcd_WriteData( 0x29);   
	Lcd_WriteData( 0x33);   
	Lcd_WriteData( 0x41);   
	Lcd_WriteData( 0x18);   
	Lcd_WriteData( 0x16);   
	Lcd_WriteData( 0x15);   
	Lcd_WriteData( 0x29);   
	Lcd_WriteData( 0x2D);   

	Lcd_WriteIndex( 0xE1);     
	Lcd_WriteData( 0xF0);   
	Lcd_WriteData( 0x04);   
	Lcd_WriteData( 0x08);   
	Lcd_WriteData( 0x08);   
	Lcd_WriteData( 0x07);   
	Lcd_WriteData( 0x03);   
	Lcd_WriteData( 0x28);   
	Lcd_WriteData( 0x32);   
	Lcd_WriteData( 0x40);   
	Lcd_WriteData( 0x3B);   
	Lcd_WriteData( 0x19);   
	Lcd_WriteData( 0x18);   
	Lcd_WriteData( 0x2A);   
	Lcd_WriteData( 0x2E);   

	Lcd_WriteIndex( 0xE4);     
	Lcd_WriteData( 0x25);   
	Lcd_WriteData( 0x00);   
	Lcd_WriteData( 0x00);   //当gate没有用完时，bit4(TMG)设为0

	Lcd_WriteIndex( 0x35);     
	Lcd_WriteData( 0x00); 

	Lcd_WriteIndex( 0x44);     
	Lcd_WriteData( 0x00);   
	Lcd_WriteData( 0x20); 
	Lcd_WriteIndex( 0x21);   
	Lcd_WriteIndex( 0x11);     
	delay(120);      
	Lcd_WriteIndex( 0x29);  

#endif


  }

void Lcd_SetRegion(uint16_t x_start,uint16_t y_start,uint16_t x_end,uint16_t y_end)
{	

#ifdef ST7735	

	Lcd_WriteIndex(0x2a);
	Lcd_WriteData(0x00);
	Lcd_WriteData(x_start);//Lcd_WriteData(x_start+2);
	Lcd_WriteData(0x00);
	Lcd_WriteData(x_end+2);

	Lcd_WriteIndex(0x2b);
	Lcd_WriteData(0x00);
	Lcd_WriteData(y_start+0);
	Lcd_WriteData(0x00);
	Lcd_WriteData(y_end+1);
	
	Lcd_WriteIndex(0x2c);
#endif

#ifdef ST7789
  if(USE_HORIZONTAL==0)
	{
		Lcd_WriteIndex(0x2a);//列地址设置
		LCD_WriteData_16Bit(x_start);
		LCD_WriteData_16Bit(x_end);
		Lcd_WriteIndex(0x2b);//行地址设置
		LCD_WriteData_16Bit(y_start+20);
		LCD_WriteData_16Bit(y_end+20);
		Lcd_WriteIndex(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==1)
	{
		Lcd_WriteIndex(0x2a);//列地址设置
		LCD_WriteData_16Bit(x_start);
		LCD_WriteData_16Bit(x_end);
		Lcd_WriteIndex(0x2b);//行地址设置
		LCD_WriteData_16Bit(x_start+20);
		LCD_WriteData_16Bit(y_end+20);
		Lcd_WriteIndex(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==2)
	{
		Lcd_WriteIndex(0x2a);//列地址设置
		LCD_WriteData_16Bit(x_start+20);
		LCD_WriteData_16Bit(x_end+20);
		Lcd_WriteIndex(0x2b);//行地址设置
		LCD_WriteData_16Bit(y_start);
		LCD_WriteData_16Bit(y_end);
		Lcd_WriteIndex(0x2c);//储存器写
	}
	else
	{
		Lcd_WriteIndex(0x2a);//列地址设置
		LCD_WriteData_16Bit(x_start+20);
		LCD_WriteData_16Bit(x_end+20);
		Lcd_WriteIndex(0x2b);//行地址设置
		LCD_WriteData_16Bit(y_start);
		LCD_WriteData_16Bit(y_end);
		Lcd_WriteIndex(0x2c);//储存器写
	}
#endif
}


void Lcd_SetXY(uint16_t x,uint16_t y)
{
  	Lcd_SetRegion(x,y,x,y);
}

	

void Gui_DrawPoint(uint16_t x,uint16_t y,uint16_t Data)
{
	Lcd_SetRegion(x,y,x+1,y+1);
	LCD_WriteData_16Bit(Data);

}    


unsigned int Lcd_ReadPoint(uint16_t x,uint16_t y)
{
  unsigned int Data;
  Lcd_SetXY(x,y);


  Lcd_WriteData(Data);
  return Data;
}

void Lcd_Clear(uint16_t Color)               
{	
   unsigned int i,m;
   Lcd_SetRegion(0,0,X_MAX_PIXEL-1,Y_MAX_PIXEL-1);
   Lcd_WriteIndex(0x2C);
   for(i=0;i<X_MAX_PIXEL;i++)
    for(m=0;m<Y_MAX_PIXEL;m++)
    {	
	  	  LCD_WriteData_16Bit(Color);
    }   
}
