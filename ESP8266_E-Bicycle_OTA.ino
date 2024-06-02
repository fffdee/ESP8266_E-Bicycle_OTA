#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <ESP8266mDNS.h>
#include <SPI.h>
#include <Wire.h>
#include "st7735.h"
#include "LIS3DHTR.h"
#include "vacal.h"
#include "wifiSetting.h"
#include "font.h"
#include "Picture.h"

IPAddress myIP;
ESP8266WebServer server(80);
/*******************************************************************TFT_Funstion*******************************************************************************/
class ESPTFT {
  private:
    uint8_t _ss_pin;

  public:
    ESPTFT(uint8_t pin): _ss_pin(pin) {}
    void begin() {
      pinMode(_ss_pin, OUTPUT);
      digitalWrite(_ss_pin, LOW);
      pinMode(DC, OUTPUT);
      pinMode(BLK, OUTPUT);
      // pinMode(MOSI, OUTPUT);
      // pinMode(SCLK, OUTPUT);
      digitalWrite(BLK, HIGH);
      SPI.setClockDivider(SPI_CLOCK_DIV4);
      SPI.begin();
      
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
    //Lcd_WriteData(0xC0); 
    Lcd_WriteData(0x60); 
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


  }



};


ESPTFT esp_tft(SS);

void Lcd_SetRegion(uint16_t x_start,uint16_t y_start,uint16_t x_end,uint16_t y_end);
void Lcd_SetXY(uint16_t x,uint16_t y);
void Gui_DrawPoint(uint16_t x,uint16_t y,uint16_t Data);
unsigned int Lcd_ReadPoint(uint16_t x,uint16_t y);
void Lcd_Clear(uint16_t Color);
uint16_t LCD_BGR2RGB(uint16_t c);
void Gui_Circle(uint16_t X,uint16_t Y,uint16_t R,uint16_t fc); 
void Gui_DrawLine(uint16_t x0, uint16_t y0,uint16_t x1, uint16_t y1,uint16_t Color);  
void Gui_box(uint16_t x, uint16_t y, uint16_t w, uint16_t h,uint16_t bc);
void Gui_box2(uint16_t x,uint16_t y,uint16_t w,uint16_t h, u8 mode);
void DisplayButtonDown(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);
void DisplayButtonUp(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);
void Gui_DrawFont_GBK16(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, u8 *s);
void Gui_DrawFont_GBK24(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, u8 *s);
void Gui_DrawFont_Num32(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint16_t num);
void showimage(const unsigned char *p);

/********************************************************Vacal_Funstion*********************************************************/
void Vacal_Play(void);
void Vacal_Stop(void);
void Vacal_Pause(void);
void Vacal_Next(void);
void Vacal_Last(void);
void Vacal_Set_Volume(uint8_t val);
void Vacal_Select_Play(uint8_t val);
/******************************************************OTA_UpGrade_Funstion******************************************************/
void OTA_Init(void);
void OTA_Loop(void);
void handleRoot(void);
/******************************************************LIS3DHTR_Funstion******************************************************/
void LIS3DHTR_Init(void);
void readAcceleration(int16_t* x, int16_t* y, int16_t* z);
/******************************************************PCF8574_Funstion******************************************************/
void PCF_Init(uint8_t addr,uint8_t Init_val);
uint8_t PCF_GPIO_Read(uint8_t addr,uint8_t val);
uint8_t PCF_GPIO_Read_All(uint8_t addr);
void PCF_GPIO_Write_All(uint8_t addr,uint8_t val);
void PCF_GPIO_Write(uint8_t addr,uint8_t IO_Num,uint8_t val);
/******************************************************UI_Struct_Funstion*******************************************************/
void main_ui(void);

unsigned char func_index=0;	 

void (*current_operation_index)(); 

typedef struct
{

        unsigned char current;

        unsigned char up;

        unsigned char down;

        unsigned char enter;

        void (*current_operation)();	 

} key_table;


key_table table[30]=

{
    {0,0,0,1,main_ui},

    // {1,4,2,5,fun2_1},	

    // {2,1,3,6,fun2_2},                

    // {3,2,4,7,fun2_3},
		
	// 	{4,3,1,0,fun2_4},
		
	// 	{5,5,5,1,fun3_1},
		
	// 	{6,6,6,2,fun3_2}, 
		     	
	// 	{7,10,8,7,fun3_3_1},

	// 	{8,7,9,8,fun3_3_2},
		
	// 	{9,8,10,9,fun3_3_3},
		
	// 	{10,9,7,1,fun3_3_4},                                                                             

};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  OTA_Init();
  LIS3DHTR_Init();
  esp_tft.begin();
  esp_tft.Lcd_Init();
  Lcd_Clear(RED);
  Vacal_Set_Volume(5);
  Vacal_Select_Play(1);
 // delay(100);
  Lcd_Clear(BLUE);
  Lcd_Clear(BLACK);
  // Output data to serial monitor
  Gui_DrawLine(20, 0,20, 101,YELLOW);
  Gui_DrawLine(20, 101,160, 101,YELLOW);  

}

void loop() {
  // put your main code here, to run repeatedly:
  OTA_Loop();
  current_operation_index=table[func_index].current_operation;
  (*current_operation_index)();
}

// void UI_handle(void)
// {
//   		if((HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)==0)||(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==0)||(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==0))

//                             {

//                                      delay(2);

//                                      if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==0)

//                                      {

//                                      func_index=table[func_index].up; 

//                                      while(!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5));

//                                      }

//                                      if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)==0)

//                                      {

//                                      func_index=table[func_index].down;

//                                      while(!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)); 

//                                      }

//                                      if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==0)

//                                      {

// 																				 func_index=table[func_index].enter; 

// 																				 while(!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6));

//                                      }


                              

//                            }                                    

                            
// }

/********************************************************Vacal_Funstion*********************************************************/
void Vacal_Play(void){
    Serial.write(play,4);
}


void Vacal_Stop(void){
    Serial.write(stop,4);
}


void Vacal_Pause(void){
    Serial.write(Pause,4);
}


void Vacal_Next(void){
    Serial.write(next,4);
}


void Vacal_Last(void){
    Serial.write(last,4);
}


void Vacal_Set_Volume(uint8_t val){
    select_play[4] = play_set[val]; 
    select_play[5] = play_SM[val]; 
    Serial.write(select_play,6);
}

void Vacal_Select_Play(uint8_t val){
    select_play[4] = play_set[val]; 
    select_play[5] = play_SM[val];  
    Serial.write(select_play,6);
}

/******************************************************OTA_UpGrade_Funstion******************************************************/
void OTA_Init(){
  
  WiFi.softAP(ssid, password);
  myIP = WiFi.softAPIP();
  MDNS.begin(host);
  //Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.on("/test", handleRoot);
  server.on("/", HTTP_GET, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", serverIndex);
    });
    server.on("/update", HTTP_POST, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
      ESP.restart();
    }, []() {
      HTTPUpload& upload = server.upload();
      if (upload.status == UPLOAD_FILE_START) {
        Serial.setDebugOutput(true);
        WiFiUDP::stopAll();
        Serial.printf("Update: %s\n", upload.filename.c_str());
        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        if (!Update.begin(maxSketchSpace)) { //start with max available size
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }
        
        
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) { //true to set the size to the current progress
          //Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        } else {
          Update.printError(Serial);
        }
        Serial.setDebugOutput(false);
      }
      yield();
    });
  server.begin();
  MDNS.addService("http", "tcp", 80);

}

void OTA_Loop(){
  server.handleClient();
  MDNS.update();
}

void handleRoot(void) {
 
                       // wait for a second
  digitalWrite(2, HIGH);   // turn the LED off by making the voltage LOW
  delay(200);  
   server.send(200, "text/html", "ok");
}

/******************************************************LIS3DHTR_Funstion******************************************************/
void LIS3DHTR_Init(void){
  // Configure LIS3DHTR
  // Set power mode to normal and data rate to 100 Hz
  Wire.begin(); 
  Wire.beginTransmission(LIS3DHTR_ADDRESS);
  Wire.write(LIS3DHTR_CTRL_REG1);
  Wire.write(0b00100111);  // Normal mode, 100 Hz ODR
  Wire.endTransmission();

  // Set full scale to ±2g
  Wire.beginTransmission(LIS3DHTR_ADDRESS);
  Wire.write(LIS3DHTR_CTRL_REG4);
  Wire.write(0b00000000);  // ±2g full scale
  Wire.endTransmission();
}

void readAcceleration(int16_t* x, int16_t* y, int16_t* z) {
  Wire.beginTransmission(LIS3DHTR_ADDRESS);
  Wire.write(LIS3DHTR_OUT_X_L | 0x80);  // Auto-increment address
  Wire.endTransmission();
  Wire.requestFrom(LIS3DHTR_ADDRESS, 6);

  // Read the six bytes of data (two bytes per axis)
  uint8_t bytes[6];
  for (uint8_t i = 0; i < 6; i++) {
    bytes[i] = Wire.read();
  }

  // Convert the data
  *x = (int16_t)(bytes[0] | (bytes[1] << 8));
  *y = (int16_t)(bytes[2] | (bytes[3] << 8));
  *z = (int16_t)(bytes[4] | (bytes[5] << 8));
}
/******************************************************PCF8574_Funstion******************************************************/
void PCF_Init(uint8_t addr,uint8_t Init_val){
  Wire.beginTransmission(addr); // 开始与地址为 0x40 的从设备通信
  Wire.write(Init_val); 
  Wire.endTransmission(); 
}

uint8_t PCF_GPIO_Read_All(uint8_t addr){
  uint8_t data;
  Wire.beginTransmission(addr); // 开始与地址为 0x40 的从设备通信
  data = Wire.read(); 
  Wire.endTransmission(); 
  return data;
}

uint8_t PCF_GPIO_Read(uint8_t addr,uint8_t val){
  
  uint8_t data; 
  uint8_t mask[8] ={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
  uint8_t IO_Status;
  if(val<8&&val>-1){
    Wire.beginTransmission(addr); // 开始与地址为 0x40 的从设备通信
    data = Wire.read(); 
    Wire.endTransmission();
  }else{
    return -1;
  }
 
  if(val>0)IO_Status = (data&mask[val])>>(val-1);
  else IO_Status = (data&mask[val]);

  return IO_Status;

}

void PCF_GPIO_Write_All(uint8_t addr,uint8_t val){
  Wire.beginTransmission(addr); // 开始与地址为 0x40 的从设备通信
  Wire.write(val); 
  Wire.endTransmission(); 
}

void PCF_GPIO_Write(uint8_t addr,uint8_t IO_Num,uint8_t val){
  uint8_t data; 
  uint8_t mask[8] ={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
  if((IO_Num<8)&&(IO_Num>-1)&&(val>-1)&&(val<2)){

    Wire.beginTransmission(addr); // 开始与地址为 0x40 的从设备通信
    data = Wire.read(); 
    Wire.endTransmission();
    data = data|(val<<IO_Num);
    Wire.beginTransmission(addr); // 开始与地址为 0x40 的从设备通信
    Wire.write(data); 
    Wire.endTransmission(); 

  }
}

/******************************************************UI_Struct_Funstion*******************************************************/

void main_ui(void){
  int16_t x, y, z;   
  // esp_tft.Lcd_Clear(GREEN);
  // delay(500);
  // Read acceleration data
  readAcceleration(&x, &y, &z);
 
  Serial.print(x);
  Serial.print(","); 
  Serial.print(y);
  Serial.print(",");
  Serial.println(z);
  // esp_tft.Lcd_Clear(BLUE);
  // delay(500);
}

/************************************************************TFT_Funstion******************************************************************/
void Lcd_SetRegion(uint16_t x_start,uint16_t y_start,uint16_t x_end,uint16_t y_end){
	esp_tft.Lcd_WriteIndex(0x2a);
	esp_tft.Lcd_WriteData(0x00);
	esp_tft.Lcd_WriteData(x_start);//Lcd_WriteData(x_start+2);
	esp_tft.Lcd_WriteData(0x00);
	esp_tft.Lcd_WriteData(x_end+2);

	esp_tft.Lcd_WriteIndex(0x2b);
	esp_tft.Lcd_WriteData(0x00);
	esp_tft.Lcd_WriteData(y_start+0);
	esp_tft.Lcd_WriteData(0x00);
	esp_tft.Lcd_WriteData(y_end+1);
	
	esp_tft.Lcd_WriteIndex(0x2c);

}


void Lcd_SetXY(uint16_t x,uint16_t y){
  	Lcd_SetRegion(x,y,x,y);
}

	

void Gui_DrawPoint(uint16_t x,uint16_t y,uint16_t Data){
	Lcd_SetRegion(x,y,x+1,y+1);
	esp_tft.LCD_WriteData_16Bit(Data);

}    


unsigned int Lcd_ReadPoint(uint16_t x,uint16_t y){
  unsigned int Data;
  Lcd_SetXY(x,y);


  esp_tft.Lcd_WriteData(Data);
  return Data;
}

void Lcd_Clear(uint16_t Color)               {	
   unsigned int i,m;
   Lcd_SetRegion(0,0,X_MAX_PIXEL-1,Y_MAX_PIXEL-1);
   esp_tft.Lcd_WriteIndex(0x2C);
   for(i=0;i<X_MAX_PIXEL;i++)
    for(m=0;m<Y_MAX_PIXEL;m++)
    {	
	  	  esp_tft.LCD_WriteData_16Bit(Color);
    }   
}

uint16_t LCD_BGR2RGB(uint16_t c){
  uint16_t  r,g,b,rgb;   
  b=(c>>0)&0x1f;
  g=(c>>5)&0x3f;
  r=(c>>11)&0x1f;	 
  rgb=(b<<11)+(g<<5)+(r<<0);		 
  return(rgb);

}




void Gui_Circle(uint16_t X,uint16_t Y,uint16_t R,uint16_t fc) {//Bresenham算法 
    unsigned short  a,b; 
    int c; 
    a=0; 
    b=R; 
    c=3-2*R; 
    while (a<b) 
    { 
        Gui_DrawPoint(X+a,Y+b,fc);     //        7 
        Gui_DrawPoint(X-a,Y+b,fc);     //        6 
        Gui_DrawPoint(X+a,Y-b,fc);     //        2 
        Gui_DrawPoint(X-a,Y-b,fc);     //        3 
        Gui_DrawPoint(X+b,Y+a,fc);     //        8 
        Gui_DrawPoint(X-b,Y+a,fc);     //        5 
        Gui_DrawPoint(X+b,Y-a,fc);     //        1 
        Gui_DrawPoint(X-b,Y-a,fc);     //        4 

        if(c<0) c=c+4*a+6; 
        else 
        { 
            c=c+4*(a-b)+10; 
            b-=1; 
        } 
       a+=1; 
    } 
    if (a==b) 
    { 
        Gui_DrawPoint(X+a,Y+b,fc); 
        Gui_DrawPoint(X+a,Y+b,fc); 
        Gui_DrawPoint(X+a,Y-b,fc); 
        Gui_DrawPoint(X-a,Y-b,fc); 
        Gui_DrawPoint(X+b,Y+a,fc); 
        Gui_DrawPoint(X-b,Y+a,fc); 
        Gui_DrawPoint(X+b,Y-a,fc); 
        Gui_DrawPoint(X-b,Y-a,fc); 
    } 
	
} 
//画线函数，使用Bresenham 画线算法
void Gui_DrawLine(uint16_t x0, uint16_t y0,uint16_t x1, uint16_t y1,uint16_t Color){
int dx,             // difference in x's
    dy,             // difference in y's
    dx2,            // dx,dy * 2
    dy2, 
    x_inc,          // amount in pixel space to move during drawing
    y_inc,          // amount in pixel space to move during drawing
    error,          // the discriminant i.e. error i.e. decision variable
    index;          // used for looping	


	Lcd_SetXY(x0,y0);
	dx = x1-x0;//计算x距离
	dy = y1-y0;//计算y距离

	if (dx>=0)
	{
		x_inc = 1;
	}
	else
	{
		x_inc = -1;
		dx    = -dx;  
	} 
	
	if (dy>=0)
	{
		y_inc = 1;
	} 
	else
	{
		y_inc = -1;
		dy    = -dy; 
	} 

	dx2 = dx << 1;
	dy2 = dy << 1;

	if (dx > dy)//x距离大于y距离，那么每个x轴上只有一个点，每个y轴上有若干个点
	{//且线的点数等于x距离，以x轴递增画点
		// initialize error term
		error = dy2 - dx; 

		// draw the line
		for (index=0; index <= dx; index++)//要画的点数不会超过x距离
		{
			//画点
			Gui_DrawPoint(x0,y0,Color);
			
			// test if error has overflowed
			if (error >= 0) //是否需要增加y坐标值
			{
				error-=dx2;

				// move to next line
				y0+=y_inc;//增加y坐标值
			} // end if error overflowed

			// adjust the error term
			error+=dy2;

			// move to the next pixel
			x0+=x_inc;//x坐标值每次画点后都递增1
		} // end for
	} // end if |slope| <= 1
	else//y轴大于x轴，则每个y轴上只有一个点，x轴若干个点
	{//以y轴为递增画点
		// initialize error term
		error = dx2 - dy; 

		// draw the line
		for (index=0; index <= dy; index++)
		{
			// set the pixel
			Gui_DrawPoint(x0,y0,Color);

			// test if error overflowed
			if (error >= 0)
			{
				error-=dy2;

				// move to next line
				x0+=x_inc;
			} // end if error overflowed

			// adjust the error term
			error+=dx2;

			// move to the next pixel
			y0+=y_inc;
		} // end for
	} // end else |slope| > 1
}



void Gui_box(uint16_t x, uint16_t y, uint16_t w, uint16_t h,uint16_t bc){
	Gui_DrawLine(x,y,x+w,y,0xEF7D);
	Gui_DrawLine(x+w-1,y+1,x+w-1,y+1+h,0x2965);
	Gui_DrawLine(x,y+h,x+w,y+h,0x2965);
	Gui_DrawLine(x,y,x,y+h,0xEF7D);
    Gui_DrawLine(x+1,y+1,x+1+w-2,y+1+h-2,bc);
}

void Gui_box2(uint16_t x,uint16_t y,uint16_t w,uint16_t h, u8 mode){
	if (mode==0)	{
		Gui_DrawLine(x,y,x+w,y,0xEF7D);
		Gui_DrawLine(x+w-1,y+1,x+w-1,y+1+h,0x2965);
		Gui_DrawLine(x,y+h,x+w,y+h,0x2965);
		Gui_DrawLine(x,y,x,y+h,0xEF7D);
		}
	if (mode==1)	{
		Gui_DrawLine(x,y,x+w,y,0x2965);
		Gui_DrawLine(x+w-1,y+1,x+w-1,y+1+h,0xEF7D);
		Gui_DrawLine(x,y+h,x+w,y+h,0xEF7D);
		Gui_DrawLine(x,y,x,y+h,0x2965);
	}
	if (mode==2)	{
		Gui_DrawLine(x,y,x+w,y,0xffff);
		Gui_DrawLine(x+w-1,y+1,x+w-1,y+1+h,0xffff);
		Gui_DrawLine(x,y+h,x+w,y+h,0xffff);
		Gui_DrawLine(x,y,x,y+h,0xffff);
	}
}


void Gui_DrawFont_GBK16(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, u8 *s){
	unsigned char i,j;
	unsigned short k,x0;
	x0=x;

	while(*s) 
	{	
		if((*s) < 128) 
		{
			k=*s;
			if (k==13) 
			{
				x=x0;
				y+=16;
			}
			else 
			{
				if (k>32) k-=32; else k=0;
	
			    for(i=0;i<16;i++)
				for(j=0;j<8;j++) 
					{
				    	if(asc16[k*16+i]&(0x80>>j))	Gui_DrawPoint(x+j,y+i,fc);
						else 
						{
							if (fc!=bc) Gui_DrawPoint(x+j,y+i,bc);
						}
					}
				x+=8;
			}
			s++;
		}
			
		else 
		{
		

			for (k=0;k<hz16_num;k++) 
			{
			  if ((hz16[k].Index[0]==*(s))&&(hz16[k].Index[1]==*(s+1)))
			  { 
				    for(i=0;i<16;i++)
				    {
						for(j=0;j<8;j++) 
							{
						    	if(hz16[k].Msk[i*2]&(0x80>>j))	Gui_DrawPoint(x+j,y+i,fc);
								else {
									if (fc!=bc) Gui_DrawPoint(x+j,y+i,bc);
								}
							}
						for(j=0;j<8;j++) 
							{
						    	if(hz16[k].Msk[i*2+1]&(0x80>>j))	Gui_DrawPoint(x+j+8,y+i,fc);
								else 
								{
									if (fc!=bc) Gui_DrawPoint(x+j+8,y+i,bc);
								}
							}
				    }
				}
			  }
			s+=2;x+=16;
		} 
		
	}
}

void Gui_DrawFont_GBK24(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, u8 *s){
	unsigned char i,j;
	unsigned short k;

	while(*s) 
	{
		if( *s < 0x80 ) 
		{
			k=*s;
			if (k>32) k-=32; else k=0;

		    for(i=0;i<16;i++)
			for(j=0;j<8;j++) 
				{
			    	if(asc16[k*16+i]&(0x80>>j))	
					Gui_DrawPoint(x+j,y+i,fc);
					else 
					{
						if (fc!=bc) Gui_DrawPoint(x+j,y+i,bc);
					}
				}
			s++;x+=8;
		}
		else 
		{

			for (k=0;k<hz24_num;k++) 
			{
			  if ((hz24[k].Index[0]==*(s))&&(hz24[k].Index[1]==*(s+1)))
			  { 
				    for(i=0;i<24;i++)
				    {
						for(j=0;j<8;j++) 
							{
						    	if(hz24[k].Msk[i*3]&(0x80>>j))
								Gui_DrawPoint(x+j,y+i,fc);
								else 
								{
									if (fc!=bc) Gui_DrawPoint(x+j,y+i,bc);
								}
							}
						for(j=0;j<8;j++) 
							{
						    	if(hz24[k].Msk[i*3+1]&(0x80>>j))	Gui_DrawPoint(x+j+8,y+i,fc);
								else {
									if (fc!=bc) Gui_DrawPoint(x+j+8,y+i,bc);
								}
							}
						for(j=0;j<8;j++) 
							{
						    	if(hz24[k].Msk[i*3+2]&(0x80>>j))	
								Gui_DrawPoint(x+j+16,y+i,fc);
								else 
								{
									if (fc!=bc) Gui_DrawPoint(x+j+16,y+i,bc);
								}
							}
				    }
			  }
			}
			s+=2;x+=24;
		}
	}
}

void Gui_DrawFont_Num32(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint16_t num){
	unsigned char i,j,k,c;
	//lcd_text_any(x+94+i*42,y+34,32,32,0x7E8,0x0,sz32,knum[i]);
//	w=w/8;

    for(i=0;i<32;i++)
	{
		for(j=0;j<4;j++) 
		{
			c=*(sz32+num*32*4+i*4+j);
			for (k=0;k<8;k++)	
			{
	
		    	if(c&(0x80>>k))	Gui_DrawPoint(x+j*8+k,y+i,fc);
				else {
					if (fc!=bc) Gui_DrawPoint(x+j*8+k,y+i,bc);
				}
			}
		}
	}
}

//取模方式 水平扫描 从左到右 低位在前
void showimage(const unsigned char *p){ //显示40*40 QQ图片
  	int i,j,k; 
	unsigned char picH,picL;
	Lcd_Clear(WHITE); //清屏  
	
	for(k=0;k<4;k++)
	{
	   	for(j=0;j<3;j++)
		{	
			Lcd_SetRegion(40*j+2,40*k,40*j+39,40*k+39);		//坐标设置
		    for(i=0;i<40*40;i++)
			 {	
			 	picL=*(p+i*2);	//数据低位在前
				picH=*(p+i*2+1);				
				esp_tft.LCD_WriteData_16Bit(picH<<8|picL);  						
			 }	
		 }
	}		
}

/***************************************************************************GUI_COM**************************************************************************************/

/**************************************************************************************
功能描述: 在屏幕显示一凸起的按钮框
输    入: uint16_t x1,y1,x2,y2 按钮框左上角和右下角坐标
输    出: 无
**************************************************************************************/
void DisplayButtonDown(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2){
	Gui_DrawLine(x1,  y1,  x2,y1, GRAY2);  //H
	Gui_DrawLine(x1+1,y1+1,x2,y1+1, GRAY1);  //H
	Gui_DrawLine(x1,  y1,  x1,y2, GRAY2);  //V
	Gui_DrawLine(x1+1,y1+1,x1+1,y2, GRAY1);  //V
	Gui_DrawLine(x1,  y2,  x2,y2, WHITE);  //H
	Gui_DrawLine(x2,  y1,  x2,y2, WHITE);  //V
}

/**************************************************************************************
功能描述: 在屏幕显示一凹下的按钮框
输    入: uint16_t x1,y1,x2,y2 按钮框左上角和右下角坐标
输    出: 无
**************************************************************************************/
void DisplayButtonUp(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2){
	Gui_DrawLine(x1,  y1,  x2,y1, WHITE); //H
	Gui_DrawLine(x1,  y1,  x1,y2, WHITE); //V
	
	Gui_DrawLine(x1+1,y2-1,x2,y2-1, GRAY1);  //H
	Gui_DrawLine(x1,  y2,  x2,y2, GRAY2);  //H
	Gui_DrawLine(x2-1,y1+1,x2-1,y2, GRAY1);  //V
    Gui_DrawLine(x2  ,y1  ,x2,y2, GRAY2); //V
}

/**************************************************************************************
功能描述: 屏幕显示绘图控件
输    入: 大小，以及输入的数据数组
**************************************************************************************/
void DataBox(uint8_t x0,uint8_t x0,uint8_t x0,uint8_t x0,)