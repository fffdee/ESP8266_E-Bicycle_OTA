#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <ESP8266mDNS.h>
#include <SPI.h>

#ifndef APSSID
#define APSSID "ESP8266AP"
#define APPSK  "88888888"
#endif

#define X_MAX_PIXEL 128
#define Y_MAX_PIXEL 160

#define RED  	0xf800
#define GREEN	0x07e0
#define BLUE 	0x001f
#define WHITE	0xffff
#define BLACK	0x0000
#define YELLOW  0xFFE0
#define GRAY0   0xEF7D   
#define GRAY1   0x8410    
#define GRAY2   0x4208   

#define  DC       0
#define  BLK      16
#define  MOSI     13
#define  SCLK     14
#define  CS       15
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
  }

  void Lcd_SetRegion(uint16_t x_start,uint16_t y_start,uint16_t x_end,uint16_t y_end)
{		
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

};

ESPTFT esp_tft(SS);





/* Set these to your desired credentials. */
const char* host = "esp8266-webupdate";
const char *ssid = APSSID;
const char *password = APPSK;
IPAddress myIP;
ESP8266WebServer server(80);
const char* serverIndex = "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";

void handleRoot() {
 
                       // wait for a second
  digitalWrite(2, HIGH);   // turn the LED off by making the voltage LOW
  delay(200);  
   server.send(200, "text/html", "ok");
}

// 任务函数1
void Task1(void *pvParameters) {
  for (;;) {
    Serial.println("Task 1 is running");
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 每秒运行一次
  }
}

// 任务函数2
void Task2(void *pvParameters) {
  for (;;) {
    Serial.println("Task 2 is running");
    vTaskDelay(2000 / portTICK_PERIOD_MS); // 每两秒运行一次
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  esp_tft.begin();
  
  esp_tft.Lcd_Init();
  
  esp_tft.Lcd_Clear(RED);
  Serial.print("Configuring access point...");
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAP(ssid, password);

  myIP = WiFi.softAPIP();
  MDNS.begin(host);
  Serial.print("AP IP address: ");
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
          Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        } else {
          Update.printError(Serial);
        }
        Serial.setDebugOutput(false);
      }
      yield();
    });
  server.begin();
  MDNS.addService("http", "tcp", 80);

  Serial.printf("Ready! Open http://%s.local in your browser\n", host);
  Serial.println("HTTP server started");
  esp_tft.Lcd_Clear(BLUE);
  
  esp_tft.Lcd_Clear(GRAY0);
  
  esp_tft.Lcd_Clear(GREEN);
  
  esp_tft.Lcd_Clear(WHITE);
  
  esp_tft.Lcd_Clear(RED);
  
  esp_tft.Lcd_Clear(GRAY2);
  
}
void loop() {
  // put your main code here, to run repeatedly:
  server.handleClient();
  MDNS.update();
  
 

  
}
