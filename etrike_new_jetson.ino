#include <ArduinoJson.h>
#include <Arduino.h>
#include <OneWire.h>
#include <DS2438.h>
#include <DallasTemperature.h>
#include <SimpleTimer.h>
#include <mcp_can.h>
#include <ADS1X15.h>
#include <Wire.h>
#include <PCF8574.h>

StaticJsonDocument<5000> voltage;
StaticJsonDocument<5000> temperature;
StaticJsonDocument<5000> arus;
StaticJsonDocument<5000> val;

const int SPI_CS_PIN = 5;
const uint8_t ONE_WIRE_PIN = 4;
OneWire oneWire(ONE_WIRE_PIN);
OneWire ow(ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);
MCP_CAN CAN(SPI_CS_PIN);
ADS1115 ADS(0x48); 
float v_sum,v_val,t_sum,t_val,i_sum,i_val;
int count = -1;
float Vsel[20];
float Tsel[20];
float Isel[1];

unsigned int canId = CAN.getCanId();
unsigned char len = 0;
unsigned char buf[1];
bool charge, discharge,lastcharge;
PCF8574 pcf8574(0x20);

//cell monitoring
uint8_t DS2438_address[20][10] = {
  {0x26, 0xD2, 0xE3, 0x30, 0x00, 0x00, 0x00, 0x17}, // 1
  {0x26, 0xFB, 0xC7, 0x33, 0x00, 0x00, 0x00, 0x58}, // 2
  {0x26, 0x91, 0x4E, 0x33, 0x00, 0x00, 0x00, 0x6C}, // 3
  {0x26, 0xB2, 0xF7, 0x33, 0x00, 0x00, 0x00, 0x3F}, // 4
  {0x26, 0x68, 0x23, 0x04, 0x00, 0x00, 0x00, 0xBB}, // 5
  {0x26, 0x1C, 0xCA, 0x33, 0x00, 0x00, 0x00, 0x18}, // 6
  {0x26, 0x1C, 0x8A, 0x34, 0x00, 0x00, 0x00, 0x77}, // 7
  {0x26, 0x51, 0x07, 0x31, 0x00, 0x00, 0x00, 0xEE}, // 8
  {0x26, 0xE8, 0x14, 0x34, 0x00, 0x00, 0x00, 0xCC}, // 9
  {0x26, 0xE1, 0x4A, 0x2B, 0x00, 0x00, 0x00, 0xCF}, // 10
  {0x26, 0x45, 0x1C, 0x34, 0x00, 0x00, 0x00, 0xE4}, // 11
  {0x26, 0x39, 0xF9, 0x33, 0x00, 0x00, 0x00, 0x8F}, // 12
  {0x26, 0x70, 0x02, 0x34, 0x00, 0x00, 0x00, 0x3C}, // 13
  {0x26, 0x25, 0x5E, 0x34, 0x00, 0x00, 0x00, 0x4D}, // 14
  {0x26, 0xF8, 0x4E, 0x34, 0x00, 0x00, 0x00, 0xBF}, // 15
  {0x26, 0x1C, 0xEA, 0x33, 0x00, 0x00, 0x00, 0xE0}, // 16
  {0x26, 0x09, 0x89, 0x2F, 0x00, 0x00, 0x00, 0x25}, // 17
  {0x26, 0xAE, 0xB6, 0x33, 0x00, 0x00, 0x00, 0x3D}, // 18
  {0x26, 0xA5, 0xCD, 0x30, 0x00, 0x00, 0x00, 0x50}, // 19
  {0x26, 0x0A, 0x83, 0x34, 0x00, 0x00, 0x00, 0x6D} // 20
  };

void ambildata(void * parameter){
  while(true){
    unsigned long start = millis();
    if (count > 4) {
      count = 0;
    }
    
    if (count < 0) {
      

      for (int x = 1; x < 21; x++){
        DS2438 ds2438(&ow, DS2438_address[x-1]);
        ds2438.begin();
        delay(10);
        ds2438.update();
        
        voltage[String(x)][0] = ds2438.getVoltage(DS2438_CHA); 
        voltage[String(x)][1] = voltage[String(x)][0];
        voltage[String(x)][2] = voltage[String(x)][0];
        voltage[String(x)][3] = voltage[String(x)][0];
        voltage[String(x)][4] = voltage[String(x)][0];

        temperature[String(x)][0] = ds2438.getTemperature(); 
        temperature[String(x)][1] = temperature[String(x)][0];
        temperature[String(x)][2] = temperature[String(x)][0];
        temperature[String(x)][3] = temperature[String(x)][0];
        temperature[String(x)][4] = temperature[String(x)][0];

        arus[String(1)][0] = ADS.readADC_Differential_0_1();
        arus[String(1)][1] = arus[String(1)][0];
        arus[String(1)][2] = arus[String(1)][0];
        arus[String(1)][3] = arus[String(1)][0];
        arus[String(1)][4] = arus[String(1)][0]; 

      }
    }
    
    else {
      
      for (int x = 1; x < 21; x++){
        DS2438 ds2438(&ow, DS2438_address[x-1]);
        ds2438.begin();
        delay(10);
        ds2438.update();
        voltage[String(x)][count] = ds2438.getVoltage(DS2438_CHA);
        temperature[String(x)][count] = ds2438.getTemperature();
        arus[String(1)][count] = ADS.readADC_Differential_0_1();
      }
    }
    
    unsigned long time_end = millis();
    Serial.print("get data : ");
    count++;
    Serial.println(time_end - start);
    vTaskDelay(1000);
  }
}

void average(void * parameter){
  while(true){
    unsigned long start = millis();
    for (int i = 0 ; i < 5 ; i++){
        i_val = arus[String(1)][i];
        i_sum = i_sum + i_val;
      }
    Isel[0] = map(i_sum,0,9600,0,20000);
    //Serial.print(Isel[0]);  
    for (int x = 1; x < 21; x++){
      v_sum = 0;
      t_sum = 0;
      for (int i = 0 ; i < 5 ; i++){
        v_val = voltage[String(x)][i];
        v_sum = v_sum + v_val;
        t_val = temperature[String(x)][i];
        t_sum = t_sum + t_val;
      }
      Vsel[x-1] = v_sum/5;
      Tsel[x-1] = t_sum/5;

      Serial.print("{"); 
      Serial.print("\"ID\":");  
      Serial.print(x);
      Serial.print(",  ");
      Serial.print("\"Tegangan\":"); 
      Serial.print(Vsel[x-1]);
      Serial.print(",  ");
      Serial.print("\"Suhu\":"); 
      Serial.print(Tsel[x-1]);
      Serial.print(",  ");
      Serial.print("\"Arus\":"); 
      Serial.print(Isel[0]);
      Serial.print("}"); 
      Serial.println();
     
    }
   
  //serializeJson(val, Serial);
  //Serial.println();
  unsigned long time_end = millis();
  Serial.print("moving average : ");
  Serial.println(time_end - start);
  Serial.println("-------------------------------------------");
  vTaskDelay(5000);
  }
}

void senddata(void * parameter){
  while(true){
    if(CAN_MSGAVAIL == CAN.checkReceive())            // cek bila ada data masuk
    {       
        CAN.readMsgBuf(&len, buf);    
        unsigned int canId = CAN.getCanId();
        unsigned int idd = buf[0] ;
        unsigned int cmd = buf[1] ;
        if (cmd == 1){
              int value1x = Vsel[idd-1]*100;
              int value2x = Tsel[idd-1]*100;
              int value3x = Isel[0]*100;
              byte x1 = highByte(value1x);
              byte x2 = lowByte(value1x);
              byte x3 = highByte(value2x);
              byte x4 = lowByte(value2x);
              byte x5 = highByte(idd*100);
              byte x6 = lowByte(idd*100);
              byte x7 = highByte(value3x);
              byte x8 = lowByte(value3x);
              unsigned char stmp[8] = {x5,x6,x1,x2,x3,x4,x7,x8};
              CAN.sendMsgBuf(101, 0, 8, stmp);}
              
        else if (canId == 200){
              if (buf[0] == 1) {
                Serial.println("Charge 1");
                charge = HIGH;
                }
              else if (buf[0] == 0){
                Serial.println("Charge 0");
                charge = LOW;
                }
                
              if (buf[1] == 1) {
                Serial.println("Discharge 1");
                discharge = HIGH;
                }
              else if (buf[1] == 0){
                Serial.println("Discharge 0");
                discharge = LOW;
                }  

              if(discharge == HIGH){
                Serial.println("discharge high");
                pcf8574.digitalWrite(P2, HIGH);
              }
              else{
                Serial.println("discharge low");
                pcf8574.digitalWrite(P2, LOW);
              }

              delay(2000);
              
              if(charge != lastcharge){
                Serial.println("logic on");
                if(charge == HIGH){
                  
                Serial.println("charge high");
                pcf8574.digitalWrite(P0, HIGH);
                delay(2000);
                pcf8574.digitalWrite(P0, LOW);
                
              }
                else{
                  Serial.println("charge low");
                  pcf8574.digitalWrite(P1, HIGH);
                  delay(2000);
                  pcf8574.digitalWrite(P1, LOW);
                  
                }
              }
              lastcharge = charge;
              Serial.println(lastcharge);
                }
  
  }
  }
}

void control(void * parameter){
  while(true){
    if(CAN_MSGAVAIL == CAN.checkReceive())            // cek bila ada data masuk
    {       
        CAN.readMsgBuf(&len, buf);    
        unsigned int canId = CAN.getCanId();
        unsigned int idd = buf[0] ;
        if (canId == 200){
              if (buf[0] == 1) {
                Serial.print("Charge");
                }
              if (buf[1] == 1) {
                Serial.print("Discharge");
                }}
//              delay(100);
//              Serial.println(idd);
//              1++;
  }
  }
}

void setup() {
  Serial.begin(115200);
  
  START_INIT:

    if(CAN_OK == CAN.begin(CAN_250KBPS, MCP_8MHz))                   // init can bus : baudrate = 500k
    {
        Serial.println("CAN Connect");
    }
    else
    {
        Serial.println("CANT Connect");
        Serial.println("Check CAN BUS Connection");
        delay(100);
        goto START_INIT;
    }
    //CAN.init_Mask(0, 0, 0x3ff);                         // there are 2 mask in mcp2515, you need to set both of them
    //CAN.init_Mask(1, 0, 0x3ff);
    //CAN.init_Filt(0, 0, 101);
    
  ADS.begin();
  ADS.setGain(16);
  
  charge = HIGH;
  discharge = HIGH;
  lastcharge = LOW;
  
  pcf8574.pinMode(P0, OUTPUT);
  pcf8574.pinMode(P1, OUTPUT);
  pcf8574.pinMode(P2, OUTPUT);

  pcf8574.begin();
  pcf8574.digitalWrite(P0, LOW);
  pcf8574.digitalWrite(P1, LOW);
  pcf8574.digitalWrite(P2, LOW);
  xTaskCreatePinnedToCore(
    ambildata
    ,  "ambil data one-wire"
    ,  5000  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL 
    ,  CONFIG_ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    average
    ,  "moving average 5 data"
    ,  5000  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL 
    ,  CONFIG_ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    senddata
    ,  "send data"
    ,  5000  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL 
    ,  CONFIG_ARDUINO_RUNNING_CORE);

/*  xTaskCreatePinnedToCore(
    control
    ,  "control"
    ,  5000  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL 
    ,  CONFIG_ARDUINO_RUNNING_CORE);
*/
}

//unsigned char stmp[8] = {0, 1, 2, 3, 4, 5, 6, 7};
void loop()
{
  
  }
