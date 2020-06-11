/*
 ********************************************************************
 This is a software for Thermal Sensor Module For ESp32 
 Thermal Sensor is OMRON D6T-01A-01 and Laser Distance Module (VL53L0X)
 Written by Dr Azam, Bhaskar
 ********************************************************************
 ESP32 Bluetooth example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
 Ported to Arduino ESP32 by Evandro Copercini updates by chegewara
*/

#include <string.h>
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID     "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define TEMP_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DIST_CHAR_UUID   "beb5483e-36e2-4688-b7f5-ea07361b26a8"
#define LED_CHAR_UUID    "beb5483e-36e3-4688-b7f5-ea07361b26a8"
#define VER_CHAR_UUID    "beb5483e-36e4-4688-b7f5-ea07361b26a8"

#define VERSION_STR "DWP Temp Scanner Version: 1.0, "__DATE__

#define READ_BUFSIZE      (20)
#define GRN 15
#define RED 2

/* defines for OMRON Thermal Sensor*/
#define D6T_ADDR 0x0A  // for I2C 7bit address
#define D6T_CMD 0x4C  // for D6T-44L-06/06H, D6T-8L-09/09H, for D6T-1A-01/02

#define N_ROW 1
#define N_PIXEL 1
#define N_READ ((N_PIXEL + 1) * 2 + 1)

#define SAMPLE_TIME_0009MS  9
#define SAMPLE_TIME_0010MS  10
#define SAMPLE_TIME_0012MS  12
#define SAMPLE_TIME_0015MS  15
#define SAMPLE_TIME_0020MS  20
#define SAMPLE_TIME_0040MS  40
#define SAMPLE_TIME_0060MS  60
#define SAMPLE_TIME_0100MS  100
#define SAMPLE_TIME_0200MS  200
#define SAMPLE_TIME_0400MS  400
#define SAMPLE_TIME_0800MS  800
#define SAMPLE_TIME_1600MS  1600
#define SAMPLE_TIME_3200MS  3200

#define PARA_0009MS_1 ((uint8_t)0x90)
#define PARA_0009MS_2 ((uint8_t)0xD3)
#define PARA_0009MS_3 ((uint8_t)0x29)
#define PARA_0010MS_1 ((uint8_t)0x90)
#define PARA_0010MS_2 ((uint8_t)0xD4)
#define PARA_0010MS_3 ((uint8_t)0x3C)
#define PARA_0012MS_1 ((uint8_t)0x90)
#define PARA_0012MS_2 ((uint8_t)0xD5)
#define PARA_0012MS_3 ((uint8_t)0x3B)
#define PARA_0015MS_1 ((uint8_t)0x90)
#define PARA_0015MS_2 ((uint8_t)0xD6)
#define PARA_0015MS_3 ((uint8_t)0x32)
#define PARA_0020MS_1 ((uint8_t)0x90)
#define PARA_0020MS_2 ((uint8_t)0xD7)
#define PARA_0020MS_3 ((uint8_t)0x35)
#define PARA_0040MS_1 ((uint8_t)0x90)
#define PARA_0040MS_2 ((uint8_t)0xD8)
#define PARA_0040MS_3 ((uint8_t)0x18)
#define PARA_0060MS_1 ((uint8_t)0x90)
#define PARA_0060MS_2 ((uint8_t)0xD9)
#define PARA_0060MS_3 ((uint8_t)0x1F)
#define PARA_0100MS_1 ((uint8_t)0x90)
#define PARA_0100MS_2 ((uint8_t)0xDA)
#define PARA_0100MS_3 ((uint8_t)0x16)
#define PARA_0200MS_1 ((uint8_t)0x90)
#define PARA_0200MS_2 ((uint8_t)0xDB)
#define PARA_0200MS_3 ((uint8_t)0x11)
#define PARA_0400MS_1 ((uint8_t)0x90)
#define PARA_0400MS_2 ((uint8_t)0xDC)
#define PARA_0400MS_3 ((uint8_t)0x04)
#define PARA_0800MS_1 ((uint8_t)0x90)
#define PARA_0800MS_2 ((uint8_t)0xDD)
#define PARA_0800MS_3 ((uint8_t)0x03)
#define PARA_1600MS_1 ((uint8_t)0x90)
#define PARA_1600MS_2 ((uint8_t)0xDE)
#define PARA_1600MS_3 ((uint8_t)0x0A)
#define PARA_3200MS_1 ((uint8_t)0x90)
#define PARA_3200MS_2 ((uint8_t)0xDF)
#define PARA_3200MS_3 ((uint8_t)0x0D)

/***** Setting Parameter 1 *****/
#define comparingNumInc 16 // x samplingTime ms   (range: 1 to 39)  (example) 16 x 100 ms -> 1.6 sec
#define comparingNumDec 16  // x samplingTime ms  (range: 1 to 39)  (example) 16 x 100 ms -> 1.6 sec
#define threshHoldInc 10 //  /10 degC   (example) 10 -> 1.0 degC (temperature change > 1.0 degC -> Enable)  
#define threshHoldDec 10 //  /10 degC   (example) 10 -> 1.0 degC (temperature change > 1.0 degC -> Disable)
//bool  enablePix[8] = {true, true, true, true, true, true, true, true};
/****************************/

/***** Setting Parameter 2 *****/
#define samplingTime SAMPLE_TIME_0100MS //ms (Can select only, 9ms, 10ms, 12ms, 15ms, 20ms, 40ms, 60ms, 100ms, 200ms, 400ms, 800ms, 1600ms, 3200ms)
/****************************/

BLEServer *pServer;
BLEService *pService;

BLECharacteristic *pTempChar;
BLECharacteristic *pDistChar;
BLECharacteristic *pLedChar;
BLECharacteristic *pVerChar;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// 21-SDA, 22-SCL for VL53L0X and OMRON

// ************* Functions decalarion here *************************

uint8_t readPacket (uint16_t timeout);
float   parsefloat (uint8_t *buffer);

// ************ General Variables used ***************************

char packetbuffer[READ_BUFSIZE+1];
String packetbuf;
char charBuf[25];
int val;

String ver = "DWP Temp Scanner Version: 1.0";

int pulsew=1; // milliseconds
int delaypluse=1; // delay between pulses
int i,j,k,qfl;
long duration, cm, inches;
float AmbTempC,ObjTempC,CtempC,Distance;

// Variables for OMRON sensor

uint8_t rbuf[N_READ];
int16_t pix_data = 0;
int16_t seqData[40] = {0};
bool  occuPix = 0;
bool  occuPixFlag = false;
uint16_t  totalCount = 0;

float getTempCH()
{
 // float temp = random(800,1200)/10.0; //Read temp from the device in Fahrenheit
  ReadTemp();
  return(ObjTempC);
}
int getDistMm()
{
 // int dist = random(40,100); //Read dist value using laser device
  ReadDistance();
  return(Distance);
}

void TestLEDs()
  {
  pinMode(GRN, OUTPUT);  // Green LED
  pinMode(RED, OUTPUT);  // Red LED
  digitalWrite(GRN,HIGH);     // Green LED ON
  delay(1000);                       // wait for a second
  digitalWrite(GRN,LOW);     // Green LED OFF
  delay(200);   
  digitalWrite(RED,HIGH);     // Green LED ON
  delay(1000);                       // wait for a second
  digitalWrite(RED,LOW);     // Green LED OFF
  } 


void GreenLedOnOff(char on_off)
{
  if(on_off == '0')
  {
    Serial.println("Green LED Turned Off");
    digitalWrite(GRN,LOW);
  }
  if(on_off == '1')
  {
    Serial.println("Green LED Turned On");
    digitalWrite(GRN,HIGH);
  }
}
void RedLedOnOff(char on_off)
{
  if(on_off == '0')
  {
    Serial.println("Red LED Turned Off");
    digitalWrite(RED,LOW);  
  }
  if(on_off == '1')
  {
    Serial.println("Red LED Turned On");
    digitalWrite(RED,HIGH); 
  }

}
void LedsOnOff(char on_off)
{
  if(on_off == '0')
  {
    Serial.println("Green LED Turned Off");    Serial.println("Red LED Turned Off");
    digitalWrite(GRN,LOW);     digitalWrite(RED,LOW);    
  }
  
  if(on_off == '1')
  {
    Serial.println("Green LED Turned On");    Serial.println("Red LED Turned On");
    digitalWrite(GRN,HIGH);    digitalWrite(RED,HIGH); 
  }
  
 }

/******************************************************************************************************
 * Callbacks
 ******************************************************************************************************/
class TempCharCallbacks: public BLECharacteristicCallbacks
{
  void onRead(BLECharacteristic *pCharacteristic)
  {
    char degC[] = "°C";
    char tempC[16] = {0,};
    float temp_degC = getTempCH(); //Read temp from the sensor
    sprintf(tempC,"%.01f%s",temp_degC,degC);
    pCharacteristic->setValue(tempC);
    Serial.print("Temperature Value sent: ");Serial.println(tempC);
  }
};

class DistCharCallbacks: public BLECharacteristicCallbacks
{
  void onRead(BLECharacteristic *pCharacteristic)
  {
    char bufDist[16] = {0,};
    int distMm = getDistMm(); //Measure distance using laser device
    sprintf(bufDist,"%d",distMm);
    strcat(bufDist," cm");
    pCharacteristic->setValue(bufDist);
    Serial.print("Distance: ");Serial.println(bufDist);
  }
};

class LedCharCallbacks: public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();

    if (value.length() > 0)
    {
      Serial.print("Received: ");
      for (int i = 0; i < value.length(); i++)
      {
        Serial.print(value[i]);
      }
      Serial.println();   
      //App writes R0/1 for Red Led Off/On, G0/1 for Green led Off/On, A0/1 for all the leds Off/On 
      switch(value[0])
      {
        case 'R':
        case 'r':
          RedLedOnOff(value[1]);
          break;
        case 'G':
        case 'g':
          GreenLedOnOff(value[1]);
          break;
        case 'A':
        case 'a':
          LedsOnOff(value[1]);
          break;
      }
    }
  }
};

class VerCharCallbacks: public BLECharacteristicCallbacks
{
  void onRead(BLECharacteristic *pCharacteristic)
  {
    pCharacteristic->setValue(VERSION_STR);
    Serial.println(VERSION_STR);
  }
};

// ********************** Setup Code here ***************************


void setup()
{
  Serial.begin(115200);
  Serial.print(VERSION_STR);
  TestLEDs();
  SetupBlue();
  SetupVL53LOX();
  SetupOMRON();
  
// Serial.println("DWP Temp Scanner Ready!");
}

void loop()
{
  ReadCommands();
  ProcessData();
}

// ***************** Subroutines ***************************

void ReadCommands()
{
  uint8_t len = readPacket(100);  // Wait for new data arrive
  if (len == 0) return;
  charToString(packetbuffer,packetbuf);
 // Serial.println(packetbuf); // for debug
  if (packetbuffer[1] == 'V')   // Version
      DispVer();
  if (packetbuffer[1] == 'T')   // Read Temperature
      ReadTemp(); 
  if (packetbuffer[1] == 'D')   // Read Distance
      ReadDistance();
  if (packetbuffer[1] == 'A')   // Measurement Mode
      AutoMeasure();
  if (packetbuffer[1] == 'Q')   // Quit from Measurement
      QuitMeasure();   
}
 
void ProcessData()
{
if(qfl==1)
  {
  digitalWrite(GRN,HIGH); 
  ReadDistance();
  ReadTemp();  
  delay(500);
  }
}


void ReadDistance()
{
 VL53L0X_RangingMeasurementData_t measure;
    
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) 
  {  
    Distance=measure.RangeMilliMeter;
    Serial.print("Distance(mm): "); 
    Serial.print(measure.RangeMilliMeter);
  } 
  else 
  {
    Distance=0;
    Serial.print("Distance(mm): "); 
    Serial.print("Out of range ");
  }
  Serial.println();
}

void ReadTemp() 

    {
    int i, j;

    memset(rbuf, 0, N_READ);
    Wire.beginTransmission(D6T_ADDR);  // I2C client address
    Wire.write(D6T_CMD);               // D6T register
    Wire.endTransmission();            // I2C repeated start for read
    Wire.requestFrom(D6T_ADDR, N_READ);
    i = 0;
    while (Wire.available()) {
        rbuf[i++] = Wire.read();
    }

    if (D6T_checkPEC(rbuf, N_READ - 1)) {
        return;
    }

    // 1st data is PTAT measurement (: Proportional To Absolute Temperature)
    int16_t itemp = conv8us_s16_le(rbuf, 0);
    Serial.print("AmbientTemp:");
    AmbTempC=itemp/10.0;
    Serial.print(AmbTempC, 1);    // Print Abient Temperature
    // loop temperature pixels of each thrmopiles measurements
    for (i = 0, j = 2; i < N_PIXEL; i++, j += 2) {
        itemp = conv8us_s16_le(rbuf, j);
        pix_data = itemp;
        ObjTempC=pix_data/10.0;   // Print Object Temperature
        Serial.print(",ObjTemp:");  
        Serial.print(ObjTempC, 1);  
        if ((i % N_ROW) == N_ROW - 1) {
            Serial.print("[C]");  // wrap text at ROW end.
        } else {
            Serial.print(",");   // print delimiter
        }
    }
    delay(samplingTime);
    Serial.println();
}

void AutoMeasure()
{
  qfl=1;
  digitalWrite(GRN,HIGH); 
}
void QuitMeasure()
{
  qfl=0;
  digitalWrite(GRN,LOW); 
}

 void DispVer()
 {  
  Serial.println(ver);
 }


// ************** VL53L0X Laser distance module routines ************


//=============Subroutines for OMRON ===============================

uint8_t calc_crc(uint8_t data) {
    int index;
    uint8_t temp;
    for (index = 0; index < 8; index++) {
        temp = data;
        data <<= 1;
        if (temp & 0x80) {data ^= 0x07;}
    }
    return data;
}

/** <!-- D6T_checkPEC {{{ 1--> D6T PEC(Packet Error Check) calculation.
 * calculate the data sequence,
 * from an I2C Read client address (8bit) to thermal data end.
 */
bool D6T_checkPEC(uint8_t buf[], int n) {
    int i;
    uint8_t crc = calc_crc((D6T_ADDR << 1) | 1);  // I2C Read address (8bit)
    for (i = 0; i < n; i++) {
        crc = calc_crc(buf[i] ^ crc);
    }
    bool ret = crc != buf[n];
    if (ret) {
        Serial.print("PEC check failed:");
        Serial.print(crc, HEX);
        Serial.print("(cal) vs ");
        Serial.print(buf[n], HEX);
        Serial.println("(get)");
    }
    return ret;
}


/** <!-- conv8us_s16_le {{{1 --> convert a 16bit data from the byte stream.
 */
int16_t conv8us_s16_le(uint8_t* buf, int n) {
    int ret;
    ret = buf[n];
    ret += buf[n + 1] << 8;
    return (int16_t)ret;   // and convert negative.
}

//******** Setup VL53LOX Laser Distance Sensor *******

void SetupVL53LOX()
{
 if (!lox.begin()) 
 {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
 }
}


//******** Setup OMRON Temperature Sensor**********

void SetupOMRON() 
{
  uint8_t para[3] = {0};
  switch(samplingTime){
    case SAMPLE_TIME_0009MS:
      para[0] = PARA_0009MS_1;
      para[1] = PARA_0009MS_2;
      para[2] = PARA_0009MS_3;
      break;
    case SAMPLE_TIME_0010MS:
      para[0] = PARA_0010MS_1;
      para[1] = PARA_0010MS_2;
      para[2] = PARA_0010MS_3;
      break;
    case SAMPLE_TIME_0012MS:
      para[0] = PARA_0012MS_1;
      para[1] = PARA_0012MS_2;
      para[2] = PARA_0012MS_3;
      break;
    case SAMPLE_TIME_0015MS:
      para[0] = PARA_0015MS_1;
      para[1] = PARA_0015MS_2;
      para[2] = PARA_0015MS_3;
      break;
    case SAMPLE_TIME_0020MS:
      para[0] = PARA_0020MS_1;
      para[1] = PARA_0020MS_2;
      para[2] = PARA_0020MS_3;
      break;
    case SAMPLE_TIME_0040MS:
      para[0] = PARA_0040MS_1;
      para[1] = PARA_0040MS_2;
      para[2] = PARA_0040MS_3;
      break;
    case SAMPLE_TIME_0060MS:
      para[0] = PARA_0060MS_1;
      para[1] = PARA_0060MS_2;
      para[2] = PARA_0060MS_3;
      break;
    case SAMPLE_TIME_0100MS:
      para[0] = PARA_0100MS_1;
      para[1] = PARA_0100MS_2;
      para[2] = PARA_0100MS_3;
      break;
    case SAMPLE_TIME_0200MS:
      para[0] = PARA_0200MS_1;
      para[1] = PARA_0200MS_2;
      para[2] = PARA_0200MS_3;
      break;
    case SAMPLE_TIME_0400MS:
      para[0] = PARA_0400MS_1;
      para[1] = PARA_0400MS_2;
      para[2] = PARA_0400MS_3;
      break;
    case SAMPLE_TIME_0800MS:
      para[0] = PARA_0800MS_1;
      para[1] = PARA_0800MS_2;
      para[2] = PARA_0800MS_3;
      break;
    case SAMPLE_TIME_1600MS:
      para[0] = PARA_1600MS_1;
      para[1] = PARA_1600MS_2;
      para[2] = PARA_1600MS_3;
      break;
    case SAMPLE_TIME_3200MS:
      para[0] = PARA_3200MS_1;
      para[1] = PARA_3200MS_2;
      para[2] = PARA_3200MS_3;
      break;
    default:
      para[0] = PARA_0100MS_1;
      para[1] = PARA_0100MS_2;
      para[2] = PARA_0100MS_3;
      break;
  }
  
    Wire.begin();  // i2c master

    Wire.beginTransmission(D6T_ADDR);  // I2C client address
    Wire.write(0x02);                  // D6T register
    Wire.write(0x00);                  // D6T register
    Wire.write(0x01);                  // D6T register
    Wire.write(0xEE);                  // D6T register
    Wire.endTransmission();            // I2C repeated start for read
    Wire.beginTransmission(D6T_ADDR);  // I2C client address
    Wire.write(0x05);                  // D6T register
    Wire.write(para[0]);                  // D6T register
    Wire.write(para[1]);                  // D6T register
    Wire.write(para[2]);                  // D6T register
    Wire.endTransmission();            // I2C repeated start for read
    Wire.beginTransmission(D6T_ADDR);  // I2C client address
    Wire.write(0x03);                  // D6T register
    Wire.write(0x00);                  // D6T register
    Wire.write(0x03);                  // D6T register
    Wire.write(0x8B);                  // D6T register
    Wire.endTransmission();            // I2C repeated start for read
    Wire.beginTransmission(D6T_ADDR);  // I2C client address
    Wire.write(0x03);                  // D6T register
    Wire.write(0x00);                  // D6T register
    Wire.write(0x07);                  // D6T register
    Wire.write(0x97);                  // D6T register
    Wire.endTransmission();            // I2C repeated start for read
    Wire.beginTransmission(D6T_ADDR);  // I2C client address
    Wire.write(0x02);                  // D6T register
    Wire.write(0x00);                  // D6T register
    Wire.write(0x00);                  // D6T register
    Wire.write(0xE9);                  // D6T register
    Wire.endTransmission();            // I2C repeated start for read 

}

// *********** Setup Bluetooth **********************


void SetupBlue()
{
  BLEDevice::init("DWP TempScan");
  pServer = BLEDevice::createServer();
  pService = pServer->createService(SERVICE_UUID);

  pTempChar = pService->createCharacteristic(TEMP_CHAR_UUID,BLECharacteristic::PROPERTY_READ);// | BLECharacteristic::PROPERTY_WRITE);
  pDistChar = pService->createCharacteristic(DIST_CHAR_UUID,BLECharacteristic::PROPERTY_READ);// | BLECharacteristic::PROPERTY_WRITE);
  pLedChar = pService->createCharacteristic(LED_CHAR_UUID,/*BLECharacteristic::PROPERTY_READ |*/ BLECharacteristic::PROPERTY_WRITE);
  pVerChar = pService->createCharacteristic(VER_CHAR_UUID,BLECharacteristic::PROPERTY_READ);// | BLECharacteristic::PROPERTY_WRITE);

  pTempChar->setCallbacks(new TempCharCallbacks());
  pDistChar->setCallbacks(new DistCharCallbacks());
  pLedChar->setCallbacks(new LedCharCallbacks());
  pVerChar->setCallbacks(new VerCharCallbacks());

  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}


//************************************************************


uint8_t  readPacket(uint16_t timeout) 
{
  uint16_t origtimeout = timeout, replyidx = 0;
  while (timeout--) 
  {
    if (replyidx >= 20) break;
    while (Serial.available()) 
    {
      char c =  Serial.read();
      if (c == '!') {
        replyidx = 0;
      }
      packetbuffer[replyidx] = c;
      replyidx++;
      timeout = origtimeout;
    }
    if (timeout == 0) break;
    delay(1);
  }
  packetbuffer[replyidx] = 0;  // null term
  if (!replyidx)  // no data or timeout 
    return 0;
  if (packetbuffer[0] != '!')  // doesn't start with '!' packet beginning
    return 0;
 return replyidx;
}

float parsefloat(uint8_t *buffer) 
{
  float f;
  memcpy(&f, buffer, 4);
  return f;
}

void charToString(char S[], String &D)
{
 String rc(S);
 D = rc;
}

//==============================================================
 
