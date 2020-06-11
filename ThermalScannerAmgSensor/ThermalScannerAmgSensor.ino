/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/

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

BLEServer *pServer;
BLEService *pService;

BLECharacteristic *pTempChar;
BLECharacteristic *pDistChar;
BLECharacteristic *pLedChar;
BLECharacteristic *pVerChar;


float getTempFH()
{
  float temp = random(800,1200)/10.0; //Read temp from the device in Fahrenheit
  return(temp);
}
int getDistCm()
{
  int dist = random(40,100); //Read dist value using laser device
  return(dist);
}

void GreenLedOnOff(char on_off)
{
  if(on_off == '0')
  {
    Serial.println("Green LED Turned Off");
  }
  if(on_off == '1')
  {
    Serial.println("Green LED Turned On");
  }
  //DigitalWrite(GreenLedPin, on_off - 0x30); //on_off will get value '0'=0x30 or '1'=0x31
}
void RedLedOnOff(char on_off)
{
  if(on_off == '0')
  {
    Serial.println("Red LED Turned Off");
  }
  if(on_off == '1')
  {
    Serial.println("Red LED Turned On");
  }
  //DigitalWrite(RedLedPin, on_off - 0x30);//on_off will get value '0'=0x30 or '1'=0x31
}
void LedsOnOff(char on_off)
{
  if(on_off == '0')
  {
    Serial.println("Green LED Turned Off");    Serial.println("Red LED Turned Off");
  }
  if(on_off == '1')
  {
    Serial.println("Green LED Turned On");    Serial.println("Red LED Turned On");
  }
  
  //DigitalWrite(GreenLedPin, on_off - 0x30);//on_off will get value '0'=0x30 or '1'=0x31
  //DigitalWrite(RedLedPin, on_off - 0x30);//on_off will get value '0'=0x30 or '1'=0x31
}

/******************************************************************************************************
 * Callbacks
 ******************************************************************************************************/
class TempCharCallbacks: public BLECharacteristicCallbacks
{
  void onRead(BLECharacteristic *pCharacteristic)
  {
    char degF[] = "°F";
    char tempF[16] = {0,};
    float temp_degF = getTempFH(); //Read temp from the sensor
    sprintf(tempF,"%.01f%s",temp_degF,degF);
    pCharacteristic->setValue(tempF);
    Serial.print("Temperature Value sent: ");Serial.println(tempF);
  }
};

class DistCharCallbacks: public BLECharacteristicCallbacks
{
  void onRead(BLECharacteristic *pCharacteristic)
  {
    char bufDist[16] = {0,};
    int distCm = getDistCm(); //Measure distance using laser device
    sprintf(bufDist,"%d",distCm);
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

void setup()
{
  Serial.begin(115200);
  Serial.print(VERSION_STR);

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
  Serial.println("DWP Temp Scanner Ready!");
}

void loop()
{
  delay(1000);
}
