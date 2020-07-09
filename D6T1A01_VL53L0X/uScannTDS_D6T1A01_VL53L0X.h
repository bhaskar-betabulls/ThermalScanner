/*********************************************************************
 This is a software for Thermal Sensor Module For Nano 
 coupled with Laser Distance Module (VL53L0X)
 Written by Dr Azam, Date : 13th May 2020
 Serial port to give commands
 
 Laser Sensor for Distance Measurment using VL53L0X
 
 */
 
// ********************************************************************/

#define READ_BUFSIZE                    (20)
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

//Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// 21-SDA, 22-SCL for VL53L0X and OMRON


// ************* Functions decalarion here *************************

uint8_t readPacket (uint16_t timeout);
float   parsefloat (uint8_t *buffer);

char packetbuffer[READ_BUFSIZE+1];
String packetbuf;
char charBuf[25];
int val;

// ************ Variables used ***************************

String ver = "Thermal Sensor FW Ver 1.0";

int pulsew=1; // milliseconds
int delaypluse=1; // delay between pulses
int i,j,k,qfl;
long duration, cm, inches;
float AmbTempC,ObjTempC,CtempC,Distance;
int dist_th_mm = 800; //millimeter (~3feet)

// Variables for OMRON sensor

uint8_t rbuf[N_READ];
int16_t pix_data = 0;
int16_t seqData[40] = {0};
bool  occuPix = 0;
bool  occuPixFlag = false;
uint8_t  resultOccupancy = 0;
uint16_t  totalCount = 0;

//y2 = 1.14062 + 0.0004250978*x - 4.421823e-7*x^2 + 1.598316e-10*x^3

double c1 = 1.14062;
double c2 = 0.0004250978;
double c3 = 4.421823e-7;
double c4 = 1.598316e-10;
double corr_factor;
// ************* Setup functions *************************

// ************ Loop Function ***************************
