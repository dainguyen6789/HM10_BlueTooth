// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "math.h"
#include <SoftwareSerial.h>


#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
using namespace std;
MPU6050 mpu;
//MPU6050 mpu1(0x69); // <-- use for AD0 high

SoftwareSerial mySerial(7, 8); // RX, TX
/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

//#define OUTPUT_READABLE_ACCEL

//#define OUTPUT_READABLE_GRAVITY

//---------------------------------------------------------------------

//#define AccelSensitivity_2G 

//#define AccelSensitivity_4G

#define AccelSensitivity_8G


//---------------------------------------------------------------------

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
unsigned long time1=0,time_old;
float delta_t,SumMagAccel;
//float delta_time;
int run1=1;

//============================
//For Normal and Faster Speed
//============================
//float AccelMagThreshold=1.5;



//============================
//For Fastest Speed;
//============================
float AccelMagThreshold=1.5;
//float SpdMagThreshold=1.6;
const int NumSamplesToSetZero=2;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container

VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements

float aaWorldX;
float aaWorldY;
float aaWorldZ;

float peak_speeds[4];// we will monitor 4 previous peak speed values

//============================

int const NumOfSamples=3;

int16_t AccelX[NumOfSamples+1], AccelY[NumOfSamples+1], AccelZ[NumOfSamples+1];

//============================

int count=0;            // initial value to count the number of samples to compute average

VectorFloat gravity;    // [x, y, z]            gravity vector, ///////////////////class VectorFloat is defined in helper_3dmath.h
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float vx,vy,vz;
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };




class AvgAccel{
  public:
  float x;
  float y;
  float z;
   float mag(){
    return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
    }
  
  };
  
int const NumOfAccelSampletoZero=3;

AvgAccel AVAWorld,AVAWorld_Zero;
float AVAWorldMagSeries[NumSamplesToSetZero];


AvgAccel aaWorld5[NumOfAccelSampletoZero];

class Speed{
  public:
  float x;
  float y;
  float z;
  float mag(){
    return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
  }
  
  };
  
Speed spd[2]; // WE NEED TWO SPDs IN ORDER TO CHECK THE RESET CONDITION

void AverageAccel(AvgAccel *AAccel) 
{
  
              if (count==0)   // 1st average
              {
                do
                {
                  mpu.dmpGetQuaternion(&q, fifoBuffer);
                  mpu.dmpGetAccel(&aa, fifoBuffer);
                  mpu.dmpGetGravity(&gravity, &q);
                  
                  #ifdef AccelSensitivity_2G
                  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                  #endif

                                    
                  #ifdef AccelSensitivity_4G
                  dmpGetLinearAccel_4G(&aaReal, &aa, &gravity);
                  #endif

                  #ifdef AccelSensitivity_8G
                  dmpGetLinearAccel_8G(&aaReal, &aa, &gravity);
                  #endif
                  
                  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
                  
                  AccelX[count]=aaWorld.x;
                  AccelY[count]=aaWorld.y;
                  AccelZ[count]=aaWorld.z;
                  count++;
                  }
                while(count<NumOfSamples);
    
                  int16_t aaWorldX_sum=0;
                  int16_t aaWorldY_sum=0;
                  int16_t aaWorldZ_sum=0;
                
                  for (int i=0;i<NumOfSamples;i++)
                  {
                    aaWorldX_sum+=AccelX[i];
                    aaWorldY_sum+=AccelY[i];
                    aaWorldZ_sum+=AccelZ[i];
                  }
                  ///=== For 2g sensitivity=====
                  #ifdef AccelSensitivity_2G
                  AAccel->x= (float) aaWorldX_sum/NumOfSamples*9.81/8192;
                  AAccel->y= (float) aaWorldY_sum/NumOfSamples*9.81/8192;
                  AAccel->z= (float) aaWorldZ_sum/NumOfSamples*9.81/8192;
                  #endif
                  
                  ///=== For 4G sensitivity=====
                  #ifdef AccelSensitivity_4G
                  AAccel->x= (float) aaWorldX_sum/NumOfSamples*9.81/4096;
                  AAccel->y= (float) aaWorldY_sum/NumOfSamples*9.81/4096;
                  AAccel->z= (float) aaWorldZ_sum/NumOfSamples*9.81/4096;
                  #endif

                  ///=== For 8G sensitivity=====
                  #ifdef AccelSensitivity_8G
                  AAccel->x= (float) aaWorldX_sum/NumOfSamples*9.81/2048;
                  AAccel->y= (float) aaWorldY_sum/NumOfSamples*9.81/2048;
                  AAccel->z= (float) aaWorldZ_sum/NumOfSamples*9.81/2048;
                  #endif
                  
                  
                }

            else
            {
//              mpu.dmpGetQuaternion(&q, fifoBuffer);
//              mpu.dmpGetAccel(&aa, fifoBuffer);
//              mpu.dmpGetGravity(&gravity, &q);
//              
//                  #ifdef AccelSensitivity_2G
//                  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//                  #endif
//
//                                    
//                  #ifdef AccelSensitivity_4G
//                  dmpGetLinearAccel_4G(&aaReal, &aa, &gravity);
//                  #endif
//                  
//                  #ifdef AccelSensitivity_8G
//                  dmpGetLinearAccel_8G(&aaReal, &aa, &gravity);
//                  #endif            
//                        
//              mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
              
              for (int i=0;i<=NumOfSamples;i++)
              {
                AccelX[i]=AccelX[i+1];
                AccelY[i]=AccelY[i+1];
                AccelZ[i]=AccelZ[i+1];
              }
              AccelX[NumOfSamples]=aaWorld.x;
              AccelY[NumOfSamples]=aaWorld.y;
              AccelZ[NumOfSamples]=aaWorld.z; 
  
              int16_t aaWorldX_sum=0;
              int16_t aaWorldY_sum=0;
              int16_t aaWorldZ_sum=0;
              
              for (int i=0;i<NumOfSamples;i++)
              {
                aaWorldX_sum+=AccelX[i];
                aaWorldY_sum+=AccelY[i];
                aaWorldZ_sum+=AccelZ[i];
              }
              ///
                  #ifdef AccelSensitivity_2G
                  AAccel->x= (float) aaWorldX_sum/NumOfSamples*9.81/8192;
                  AAccel->y= (float) aaWorldY_sum/NumOfSamples*9.81/8192;
                  AAccel->z= (float) aaWorldZ_sum/NumOfSamples*9.81/8192;
                  #endif
                  
                  /// For 4G sensitivity
                  #ifdef AccelSensitivity_4G
                  AAccel->x= (float) aaWorldX_sum/NumOfSamples*9.81/4096;
                  AAccel->y= (float) aaWorldY_sum/NumOfSamples*9.81/4096;
                  AAccel->z= (float) aaWorldZ_sum/NumOfSamples*9.81/4096;
                  #endif  
                  
                  /// For 8G sensitivity
                  #ifdef AccelSensitivity_8G
                  AAccel->x= (float) aaWorldX_sum/NumOfSamples*9.81/2048.0;
                  AAccel->y= (float) aaWorldY_sum/NumOfSamples*9.81/2048.0;
                  AAccel->z= (float) aaWorldZ_sum/NumOfSamples*9.81/2048.0;
                  #endif
                        
            
            }
  
  }

// ================================================================
// ===                     SPEED CALCULATION                    ===
// ================================================================

void speed_calc(Speed *spd,AvgAccel accel, float delta_t)
 {
  
  spd->x = spd->x + accel.x*delta_t/1000.0;
  
  spd->y = spd->y + accel.y*delta_t/1000.0;
  
  spd->z = spd->z + accel.z*delta_t/1000.0;
  
 }





// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high


void dmpDataReady() {
    mpuInterrupt = true;
}




// ================================================================
// ===               Compensate Gravity    4G Range             ===
// ================================================================

uint8_t dmpGetLinearAccel_4G(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
 // get rid of the gravity component (+1g = +4096 in standard DMP FIFO packet, Range is 4g)
    v -> x = vRaw -> x - gravity -> x*4096;
    v -> y = vRaw -> y - gravity -> y*4096;
    v -> z = vRaw -> z - gravity -> z*4096;
    return 0;
}



// ================================================================
// ===               Compensate Gravity    8G Range             ===
// ================================================================
uint8_t dmpGetLinearAccel_8G(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) 
{
 // get rid of the gravity component (+1g = +2048 in standard DMP FIFO packet, Range is 8g)
    v -> x = vRaw -> x - gravity -> x*2048;
    v -> y = vRaw -> y - gravity -> y*2048;
    v -> z = vRaw -> z - gravity -> z*2048;
    return 0;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
//            mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
//            DEBUG_PRINTLN(F("Setting accelerometer sensitivity to +/- 4..."));
//            I2Cdev::writeByte(0x68, MPU6050_RA_ACCEL_CONFIG, 0x01);

    devStatus = mpu.dmpInitialize();
    
    // config 4g sensitivity
    
    #ifdef AccelSensitivity_4G
    mpu.setFullScaleAccelRange(1);  //0 = +/- 2g | 1 = +/- 4g | 2 = +/- 8g | 3 =  +/- 16g 
    #endif

    #ifdef AccelSensitivity_8G
    mpu.setFullScaleAccelRange(2);  //0 = +/- 2g | 1 = +/- 4g | 2 = +/- 8g | 3 =  +/- 16g 
    #endif

/*
Sensor readings with offsets:  5 4 16383 1 0 0
Your offsets: -1036 -2044 851 19  -13 -23

Data is printed as: acelX acelY acelZ giroX giroY giroZ
*/

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(22);
    mpu.setYGyroOffset(-13);
    mpu.setZGyroOffset(-35);
    
    mpu.setZAccelOffset(805); //  factory default for my test chip
    //    mpu.setZAccelOffset(417); 
    //    mpu.setXAccelOffset(-1036);
    //    mpu.setYAccelOffset(-2044);
    //    make sure it worked (returns 0 if so)
    //
    //    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);

    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
      
        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else 
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
    Serial.println("I am Slave!");
    
    //  =======================================================
    // set the data rate for the SoftwareSerial port
    //  =======================================================

    mySerial.begin(9600);
    delay(1000);
}











// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here

    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //delay(200);
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
    else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

    

    
            #ifdef OUTPUT_READABLE_WORLDACCEL
            if( time1<2500)
            {
              time1=millis();
              }
            
            if (time1>2500)
            {
                // display initial world-frame acceleration, adjusted to remove gravity
                // and rotated based on known orientation from quaternion
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetAccel(&aa, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                
                #ifdef AccelSensitivity_2G
                mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                #endif
                
                #ifdef AccelSensitivity_4G
                dmpGetLinearAccel_4G(&aaReal, &aa, &gravity);
                #endif
    
                #ifdef AccelSensitivity_8G
                dmpGetLinearAccel_8G(&aaReal, &aa, &gravity);
                #endif
                
                mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
                AverageAccel(&AVAWorld);
    
                if(run1==1)
                {
                    if (AVAWorld.mag()<AccelMagThreshold)
                      {
                        AVAWorldMagSeries[NumSamplesToSetZero-1]=0;
                      }
                      else
                      {
                        AVAWorldMagSeries[NumSamplesToSetZero-1]= AVAWorld.mag(); 
                      }
                    time1=millis();
                    run1++;
                }
                else
                {
                    for(int ii=0;ii<NumSamplesToSetZero-1;ii++)
                      {
                        AVAWorldMagSeries[ii]= AVAWorldMagSeries[ii+1];
                      }
                   
                    if (AVAWorld.mag()<AccelMagThreshold)
                      {
                        AVAWorldMagSeries[NumSamplesToSetZero-1]=0;
                      }
                      else
                      {
                        AVAWorldMagSeries[NumSamplesToSetZero-1]= AVAWorld.mag(); 
                      }
                    
                    time_old=time1;
                    time1=millis();
                }         
                        delta_t=(time1-time_old);
                        //==================================================================//
                        //==============    RESET SPEED TO ZERO IF NECESSARY ===============//
                        //==================================================================//
                        SumMagAccel=0;
                        speed_calc(&spd[1],AVAWorld, delta_t);
                        for(int ii=0;ii<NumSamplesToSetZero;ii++)
                        {
                           SumMagAccel+=AVAWorldMagSeries[ii];
                          }
                          
                        if(SumMagAccel==0)
                        {
                          spd[1].x=0;
                          spd[1].y=0;
                          spd[1].z=0;                         
                        }
                        
                        //==================================================================//
                        //  Catch the peak speed value, minor bug when move at low speed
                        //==================================================================//
//
//
                        peak_speeds[0]=max(peak_speeds[0],absolute(spd[1].x)); // we have to use our own absolute function because built-in abs() returns int value
//                        
//                        spd[0].x=abs(spd[1].x);

                        //==================================================================//
                        //==============              Serial Print           ===============//
                        //==================================================================//  
                                          
//                        Serial.print(time1);
//                        Serial.print(",");
                        Serial.print(spd[1].x);
                        Serial.print(",");
                        Serial.println(peak_speeds[0]);
                        
              }
            //==================================================================//
            //==============       BLE SoftwareSerial Print      ===============//
            //==================================================================//  
//            mySerial.println(AVAWorld.x);


            }
        #endif

}

float absolute(float x)
{
  if (x>0)
    return x;
  else
    return -x;
}















  
