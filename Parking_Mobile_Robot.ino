unsigned long LastMillis = 0;

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <SPI.h>
#include <MFRC522.h>


//MPU 6050 Configuration setup

MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define OUTPUT_READABLE_YAWPITCHROLL
bool blinkState = false;


int current_status = 0;
int last_status = 0;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yprf[3];
float yprf1[3];
float target;
float current;
float error_angle;
int count = 0;
int count1 = 0;
float temp = 0;
float angle();
unsigned long previousMillis = 0;
float Error(float target);
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


//Motor pins 
  
#define IN1 2                
#define IN2 3
#define IN3 8
#define IN4 9
#define Flash 22

#define IN5 4
#define IN6 6
#define IN7 10
#define IN8 11
String inByte = "";
int ch = 0;
byte ch1 = 0;
//int leftCount = 0;
//int rightCount = 0;
//int rpmLeft = 200;
//int rpmRight = 200;
int capture=0;
int LastL = 0;
int LastR = 0;
int error = 0;

// PID Controller parameter initialisation

float Threshold = 2.5;       //2.5
float Threshold_1 = 3.5;     //3.5
//float Integral = 0;  
//long Drive = 0;
//long DriveF = 0;
//
//float kP = 1.5;
//float kI = 0.2; //0.2
//float kD = 0.1;//0.15
//float P = 0;
//float I = 0;
//float D = 0;

//int speedL = 0;
//int diff = 0;
//int Final = 0;
//int flag = 0;

int f = 0;
int b = 0;
int r = 0;
int l = 0;
int s = 0;
int tl = 0;
int tr = 0;
int bl = 0;
int br = 0;
int mf = 0;
int i = 0;
int otr = 0;

int F = 0;
int L =0;
int R =0;
int S =0;
int B = 0;
int turn_forward =0;


//Setting high speed for left and right wheels.This happens for FORWARD and REVERSE motion

int L1 = 150;
int L2 = 150;
int R1 = 150;
int R2 = 150;


//Setting low speed for left and right wheels.This happens for LEFT ,RIGHT and INCREMENTAL motion

int L11 = 90;
int L22 = 90;
int R11 = 90;
int R22 = 90;



// Setting pins for RFID pins

#define SS_PIN 53
#define RST_PIN 5


MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.

String previous_content = "";
String current_content = "";

//SoftwareSerial portOne(52,53);

void setup()
{
  Serial.begin(57600);
  Serial2.begin(115200);
  Serial3.begin(115200);

  // portOne.begin(9600);

  SPI.begin();      // Initiate  SPI bus
  mfrc522.PCD_Init();   // Initiate MFRC522
  //Serial3.println("Approximate your card to the reader...");
  //Serial3.println();


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)

  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  // Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  // Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  //    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //    while (Serial.available() && Serial.read()); // empty buffer
  //    while (!Serial.available());                 // wait for data
  //    while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-7);
  mpu.setYGyroOffset(2);
  mpu.setZGyroOffset(17);
  mpu.setZAccelOffset(1445); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 4)..."));
    attachInterrupt(4, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }


  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);


  pinMode(Flash, OUTPUT);


}


void loop()
{


// Waiting 25 seconds for the calibration to be done.DON'T GIVE ANY COMMANDS UNTIL CALIBRATION IS DONE AND FLASH IS TURNED ON

  if (count < 1)
  {
    if (millis() < 25000)
    {

    }
    else
    {
      target = angle();
      count = count + 1;
      digitalWrite(Flash, HIGH);
      delay(2000);
      digitalWrite(Flash, LOW);
    }
  } else {

    unsigned long currentMillis = millis();           //PROBLEM WITH IMU 6050,DRIFT of 1 degree/3.5 minute.SO hard coding to avoid DRIFT ERROR
    if (currentMillis - previousMillis >= 27000)
    {
  
      target = target - 0.15;                         // Updating target value in every 27 seconds...DONT WORRY ITS CALIBRATED SO THAT least DRIFT ERROR will occur
      if (target < 0)
      {
        target = 360 + target;
      }
      previousMillis = currentMillis;
    }
  }

  current = angle();
  
  error_angle = Error(target);

  if (Serial.available() > 0)                       //Input from SECONDARY controller a.k.a    Rpi 3
                                                    //USE when something to be TWEAKED with visual based localisation and navigation
                                                    //For finding the MARKER ID,show the marker to the camera with the Rpi connected to screen.
                                                    //Check Rpi 3 code for finding how each MARKER ID is tagged to INCOMING BYTE
                                                    //eg, MARKER ID : 24 may be tagged as incomingByte =1
  {

    unsigned long NowMillis = millis();

    int incomingByte = Serial.read(); // read byte
    current_status = incomingByte;
    if ((NowMillis - LastMillis >= 4000)) //||(current_status != last_status))   //If same marker is not READ twice or time elapsed after last one > 4 seconds
    {
      if (incomingByte == '1')                      //Forward
      {
        //last_status = 1;
        forward();
      //  test_stop();
      }
      if (incomingByte == '2')
      {
        if(current_status != 2)               //To avoid multiple image captures during STOP
        {
          STOP();
          test_stop();
          Serial2.println(capture);           //Sending command to Phone for image capture (TCP)
        }
        last_status = 2;

      }
      if (incomingByte == '3')
      {
        last_status = 3;
        //       Stop();
        //       STOP();
        left();  //ONCE LEFT DONE ...DONE =1 ...IF DONE =1 ==> FORWARD
        

      }
      if (incomingByte == '4')
      {
        last_status = 4;
;        //       Stop();
        //       STOP();
        right();


      }
            
      LastMillis = NowMillis;
      
    }


  }





  if (Serial2.available())            //Control from the Android tablet
  {

    ch = Serial2.read();

    if (ch == '1')           //Basic motions like forward,backward,left,right,stop 
    {
      test_forward();
    }
    if (ch == '2')
    {

      test_left();
      

    }
    if (ch == '3')
    {
      test_right();

    }
        if (ch == '4')
    {

      test_backward();
      

    }
    if (ch == '5')
    {

      test_stop();

    }
 
    if (ch == '8')
    {
      bottomleft();               //Incremental motion of 18 degree
    }
    if (ch == '9')
    {
      bottomright();  
    }
    if (ch == '0')
    {
      flash();  
    }


  }


  if (f == 1 )
  {

    analogWrite(IN1, L1);          //Basic forwad motion code   FOR VISION BASED
    analogWrite(IN2, 0);
    analogWrite(IN3, L2);
    analogWrite(IN4, 0);

    analogWrite(IN5, 0);
    analogWrite(IN6, R1);
    analogWrite(IN7, 0);
    analogWrite(IN8, R2);



    if (abs(target - current) < 180)        //CODE FOR AUTO CORRECTION IF error goes beyond threshold   FOR VISION BASED
    {
      if (current > target)
      {
        while (abs(error_angle) > Threshold )
        {
          //          Serial.println("Move left 1");

          current = angle();
          error_angle = Error(target);
          LEFT();


        }

      }
      else
      {

        while (abs(error_angle) > Threshold )
        {
          //          Serial.println("Move right 1");

          current = angle();
          error_angle = Error(target);
          RIGHT();
        }
      }
    }
    else
    {
      if (current > target)
      {

        while (abs(error_angle) > Threshold )
        {
          //          Serial.println("Move right 2");

          current = angle();
          error_angle = Error(target);
          RIGHT();

        }
      }
      else
      {

        while (abs(error_angle) > Threshold )
        {
          //          Serial.println("Move left 2");
          current = angle();
          error_angle = Error(target);
          LEFT();

        }
      }


    }





  }

else if (F == 1 )
  {



    analogWrite(IN1, L1);    //Basic code for forward motion  with ANDROID TABLET CONTROLLED mode
    analogWrite(IN2, 0);
    analogWrite(IN3, L2);
    analogWrite(IN4, 0);

    analogWrite(IN5, 0);
    analogWrite(IN6, R1);
    analogWrite(IN7, 0);
    analogWrite(IN8, R2);


 //   Serial.println("Inside f==1");

    
    //  Serial.println("PID");
    if (abs(target - current) < 180)      //Error correction with ANDROID TABLET CONTROLLED mode
    {
      if (current > target)
      {
        while (abs(error_angle) > Threshold )
        {
          //          Serial.println("Move left 1");

          current = angle();
          error_angle = Error(target);
          LEFT();


        }

      }
      else
      {

        while (abs(error_angle) > Threshold )
        {
          //          Serial.println("Move right 1");

          current = angle();
          error_angle = Error(target);
          RIGHT();
        }
      }
    }
    else
    {
      if (current > target)
      {

        while (abs(error_angle) > Threshold )
        {
          //          Serial.println("Move right 2");

          current = angle();
          error_angle = Error(target);
          RIGHT();

        }
      }
      else
      {

        while (abs(error_angle) > Threshold )
        {
          //          Serial.println("Move left 2");
          current = angle();
          error_angle = Error(target);
          LEFT();

        }
      }


    }





  }

  else if (b == 1 )
  {





    analogWrite(IN1, 0);      //Basic Backward motion FOR VISION BASED
    analogWrite(IN2, L1);
    analogWrite(IN3, 0);
    analogWrite(IN4, L2);

    analogWrite(IN5, R1);
    analogWrite(IN6, 0);
    analogWrite(IN7, R2);
    analogWrite(IN8, 0);



    if (abs(target - current) < 180)   //CODE FOR AUTO CORRECTION IF error goes beyond threshold   FOR VISION BASED
    {
      if (current > target)
      {
        while (abs(error_angle) > Threshold )
        {
          //        Serial.println("Move left 1");

          current = angle();
          error_angle = Error(target);
          LEFT();

        }

      }
      else
      {

        while (abs(error_angle) > Threshold )
        {
          //          Serial.println("Move right 1");

          current = angle();
          error_angle = Error(target);
          RIGHT();

        }
      }
    }
    else
    {
      if (current > target)
      {

        while (abs(error_angle) > Threshold )
        {
         // Serial.println("Move right 2");

          current = angle();
          error_angle = Error(target);
          RIGHT();

        }
      }
      else
      {

        while (abs(error_angle) > Threshold )
        {
          //          Serial.println("Move left 2");
          current = angle();
          error_angle = Error(target);
          LEFT();

        }
      }


    }



  }
 else if (B == 1)
{
    analogWrite(IN1, 0);   // Basic motion backward with ANDROID TABLET CONTROLLED mode
    analogWrite(IN2, L1);
    analogWrite(IN3, 0);
    analogWrite(IN4, L2);

    analogWrite(IN5, R1);
    analogWrite(IN6, 0);
    analogWrite(IN7, R2);
    analogWrite(IN8, 0);



    if (abs(target - current) < 180)     //Error correction with ANDROID TABLET CONTROLLED mode
    {
      if (current > target)
      {
        while (abs(error_angle) > Threshold )
        {
          //        Serial.println("Move left 1");

          current = angle();
          error_angle = Error(target);
          LEFT();

        }

      }
      else
      {

        while (abs(error_angle) > Threshold )
        {
          //          Serial.println("Move right 1");

          current = angle();
          error_angle = Error(target);
          RIGHT();

        }
      }
    }
    else
    {
      if (current > target)
      {

        while (abs(error_angle) > Threshold )
        {
         // Serial.println("Move right 2");

          current = angle();
          error_angle = Error(target);
          RIGHT();

        }
      }
      else
      {

        while (abs(error_angle) > Threshold )
        {
          //          Serial.println("Move left 2");
          current = angle();
          error_angle = Error(target);
          LEFT();

        }
      }


    }
  
}


  else if ( r == 1)
  {
    current = angle();
/////////////////////////////////////////////////////////////    

    if ( target > 270)               //CODE FOR turning 90 degree right  FOR VISION BASED
    {
      target = target - 270;
      error_angle = Error(target);
    }
    else if (target < 270)
    {
      target = target + 90;
      error_angle = Error(target);
    }

    Stop();

    STOP();


  }
  else if(R == 1)                //CODE FOR turning 90 degree right  FOR ANDROID TABLET BASED
  {
  current = angle();

    if ( target > 270)
    {
      target = target - 270;
      error_angle = Error(target);
    }
    else if (target < 270)
    {
      target = target + 90;
      error_angle = Error(target);
    }

    test_stop();
    STOP();  
    
    
  }
else if(L == 1)                  //CODE FOR turning 90 degree left  FOR ANDROID TABLET BASED
{
 current = angle();
//    error_angle = Error(target);

    if (target > 90)
    {
      target = target - 90;
      error_angle = Error(target);



    }

    else if (target < 90)
    {
      target = 270 + target ;
      error_angle = Error(target);


    }

    test_stop();
    STOP(); 
 }
  
  else if (l == 1)            //CODE FOR turning 90 degree left  FOR VISION BASED
  {

    current = angle();
//    error_angle = Error(target);

    if (target > 90)
    {
      target = target - 90;
      error_angle = Error(target);



    }

    else if (target < 90)
    {
      target = 270 + target;
      error_angle = Error(target);


    }

    Stop();
    STOP();


  }

  
  else if (mf == 1)
  {

    STOP();

  }
  else if (s == 1)                         //CODE FOR STOP and AUTOCORRECTION FOR VISION BASED
  {


    STOP();
    if (abs(target - current) < 180)
    {
      if (current > target)
      {
        while (abs(error_angle) > Threshold_1 )
        {
          //          Serial.println("Move left 1");

          current = angle();
          error_angle = Error(target);
          LEFT();

        }

      }
      else
      {

        while (abs(error_angle) > Threshold_1 )
        {
          //          Serial.println("Move right 1");

          current = angle();
          error_angle = Error(target);
          RIGHT();

        }
      }
    }
    else
    {
      if (current > target)
      {

        while (abs(error_angle) > Threshold_1 )
        {
          //          Serial.println("Move right 2");

          current = angle();
          error_angle = Error(target);
          RIGHT();

        }
      }
      else
      {

        while (abs(error_angle) > Threshold_1 )
        {
          //          Serial.println("Move left 2");
          current = angle();
          error_angle = Error(target);
          LEFT();
        }
      }


    }



    
    STOP();
    
   //////////////////////////////////////////////////////////// 
    
    if(abs(target - current ) < 180)
    {
      if(abs(target - current)< Threshold_1)
      {
       STOP(); 
       forward();
      }
      else
      {
        
      STOP();
      }  
    }
    else
    {
      if((360 - abs(target - current)) < Threshold_1)
      {
      STOP();  
      forward();   
      }  
      else
      {
      STOP();
      }        
    }
   ////////////////////////////////////////////////////////////// 

  }
else if (S == 1)                      //CODE FOR STOP and autocorrection FOR TABLET BASED
  {
//  analogWrite(IN1, 0);
//  analogWrite(IN2, 0);
//  analogWrite(IN3, 0);
//  analogWrite(IN4, 0);
//
//  analogWrite(IN5, 0);
//  analogWrite(IN6, 0);
//  analogWrite(IN7, 0);
//  analogWrite(IN8, 0);

 // Serial.println("s ==1");
    if (abs(target - current) < 180)
    {
      if (current > target)
      {
        while (abs(error_angle) > Threshold_1 )
        {
          //          Serial.println("Move left 1");

          current = angle();
          error_angle = Error(target);
          LEFT();

        }

      }
      else
      {

        while (abs(error_angle) > Threshold_1 )
        {
          //          Serial.println("Move right 1");

          current = angle();
          error_angle = Error(target);
          RIGHT();

        }
      }
    }
    else
    {
      if (current > target)
      {

        while (abs(error_angle) > Threshold_1 )
        {
          //          Serial.println("Move right 2");

          current = angle();
          error_angle = Error(target);
          RIGHT();

        }
      }
      else
      {

        while (abs(error_angle) > Threshold_1 )
        {
          //          Serial.println("Move left 2");
          current = angle();
          error_angle = Error(target);
          LEFT();
        }
      }


    }
    //
    //
    //    digitalWrite(IN1, LOW);
    //    digitalWrite(IN2, LOW);
    //    digitalWrite(IN3, LOW);
    //    digitalWrite(IN4, LOW);

    STOP();


  }


  
//  else if (tl == 1)
//  {
//    current = angle();
//    error_angle = Error(target);
//
//    if (current > 15)
//    {
//      target = current - 15;
//      error_angle = Error(target);
//
//
//
//    }
//
//    else if (current < 15)
//    {
//      target = 360 - 15 + current;
//      error_angle = Error(target);
//
//
//    }
//
//  //  Stop();
//    test_stop();
//    STOP();
//
//  }
//  else if ( tr == 1)
//  {
//
//
//    current = angle();
//    error_angle = Error(target);
//    if ( current > 345)
//    {
//      target = current - 345;
//      error_angle = Error(target);
//    }
//    else if (current < 345)
//    {
//      target = current + 15;
//      error_angle = Error(target);
//    }
//
//    test_stop();
//    STOP();
//
//
//
//  }




  else if (bl == 1)        //Incremental shift  (backward-left)/(forwar-right) direction with 15 degree
  {                        //CODE FOR AUTO CORRECTION INCLUDED 
    current = angle();
    error_angle = Error(target);
    if ( current > 345)           
    {
      target = current - 345;
      error_angle = Error(target);
    }
    else if (current < 345)
    {
      target = current + 15;
      error_angle = Error(target);
    }
    test_stop();
    STOP();

  }
  else if (br == 1)      //Incremental shift  (backward-left)/(forwar-right) direction with 15 degree
  {                      //CODE FOR AUTO CORRECTION INCLUDED 
    current = angle();
    error_angle = Error(target);

    if (current > 15)
    {
      target = current - 15;
      error_angle = Error(target);



    }

    else if (current < 15)
    {
      target = 360 - 15 + current;
      error_angle = Error(target);


    }
    test_stop();
    STOP();
  }


  ////////////////////////////////////////////////////////////////////





  /////////////////////////////////////////////////////////////////////

  //Card_read();

  ////////////////////////////////////////////////////////////////////

//

}

















void forward()        //FORWARD _VISION BASED
{

  f = 1;
  b = 0;
  r = 0;
  l = 0;
  s = 0;
  tl = 0;
  tr = 0;
  bl = 0;
  br = 0;
 
  F =0;
  S = 0;  
  B = 0;
  R = 0;
  L = 0;
  
 // Serial.println("Inside forward()");

}

void test_forward()   //FORWARD _ANDROID TABLET BASED
{

  f = 0;
  b = 0;
  r = 0;
  l = 0;
  s = 0;
  tl = 0;
  tr = 0;
  bl = 0;
  br = 0;
 
  F =1;
  S = 0;  
  B = 0;
  R = 0;
  L = 0;
 // Serial.println("Inside forward()");

}


void test_stop()    //STOP _ANDROID TABLET BASED
{
  f = 0;
  b = 0;
  r = 0;
  l = 0;
  s = 0;
  tl = 0;
  tr = 0;
  bl = 0;
  br = 0;
  mf = 0;
 
  F =0;
  S = 1;  
  B = 0;
  R = 0;
  L = 0;
}

void test_backward() //BACKWARD _ANDROID TABLET BASED
{
  
  f = 0;
  b = 0;
  r = 0;
  l = 0;
  s = 0;
  tl = 0;
  tr = 0;
  bl = 0;
  br = 0;
  mf = 0;
  
  F =0;
  S = 0;  
  B = 1;
  R = 0;
  L = 0;
  
}


void test_right()  //90 degree RIGHT _ANDROID TABLET BASED
{
  
  f = 0;
  b = 0;
  r = 0;
  l = 0;
  s = 0;
  tl = 0;
  tr = 0;
  bl = 0;
  br = 0;
  mf = 0;
  
  F =0;
  S = 0;  
  B = 0;
  R = 1;
  L = 0;
}
void test_left()   //90 degree LEFT _ANDROID TABLET BASED
{
  
  f = 0;
  b = 0;
  r = 0;
  l = 0;
  s = 0;
  tl = 0;
  tr = 0;
  bl = 0;
  br = 0;
  mf = 0;
  
  F =0;
  S = 0;  
  B = 0;
  R = 0;
  L = 1;
  
}




void backward()   //BACKWARD _ANDROID TABLET BASED
{



  f = 0;
  b = 1;
  r = 0;
  l = 0;
  s = 0;
  tl = 0;
  tr = 0;
  bl = 0;
  br = 0;
  mf = 0;
  
  F =0;
  S = 0;  
  B = 0;
  R = 0;
  L = 0;

}
void left()  // 90 degree LEFT VISION BASED
{

  analogWrite(IN1, 0);
  analogWrite(IN2, L1);
  analogWrite(IN3, 0);
  analogWrite(IN4, L2);

  analogWrite(IN5, 0);
  analogWrite(IN6, R1);
  analogWrite(IN7, 0);
  analogWrite(IN8, R2);

  f = 0;
  b = 0;
  r = 0;
  l = 1;
  s = 0;
  tl = 0;
  tr = 0;
  bl = 0;
  br = 0;
  mf = 0;
  otr = 1;
   
  F =0;
  S = 0;  
  B = 0;
  R = 0;
  L = 0;
}
void LEFT()         // 90 degree LEFT ANDROID TABLET BASED
{
  analogWrite(IN1, 0);
  analogWrite(IN2, L11);
  analogWrite(IN3, 0);
  analogWrite(IN4, L22);

  analogWrite(IN5, 0);
  analogWrite(IN6, R11);
  analogWrite(IN7, 0);
  analogWrite(IN8, R22);


}

void RIGHT()       // 90 degree RIGHT ANDROID TABLET BASED
{
  analogWrite(IN1, L11);
  analogWrite(IN2, 0);
  analogWrite(IN3, L22);
  analogWrite(IN4, 0);

  analogWrite(IN5, R11);
  analogWrite(IN6, 0);
  analogWrite(IN7, R22);
  analogWrite(IN8, 0);


}

void STOP()        // STOP ANDROID TABLET BASED
{
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);

  analogWrite(IN5, 0);
  analogWrite(IN6, 0);
  analogWrite(IN7, 0);
  analogWrite(IN8, 0);
}





void right()   // 90 degree RIGHT VISION BASED
{



  analogWrite(IN1, L1);
  analogWrite(IN2, 0);
  analogWrite(IN3, L2);
  analogWrite(IN4, 0);

  analogWrite(IN5, R1);
  analogWrite(IN6, 0);
  analogWrite(IN7, R2);
  analogWrite(IN8, 0);

  f = 0;
  b = 0;
  r = 1;
  l = 0;
  s = 0;
  tl = 0;
  tr = 0;
  bl = 0;
  br = 0;
  mf = 0;
  otr = 1;
   
  F =0;
  S = 0;  
  B = 0;
  R = 0;
  L = 0;
}
void Stop()
{



  f = 0;
  b = 0;
  r = 0;
  l = 0;
  s = 1;
  tl = 0;
  tr = 0;
  bl = 0;
  br = 0;
  mf = 0;
   
  F =0;
  S = 0;  
  B = 0;
  R = 0;
  L = 0;
}

void topleft()          //Incremental 15 degree turns
{
  for (i = 0; i < 1000 ; i++ )
  {


    analogWrite(IN1, L1);
    analogWrite(IN2, 0);
    analogWrite(IN3, L2);
    analogWrite(IN4, 0);

    analogWrite(IN5, 0);
    analogWrite(IN6, R1);
    analogWrite(IN7, 0);
    analogWrite(IN8, R2);



  }


  analogWrite(IN1, 0);
  analogWrite(IN2, L1);
  analogWrite(IN3, 0);
  analogWrite(IN4, L2);

  analogWrite(IN5, 0);
  analogWrite(IN6, R1);
  analogWrite(IN7, 0);
  analogWrite(IN8, R2);

  f = 0;
  b = 0;
  r = 0;
  l = 0;
  s = 0;
  tl = 1;
  tr = 0;
  bl = 0;
  br = 0;
  mf = 0;
  otr = 1;
  
  F =0;
  S = 0;  
  B = 0;
  R = 0;
  L = 0;
}

void topright()      //Incremental 15 degree turns
{

  //  digitalWrite(IN1, HIGH);
  //  digitalWrite(IN2, LOW);
  //  digitalWrite(IN3, HIGH);
  //  digitalWrite(IN4, LOW);

  analogWrite(IN1, L1);
  analogWrite(IN2, 0);
  analogWrite(IN3, L2);
  analogWrite(IN4, 0);

  analogWrite(IN5, R1);
  analogWrite(IN6, 0);
  analogWrite(IN7, R2);
  analogWrite(IN8, 0);

  f = 0;
  b = 0;
  r = 0;
  l = 0;
  s = 0;
  tl = 0;
  tr = 1;
  bl = 0;
  br = 0;
  mf = 0;
  otr = 1;
   
  F =0;
  S = 0;  
  B = 0;
  R = 0;
  L = 0;
}

void bottomleft()
{

  //  digitalWrite(IN1, HIGH);
  //  digitalWrite(IN2, LOW);
  //  digitalWrite(IN3, HIGH);
  //  digitalWrite(IN4, LOW);
  analogWrite(IN1, L1);
  analogWrite(IN2, 0);
  analogWrite(IN3, L2);
  analogWrite(IN4, 0);

  analogWrite(IN5, R1);
  analogWrite(IN6, 0);
  analogWrite(IN7, R2);
  analogWrite(IN8, 0);

  f = 0;
  b = 0;
  r = 0;
  l = 0;
  s = 0;
  tl = 0;
  tr = 0;
  bl = 1;
  br = 0;
  mf = 0;
  otr = 1;
   
  F =0;
  S = 0;  
  B = 0;
  R = 0;
  L = 0;
}

void bottomright()
{

  //  digitalWrite(IN1, LOW);
  //  digitalWrite(IN2, HIGH);
  //  digitalWrite(IN3, LOW);
  //  digitalWrite(IN4, HIGH);
  analogWrite(IN1, 0);
  analogWrite(IN2, L1);
  analogWrite(IN3, 0);
  analogWrite(IN4, L2);

  analogWrite(IN5, 0);
  analogWrite(IN6, R1);
  analogWrite(IN7, 0);
  analogWrite(IN8, R2);


  f = 0;
  b = 0;
  r = 0;
  l = 0;
  s = 0;
  tl = 0;
  tr = 0;
  bl = 0;
  br = 1;
  mf = 0;
  otr = 1;
   
  F =0;
  S = 0;  
  B = 0;
  R = 0;
  L = 0;
}

void motoroff()
{
  f = 0;
  b = 0;
  r = 0;
  l = 0;
  s = 0;
  tl = 0;
  tr = 0;
  bl = 0;
  br = 0;
  mf = 1;
  otr = 1;
     
  F =0;
  S = 0;  
  B = 0;
  R = 0;
  L = 0;
  //  digitalWrite(IN1, LOW);
  //  digitalWrite(IN2, LOW);
  //  digitalWrite(IN3, LOW);
  //  digitalWrite(IN4, LOW);

  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);

  analogWrite(IN5, 0);
  analogWrite(IN6, 0);
  analogWrite(IN7, 0);
  analogWrite(IN8, 0);

}



float angle()         //Getting live angles from IMU 6050
{

  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
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
   // Serial.println(("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;



    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);





    yprf[0] = (ypr[0] * 180 / M_PI);
    yprf[1] = (ypr[1] * 180 / M_PI);
    yprf[2] = (ypr[2] * 180 / M_PI);

    if (yprf[0] > 0 )
    {
      temp = yprf[0];

    }
    else
    {
      temp = 360 + yprf[0];

    }

    return temp;

  }


}


float Error(float target)   // Calculating error angle at every instant
{
  if (abs(target - current) < 180)
  {
    if (current > target)
    {
      error_angle = target - current;
    }
    else
    {
      error_angle = target - current;
    }
  }
  else
  {
    if (current > target)
    {
      error_angle = 360 + target - current;
    }
    else
    {
      error_angle = 360 - target + current;
    }
  }


  return error_angle;
}




void flash()          //Turning on flash for 2 seconds
{
      digitalWrite(Flash, HIGH);
      delay(2000);
      digitalWrite(Flash, LOW);  
}

