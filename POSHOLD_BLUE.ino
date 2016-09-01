#include "Config.h"
#include <SPI.h>
#include "ADNS3080.h"
/*
 * The library goes here:
*/


#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
//#include <Timer.h>

//#include <NewPing.h>
//
//Choose two modes belows
/*Uncomment one or two lines below
 *AUTONOMOUS MODE: TAKE OFF AND ALTITUDE CONTROL AUTOMATICALLY
 *RADIOCONTROL: NEED RADIO COMMAND FOR THROTTLE
*/
#define AUTONOMOUS
//#define RADIOCONTROL

//#define JOYSTICK_CMD
#define MATLAB_CMD


#define COMMUNICATION_ENABLE

/*
Debug function
*/
//#define DEBUG //DEBUG MODE
//#define DEBUG_MOTOR_SPEED
//#define DEBUG_SENSOR_DATA
//#define DEBUG_LOOP_TIME
//#define DEBUG_RC_DATA
//#define PID_VALUE
//#define RATE_MODE
//#define ALT_DEBUG
//#define ACCEL_DEBUG
//#define VELO_DEBUG
//#define POS_DEBUG
/*ESC and Motors
*/

Servo motor0 ;
Servo motor1 ;
Servo motor2 ;
Servo motor3 ;

float v0, v1, v2, v3;

/*
Sonar variables
*/
//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_dis); // NewPing setup of pins and maximum dis.
float p,r;
float dis;
float dis_pre;
int LoopStart; //Timing for getting dis, sonar need at least 30ms delay for getting high accurate data
int FinTime;
int ITime = 0;

unsigned long LoopZ;
unsigned long IZ;
unsigned long EndZ;
int sensor_Z;
int virgin_z= 1; 

float up_cmd = 0;
float pid_dis;
float dis_sp = 45; 
PID disReg(&dis, &pid_dis, &dis_sp, DIS_P, DIS_I, DIS_D, DIRECT );



/*MPU variables
*/
int ax, ay, az, gx, gy, gz;

VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements

float rateX, rateY, rateZ;
float accX, accY, accZ;
float readingX, readingY, readingZ;
float veloX, veloY, veloZ;


float acX[4] = {0,0,0,0};
float acY[4] = {0,0,0,0};
float acZ[4] = {0,0,0,0}; //Filtering memory

float yaw, pitch, roll;
float roll_off = 0;
float pitch_off = 0;
MPU6050 mpu;                           // mpu interface object
uint8_t mpuIntStatus;                  // mpu statusbyte
uint8_t devStatus;                     // device status
uint16_t packetSize;                   // estimated packet size
uint16_t fifoCount;                    // fifo buffer size
uint8_t fifoBuffer[64];                // fifo buffer
Quaternion q;                          // quaternion for mpu output
VectorFloat gravity;                   // gravity vector for ypr
float ypr[3] = {0.0f, 0.0f, 0.0f};     // yaw pitch roll values
float yprLast[3] = {0.0f, 0.0f, 0.0f};
volatile bool mpuInterrupt = false;    //interrupt flag
boolean interruptLock = false;
float yaw_init = 0;
////////////////////////////////////////////////////////////////

/*PID variables
*/
float rollsp = 0.0;
float pitchsp = 0.0;
float yawsp = 0;

float pidRateX, pidRateY, pidRateZ;
float rateXsp = 0;
float rateYsp = 0;
float rateZsp = 0;

PID rateXReg(&rateX, &pidRateX, &rateXsp, RATEX_P, RATEX_I, RATEX_D, DIRECT );
PID rateYReg(&rateY, &pidRateY, &rateYsp, RATEY_P, RATEY_I, RATEY_D, DIRECT );
PID rateZReg(&rateZ, &pidRateZ, &rateZsp, RATEZ_P, RATEZ_I, RATEZ_D, DIRECT );

float pid_roll, pid_pitch, pid_yaw; //angle PID
float diff_roll,diff_pitch;
PID pitchReg(&pitch, &pid_pitch, &pitchsp, PITCH_P, PITCH_I, PITCH_D, DIRECT );
PID rollReg(&roll, &pid_roll, &rollsp, ROLL_P, ROLL_I, ROLL_D, DIRECT);
PID yawReg(&yaw, &pid_yaw, &yawsp, YAW_P, YAW_I, YAW_D, DIRECT);


/*
Radio Control variables
*/



float ch1, ch2, ch3, ch4, ch5;         // RC channel inputs
float ch3_pre;

unsigned long rcLastChange1 = micros();
unsigned long rcLastChange2 = micros();
unsigned long rcLastChange3 = micros();
unsigned long rcLastChange4 = micros();
unsigned long rcLastChange5 = micros();
float throttle;
///////////////////-----------------------------------------------------------------------------------------------////////////////////


/*
Timer variable
*/
Servo Trigpin;



/*Optical Mouse sensor variable
 * goes here
*/
float field_of_view = AP_OPTICALFLOW_ADNS3080_08_FOV;
float conv_factor = ((1.0f / (float)(30 * 1.25))
                   * 2.0f * tan(field_of_view / 2.0f));// 0.0379;
float radians_to_pixels = (30 * 1.35) / field_of_view;


boolean powerDown = false;                     // flag to check if need to power down because went into image send mode

byte  orig_spi_settings_spcr;                  // bytes to store SPI settings  
byte  orig_spi_settings_spsr;                  
                                               // Pertinent Data returned by the Camera
//int x_pos, y_pos;                              // dx, dy values from camera accumulated in these variables
//int dx_pos, dy_pos;                            // values returns by camera  dx, dy 
char  dx,dy;  
int  raw_dx;            // raw sensor change in x and y position (i.e. unrotated)
int  raw_dy; 
byte surface_quality;                          // surface quality - not really used
byte _overflow;                                 // overflow of dx,dy values - not really used
boolean _motion;                                // detects if dx, dy values changed since last reading
float    exp_change_x, exp_change_y;    // expected change in x, y coordinates
float    change_x, change_y;            // actual change in x, y coordinates
float    x_cm, y_cm;                    // x,y position in cm
float velo_x_pre, velo_y_pre;
float roll_pre, pitch_pre;
uint8_t motion_reg;
byte stat;
int signalupdate;
float pre_roll,pre_pitch;
float velo_x, velo_y;
float pidX, pidY;
float xsp = 0;
float ysp = 0;

float flow_x,flow_y;
float x_rate,y_rate;
float veloXsp = 0;
float veloYsp = 0;
float pidVX,pidVY;

PID VXReg(&velo_x, &pidVX, &veloXsp, VELO_X_P, VELO_X_I, VELO_X_D, DIRECT);
PID VYReg(&velo_y, &pidVY, &veloYsp, VELO_Y_P, VELO_Y_I, VELO_Y_D, DIRECT);


int count_mode =0;
PID XReg(&x_cm, &pidX, &xsp, X_P, X_I, X_D, DIRECT);
PID YReg(&y_cm, &pidY, &ysp, Y_P, Y_I, Y_D, DIRECT);

float move_x,move_y;
float x_moved, y_moved;
float pid_xm, pid_ym;
float set_x = 0;
float set_y = 0;
PID XM(&x_moved, &pid_xm, &set_x, XY_P, XY_I, XY_D, DIRECT);
PID YM(&y_moved, &pid_ym, &set_y, XY_P, XY_I, XY_D, DIRECT);

//////////////////////////////////
#include "MyPX4flow.h"

#define PARAM_FOCAL_LENGTH_MM 16

// Pin 13 has an LED connected on most Arduino boards.
// Pin 11 has the LED on Teensy 2.0
// Pin 6  has the LED on Teensy++ 2.0
// Pin 13 has the LED on Teensy 3.0
// give it a name:
#define LED 13

long last_check = 0;
int px = 0;
int py = 0;
float focal_length_px = (PARAM_FOCAL_LENGTH_MM) / (4.0f * 6.0f) * 1000.0f;
  
// Initialize PX4Flow library
PX4Flow sensor = PX4Flow(); 
///////////////////////////////////////
union NumericIntType
{
  int intValue;
  unsigned int uintValue;
  byte byteValue[2];
};

//////////////////////////////////////
////End of everything////////////////

void setup() {
  Init();
}

void loop() {
  // put your main code here, to run repeatedly:
  /*Autonomous mode: provide some function for updating altitude and Optical flow sensor*/

// unsigned int a = millis();
  readData();//roll pitch yaw and distance
  getRate(); 
/*
  getAltitudePID();//compute only when throttle is up
  Compute_PositionPID();

  RateControl();
*/
  writemotors();
  
  EndData();
 // Serial.println(rollsp);
//  unsigned int b = millis();
//  Serial.println(b-a);
/*This is for communication
 * between Matlab and Quadcopter
*/
#ifdef COMMUNICATION_ENABLE
  /*Send data packet
   * This should be easy
  *///sendPacket is dangerous!!, delay up to 20ms
  //sendPacket();
  /*Check signal
  */
  StopNow();
#endif

/*THIS IS FOR DEBUG
*/  
#ifdef DEBUG
  Debugging();
#endif
}

void Debugging() {
#ifdef DEBUG_MOTOR_SPEED
//  Serial.print(v0);
//  Serial.print(" ");
//  Serial.println(v1);
//  Serial.print(" ");
//  Serial.println(v2);
  Serial.print(" ");
  Serial.println(v3);

#endif

#ifdef DEBUG_SENSOR_DATA
//  Serial.print("ROLL: ");
 // Serial.print(roll);
 // Serial.print(" ");
  Serial.println(yaw);
  //Serial.println(pitch);
  //Serial.print("YAW: ");
//  Serial.println(yaw);
  //Serial.print("dis: ");
  //Serial.println(dis);
#endif


#ifdef DEBUG_LOOP_TIME
  Serial.print("LOOP TIME: ");
  Serial.println(ITime);
#endif

#ifdef DEBUG_RC_DATA
  Serial.print("CHANNEL 3:");
  Serial.println(ch3);
#endif

#ifdef PID_VALUE
  Serial.print("PID ROLL: ");
  Serial.print(pid_roll);
  Serial.print("PID PITCH: ");
  Serial.print(pid_pitch);
  Serial.print("PID YAW: ");
  Serial.print(pid_yaw);
  Serial.println("PID dis: ");
  //Serial.println(pid_dis);
#endif

#ifdef RATE_MODE
  //Serial.print("RATE X:");
  //Serial.print(pidRateX);

  //Serial.print("RATE Y:");
  //Serial.println(pidRateY);
  Serial.println(rateX);

#endif

#ifdef ALT_DEBUG
  Serial.print("a ");
//  Serial.print(pid_dis);
//  Serial.print(" ");
  Serial.print(r);
  Serial.print(" ");
  Serial.print(p);
  Serial.print(" ");
  Serial.print(dis);
  Serial.print(" ");
  Serial.println(abs(1/(cos(r)*cos(p))));
#endif

#ifdef ACCEL_DEBUG
  Serial.print("b ");
  //Serial.println(accZ);
  Serial.println(accX);
  
  
#endif

#ifdef VELO_DEBUG
  Serial.print("a ");
  Serial.println(veloX);
#endif

#ifdef POS_DEBUG

//  Serial.print("dx : ");
//  Serial.print(raw_dx);
//  Serial.print("p:");
//  Serial.println(exp_change_x);
//  Serial.print("dy : ");
//  Serial.println(raw_dy);
//  Serial.print(" exp: ");
//  Serial.print(exp_change_y);
//Serial.println(change_y);
//  Serial.print("diff :");
//  Serial.println(diff_roll);
//Serial.print("y ");
//Serial.print(y_cm,6);
//Serial.print("y ");
//Serial.println(y_cm);
//Serial.print(" pitch : ");
//Serial.println(pitch);

//Serial.print("rollsp: ");
//Serial.println(rollsp,6);
//Serial.print("pitchsp: ");
//Serial.println(pitchsp,6);
//Serial.print("a ");
//Serial.print(pidX);
//Serial.print("b ");
//int UP = (int)(up_cmd);
Serial.print(flow_x);
Serial.print(" ");
Serial.println(x_rate);
//Serial.print(" ");
//Serial.println(velo_x,6);
//Serial.print(" ");
//Serial.print(dis);
//Serial.print(" ");
//Serial.println(ch5);
#endif

}
