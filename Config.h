/*
All the configurations goes here
This includes define of constant
PTB 17/12/2015
*/

//PID parameters
#define RATEX_P 7.55//Good gain 5.5
#define RATEX_I 0.05f//0.01
#define RATEX_D 0.45f //0.25

#define RATEY_P 7.55f//5.1 
#define RATEY_I 0.05f 
#define RATEY_D 0.45f //0.25

#define RATEZ_P 4.0f
#define RATEZ_I 0.01f
#define RATEZ_D 0.0f

#define PITCH_P  3.25f//2.5 
#define PITCH_I 0.03f 
#define PITCH_D 0.02f//0.008 
#define PITCH_MAX 100

#define ROLL_P 3.25f//2.5 
#define ROLL_I 0.03f//0.015 
#define ROLL_D 0.02f 
#define ROLL_MAX 100

#define YAW_P 3.2 //4.3f
#define YAW_I 0.3 //0.9f
#define YAW_D 0.2 //0.6f
#define YAW_MAX 100

//For position
/*
#define X_P 0.95//2.35 
#define X_I 0.4
#define X_D 0.08//0.04

#define Y_P 0.95
#define Y_I 0.4
#define Y_D 0.08
*/
/*
Baseline velo: 0.1  0.02 0.01
               0.17 0.02 0.012
*/
#define X_P  8.40f//6.10//8.4
#define X_I  0.02f//0.02//0.02
#define X_D  0.02f//0.05//0.00

#define Y_P  8.40//6.10f//1.5
#define Y_I  0.02//0.02f//0.001
#define Y_D  0.02//0.05f//4.00


#define XY_P 0.90f
#define XY_I 0.001f
#define XY_D 0.001f



#define VELO_X_P 0.0//1.15//OK
#define VELO_X_I 0.0//0.5
#define VELO_X_D 0.0//0.05//0.0

#define VELO_Y_P 0.0//1.15
#define VELO_Y_I 0.0//0.5
#define VELO_Y_D 0.0//0.05


#define DIS_P 2.5f //2.5
#define DIS_I 2.0f //2//1
#define DIS_D 1.2f //1.2



//ESC parameters

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000

#define MOTOR_PIN_0 2//8
#define MOTOR_PIN_1 3//9
#define MOTOR_PIN_2 5//10
#define MOTOR_PIN_3 6//11
/*
    8     9 --------2   3
       T    --------  T 
    10    11--------5   6
*/


//Sonar parameters

#define TRIG_Z  9  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_Z     12 // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.


//RC paramters

#define RC_3 11







