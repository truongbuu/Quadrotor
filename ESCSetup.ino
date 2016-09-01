/*This will sketch will enable 
  arming function and calibration of quadcopter
*/
void armESC(){
  delay(1000);
  ////////////////////////////////////////////////
  /****------DEFINE MOTOR PIN-----------****/
  motor0.attach(MOTOR_PIN_0);
  motor1.attach(MOTOR_PIN_1);
  motor2.attach(MOTOR_PIN_2);
  motor3.attach(MOTOR_PIN_3);

  delay(1000);
  /*********************************************/  
  motor0.writeMicroseconds(MAX_SIGNAL);
  //delay(500);
  motor1.writeMicroseconds(MAX_SIGNAL);
  //delay(500);
  motor2.writeMicroseconds(MAX_SIGNAL);
  //delay(500);
  motor3.writeMicroseconds(MAX_SIGNAL);
  //delay(500);
  
  /*********************************************/
  // Wait for input
  
  delay(2000);
  // Send min output
/***********************************************/
  motor0.writeMicroseconds(MIN_SIGNAL);
  //delay(500);
  motor1.writeMicroseconds(MIN_SIGNAL);
  //delay(500);
  motor2.writeMicroseconds(MIN_SIGNAL);
  //delay(500);
  motor3.writeMicroseconds(MIN_SIGNAL);
/**********************************************/
  delay(4000);
}

void writemotors(){
  
  acquireLock();
  ch3 = floor(ch3/50)*50;
  releaseLock();
  
/*Filter for channel 3*/
  if (ch3 > 2000) 
  throttle = ch3_pre;
  else {
    throttle = ch3;
    ch3_pre = ch3;    
  }

  if (ch3 > ch3_pre+300)
  { ch3 = ch3_pre;
  }
  
  /*This is a past, move on...
  v0 = throttle + (pid_roll - pid_pitch - pid_yaw);
  v1 = throttle + (pid_roll + pid_pitch + pid_yaw);
  v2 = throttle + (-pid_roll - pid_pitch + pid_yaw);
  v3 = throttle + (-pid_roll + pid_pitch - pid_yaw);
  */
  #ifdef RADIOCONTROL
  if (throttle > 1100){
  v0 = throttle + pidRateX + pidRateY + pidRateZ;
  v1 = throttle + pidRateX - pidRateY - pidRateZ;
  v2 = throttle - pidRateX + pidRateY - pidRateZ;
  v3 = throttle - pidRateX - pidRateY + pidRateZ;
  }
  else{//We want it completely shut down!
    v0 = 1000;
    v1 = 1000;
    v2 = 1000;
    v3 = 1000;
    }
  
  #endif

  #ifdef AUTONOMOUS
  /*
   * In this mode, PID for altitude is included
  */
  #ifdef JOYSTICK_CMD
  if (ch3 > 1500)
  {
    //  computePID4stabilze();

  
  getAltitudePID();//compute only when throttle is up
  Compute_PositionPID();

  RateControl();
 
  
  
  v0 = 1000+up_cmd + pidRateX + pidRateY + pidRateZ ;
  v1 = 1000+up_cmd + pidRateX - pidRateY - pidRateZ ;
  v2 = 1000+up_cmd - pidRateX + pidRateY - pidRateZ ;
  v3 = 1000+up_cmd - pidRateX - pidRateY + pidRateZ ;
  }
  else
  { 
    v0 = 1000;
    v1 = 1000;
    v2 = 1000;
    v3 = 1000;
    
    }
  #endif
  #ifdef MATLAB_CMD
  getAltitudePID();//compute only when throttle is up
  Compute_PositionPID();
  //Serial.println("Am I here?");
  RateControl();

  v0 = 1000+up_cmd + pidRateX + pidRateY + pidRateZ ;
  v1 = 1000+up_cmd + pidRateX - pidRateY - pidRateZ ;
  v2 = 1000+up_cmd - pidRateX + pidRateY - pidRateZ ;
  v3 = 1000+up_cmd - pidRateX - pidRateY + pidRateZ ;

  
  //Implement fail-safe
  if (ch3 > 1500){
    v0 = 1000;
    v1 = 1000;
    v2 = 1000;
    v3 = 1000;
  }
  
  #endif

  
  #endif

  /*
   * We dont want to make v goes crazy
  */
  if(v0 > 2000) v0 = 1900;
  if(v1 > 2000) v1 = 1900;
  if(v2 > 2000) v2 = 1900;
  if(v3 > 2000) v3 = 1900;
  
  //Serial.print(dis);
  //Serial.print(" ");
  //Serial.println(pid_dis);
  motor0.writeMicroseconds(v0);
  motor1.writeMicroseconds(v1);
  motor2.writeMicroseconds(v2);
  motor3.writeMicroseconds(v3);
/*
    8     9
       T
    10    11
*/
/*
    0     1

    2     3
*/
  }

