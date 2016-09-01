/*
This part will provide the info about yaw pitch roll rotation rate and acceleration
*/
void getRate(){
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  rateX = float(gx)/131;
  rateY = float(gy)/131;
  rateZ = float(gz)/131;
/*
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  
  readingX = float(aaReal.x)/16384;
  readingY = float(aaReal.y)/16384;
  readingZ = float(aaReal.z)/16384;
  accel_Filter();
*/  }

  
void accel_Filter(){
//
//X:
  acX[3] = acX[2]; acX[2] = acX[1]; acX[1] = acX[0]; acX[0] = readingX; 
  acY[3] = acY[2]; acY[2] = acY[1]; acY[1] = acY[0]; acY[0] = readingY;
  acZ[3] = acZ[2]; acZ[2] = acZ[1]; acZ[1] = acZ[0]; acZ[0] = readingZ;
//  
 accX = 0.125*(acX[0]+3*acX[1] +3*acX[2] + acX[3]) + 0.01;
 accY = 0.125*(acY[0]+3*acY[1] +3*acY[2] + acY[3]) - 0.01;
 accZ = 0.125*(acZ[0]+3*acZ[1] +3*acZ[2] + acZ[3]) - 0.01;
 accX = accX*9.8 - 0.1;
 accY = accY*9.8;
 accZ = accZ*9.8;
  }

void getVelocity(){
  
  veloX += accX*0.02;
  veloY += accY*0.02;
  }
void initYaw(){
  int count;
  for (count = 0; count < 3000; count ++){
    getYPR();
    yaw = ypr[0]*180/3.1415;
    pitch = ypr[1]*180/3.1415;
    roll = ypr[2]*180/3.1415;

    if (abs(yaw - yprLast[0]) > 25) yaw = yprLast[0] ;
    if (abs(pitch - yprLast[1]) > 25) pitch = yprLast[1];
    if (abs(roll - yprLast[2]) > 25) roll = yprLast[2];

    }
    yaw_init = yaw;
    yprLast[0] = yaw-yaw_init;
  }

void readData(){
  getYPR();
  sensor.init_up_integral();
  //unsigned int s = micros();
  startZ();
  
  pitch = ypr[1]*180/3.1415;
  roll = ypr[2]*180/3.1415;
  if (abs(pitch - yprLast[1]) > 10) pitch = yprLast[1];
  if (abs(roll - yprLast[2]) > 10) roll = yprLast[2];
  
  
  //Motion burst in optical flow

  
  yaw = ypr[0]*180/3.1415;
  
  //
  yaw -= yaw_init;// set yaw back to initial 
  /*Serial.print("HI HI ");
  Serial.print(yaw);
  Serial.print(" ");
  Serial.print(yprLast[0]);
  */
  //yaw = yaw + 0.0000275753767;
  if (abs(yaw - yprLast[0]) > 10) yaw = yprLast[0] ;
  //Serial.print("... ");
  //Serial.print(yprLast[0]);
  //Serial.print(" ");
  //Serial.println(yaw);
 
  yprLast[0] = yaw;
  p = pitch*3.1415/180;
  r = roll*3.1415/180;
  //distance = distance + 0.9;
  
  readZ();
  endZ();
  //getAltitudePID();
  //unsigned int e = micros();
  //Serial.println(e-s);
  
  readFlow();
 /*
  
  yprLast[0] = yaw;
  yprLast[1] = pitch;
  yprLast[2] = roll;
  yaw -= yaw_init;// set yaw back to initial 
  roll = roll - roll_off;
  pitch = pitch - pitch_off;*/
  //distance = sonar.ping_cm();
  //Serial.println(distance);
}
void EndData(){
  
  yprLast[1] = pitch;
  yprLast[2] = roll;
  }
void readyData(){
  getYPR();
  yaw = ypr[0]*180/3.1415;
  pitch = ypr[1]*180/3.1415;
  roll = ypr[2]*180/3.1415;
  //if (abs(yaw - yprLast[0]) > 25) yaw = yprLast[0] ;
  //if (abs(pitch - yprLast[1]) > 25) pitch = yprLast[1];
  //if (abs(roll - yprLast[2]) > 25) roll = yprLast[2];
  //
  
  
  yprLast[0] = yaw;
  yprLast[1] = pitch;
  yprLast[2] = roll;
  yaw -= yaw_init;// set yaw back to initial 
  roll = roll - roll_off;
  pitch = pitch - pitch_off;
}

void getYPR(){
  /*Now this is going to be a mess!, beware*/
  
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
  
    fifoCount = mpu.getFIFOCount();
    
    if((mpuIntStatus & 0x10) || fifoCount >= 1024){ 
      
      mpu.resetFIFO(); 
    
    }else if(mpuIntStatus & 0x02){
    
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      fifoCount -= packetSize;
    
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    }

}


inline void dmpDataReady() {
    mpuInterrupt = true;
}

void initMPU(){
  
  Wire.begin();
  TWBR = 24;//Test of changes
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if(devStatus == 0){
  
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
    /*Calibration */
  mpu.setXGyroOffset(63);
  mpu.setYGyroOffset(-6);
  mpu.setZGyroOffset(33);
  mpu.setXAccelOffset(-2157);
  mpu.setYAccelOffset(1550);
  mpu.setZAccelOffset(1463);
  }
}

