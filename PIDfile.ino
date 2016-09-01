void Init_PID_Velo(){
  VXReg.SetMode(AUTOMATIC);
  VYReg.SetMode(AUTOMATIC);

  VXReg.SetSampleTime(50);
  VYReg.SetSampleTime(50);

  VXReg.SetOutputLimits(-10,10);
  VYReg.SetOutputLimits(-10,10);
  }
void Init_XY(){
  XM.SetMode(AUTOMATIC);
  YM.SetMode(AUTOMATIC);

  XM.SetSampleTime(5);
  YM.SetSampleTime(5);

  XM.SetOutputLimits(-5,5);
  YM.SetOutputLimits(-5,5);
  }
  
void Com_XY(){
  XM.Compute();
  YM.Compute();
  }  
void Compute_Velo_PID(){
  veloXsp = pidX;
  veloYsp = pidY;
  VXReg.Compute();
  VYReg.Compute();
  }
  
void Init_PID_Position(){
  XReg.SetMode(AUTOMATIC);
  YReg.SetMode(AUTOMATIC);

  XReg.SetSampleTime(5);
  YReg.SetSampleTime(5);

  XReg.SetOutputLimits(-5,5);
  YReg.SetOutputLimits(-5,5);
  }

void Compute_PositionPID(){
  Com_XY();
  xsp = pid_xm;
  ysp = pid_ym;
  XReg.Compute();
  YReg.Compute();
  }  
void Init_PID_Angle(){
  pitchReg.SetMode(AUTOMATIC);
  rollReg.SetMode(AUTOMATIC);
  yawReg.SetMode(AUTOMATIC);
  
  pitchReg.SetSampleTime(5);
  rollReg.SetSampleTime(5);
  yawReg.SetSampleTime(5);
  
  pitchReg.SetOutputLimits(-40,40);
  rollReg.SetOutputLimits(-40,40);
  yawReg.SetOutputLimits(-40,40);
  
  }
void Init_PID_Rate(){
  rateXReg.SetMode(AUTOMATIC);
  rateYReg.SetMode(AUTOMATIC);
  rateZReg.SetMode(AUTOMATIC);
  
  rateXReg.SetSampleTime(5);
  rateYReg.SetSampleTime(5);
  rateZReg.SetSampleTime(5);
  
  rateXReg.SetOutputLimits(-100,100);
  rateYReg.SetOutputLimits(-100,100);
  rateZReg.SetOutputLimits(-100,100);
  }

void RateControl(){
  computePID4stabilze();
  updateRateSP();
  rateXReg.Compute();
  rateYReg.Compute();
  rateZReg.Compute();
  
  }
void updateRateSP(){
  rateXsp = pid_roll;
  rateYsp = -pid_pitch;
  rateZsp = -pid_yaw;
  }
void computePID4stabilze(){
  //Good:
  rollsp = -pidX;//+
  pitchsp = -pidY;

//  rollsp = 0;
//  pitchsp = 0;
  pitchReg.Compute();
  rollReg.Compute();
  yawReg.Compute();
}
