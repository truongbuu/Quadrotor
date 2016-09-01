/*
Since this is kind of other control problem, 
altitude control will be landed here!
*/
void Init_PID_ALT(){
  disReg.SetMode(AUTOMATIC);
  disReg.SetSampleTime(50);
  disReg.SetOutputLimits(-300,300);
  }

void getAltitudePID(){
  disReg.Compute();
  throttle = 290;
  
  up_cmd = (throttle + pid_dis)/(abs(cos(r)*cos(p)));
  }


