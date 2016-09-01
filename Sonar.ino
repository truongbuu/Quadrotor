
/*Since NewPing library has some problem, we have to move on....*/
void SonarInit(){
  PCintPort::attachInterrupt(ECHO_Z, rcInterrupt5, HIGH);
  //pulseT.pulse(TRIGGER_PIN,10,HIGH);
  //Trigpin.attach(TRIGGER_PIN);
  pinMode(TRIG_Z,OUTPUT);
  pinMode(ECHO_Z,INPUT);
 // m0.attach(TRIGGER_PIN);
  }
//int a,b;
void startZ(){
  LoopZ = millis();
  IZ += LoopZ - EndZ;
  //Serial.println(IZ);
  if(IZ >= 50){
    // a = micros();
    sensor_Z = 1;
    digitalWrite(TRIG_Z,HIGH);
    IZ = 0;
    }
  else{
    digitalWrite(TRIG_Z,LOW);
    sensor_Z = 0;  
  }
  }
void endZ(){
  EndZ = millis();
  sensor_Z = 0;
  }
void readZ(){
  if(sensor_Z == 1){ //signal for reading
    
    
    digitalWrite(TRIG_Z,LOW);
    
    acquireLock();
    releaseLock();
   
    dis = ch5/58;
    dis = (dis)*abs(cos(p)*cos(r)) + 6.5*sin(p);
    
    if(virgin_z < 3){
      virgin_z ++;
      dis_pre = dis;
      }
    if(dis > 100){
      dis = dis_pre;
      }
    if(abs(dis - dis_pre) > 50){
      dis = dis_pre;
      }
    if(dis_pre > 30 && abs(dis - dis_pre) > 10){
      dis = dis_pre;
    }
      
  }  
  dis_pre = dis;
  }
float r_pre;
float p_pre;
void readFlow(){
//    float x_rate = (r - r_pre)*1000;
//    float y_rate = (p - p_pre)*1000;
    
    sensor.read_integral();
    r_pre = r;
    p_pre = p;
    x_rate = sensor.gyro_x_rate_integral() / 10.0f;
    y_rate = sensor.gyro_y_rate_integral() / 10.0f;

    flow_x = sensor.pixel_flow_x_integral() / 10.0f;      // mrad
    flow_y = sensor.pixel_flow_y_integral() / 10.0f;      // mrad  
    filter_flow();
    
    unsigned int timespan = sensor.integration_timespan();               // microseconds
    int ground_dis = sensor.ground_distance_integral();    // mm Don't use this, it is not reliable
    int quality = sensor.quality_integral();
    //Serial.println("...");
    
    //  Serial.print(x_rate);
    //  Serial.print(" ");
    //  Serial.println(flow_x);
    ground_dis = (dis+6.5*sin(p))*10;// ground dis in mm
    //Serial.print("a ");
    //Serial.print(x_rate);
    //Serial.print("b ");
    //Serial.println(flow_x);
   // Serial.print(timespan);
    if(quality>100){
      float pixel_x = flow_x - x_rate; // mrad 0.8 alr
      float pixel_y = flow_y - y_rate; // mrad
      
      // Scale based on ground dis and compute speed
      // (flow/1000) * (ground_dis/1000) / (timespan/1000000)
      move_x = pixel_x * ground_dis/1000000 -0.02*sin(x_rate/1000);     // m
      move_y = pixel_y * ground_dis/1000000 -0.02*sin(y_rate/1000);     // m 
      velo_x = move_x*1000000/timespan;
      velo_y = move_y*1000000/timespan;
      filter_velo();
      //Serial.println(timespan);
      /*x_cm += move_x ;//although we denote cm it is actually METER
      y_cm += move_y ;*/
      if(velo_x > 5){
        velo_x = velo_x_pre;
      }
      if(velo_y > 5){
        velo_y = velo_y_pre;
      }
      //Serial.println(velo_x);
      velo_x_pre = velo_x;
      velo_y_pre = velo_y;
      
      x_cm = velo_x;
      y_cm = velo_y;
      
      x_moved += move_x;
      y_moved += move_y;      
    
    //  Serial.print("a ");
      //Serial.print("c ");
      //Serial.println(pixel_x);
      if(dis<27) {
        x_cm = 0;
        y_cm = 0;
      }
        }
else{
  x_cm = 0;
  y_cm = 0;
}
    }

  
