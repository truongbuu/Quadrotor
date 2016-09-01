/*The protocol will have 3 main goals:
 * Receive active command: For now, only use for active quadcopter
 * Sending data: Back formatted data to Matlab.
 * Receive stop command
 * Future orientation: Use Rasberry Pi for communication
 */
  
void readyNow(){
  Serial.println("Y");
  while(Serial.read() != 'I'){
    Serial.println("Y");
    }
  }

void sendPacket(){
  Serial.print("a ");
  Serial.print(roll);
  Serial.print("b ");
  Serial.print(pitch);
  Serial.print("c ");
  Serial.print(yaw);
  Serial.print("d ");
  Serial.println(dis);
  }
int FALL_NOW = 0;
void StopNow(){
  if(Serial.read() == 'O')
  {/*
  motor0.writeMicroseconds(1000);
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
 // dis_sp = 2;
  while(Serial.read() != 'I');*/
  dis_sp = 0;
  FALL_NOW = 1;
  }

  if(FALL_NOW == 1){
    if(dis < 5){
      motor0.writeMicroseconds(1000);
      motor1.writeMicroseconds(1000);
      motor2.writeMicroseconds(1000);
      motor3.writeMicroseconds(1000);
      while(1);
    }
  }
  /*if(FALL_NOW ==1){
    if(dis_sp > 0)
    dis_sp -= 1;
    //Serial.println(dis_sp);
    if(dis < 5){
      motor0.writeMicroseconds(1000);
      motor1.writeMicroseconds(1000);
      motor2.writeMicroseconds(1000);
      motor3.writeMicroseconds(1000);
      while(1); //stop now !
    }
  }*/
  //Serial.println(v0);
  }
