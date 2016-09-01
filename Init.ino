void Init(){
  #ifdef DEBUG
  Serial.begin(115200);
//  Serial.println("BEGIN:");
  #endif    

  #ifdef COMMUNICATION_ENABLE
  Serial.begin(9600);
  #endif
  
  initMPU();  
  initRC();
  SonarInit();
  armESC();
  int count;
  initYaw(); //we dont want initial yaw different from 0
  
  for(count = 0; count < 10000; count++) 
  {
    readyData();
//    Serial.println(count);
  }
  yaw_init = yaw;
  yaw = 0;
  yprLast[0] = 0;
  /*Serial.print("yaw: ");
  Serial.print(yaw);
  Serial.print("Last: ");
  Serial.print(yprLast[0]);
  Serial.println("Init ");
  Serial.println(yaw_init);
  */
  Init_PID_Angle();
  Init_PID_Rate();
  Init_PID_ALT();
  Init_PID_Velo();
  Init_PID_Position();
  Init_XY();  
  /*Opitcal FLOWWW*/
 /*Serial.println("HERE");
  if (init_ADNS3080() == false) 
  {
    Serial.println("Bad Init Status from ADNS3080 - looking for 0x17 which is the Product ID!");
    while(1);     //Wait - you are screwed!
  }*/
  
  ///For debugging////
  #ifdef DEBUG
  //Serial.println("OK");
  #endif

  #ifdef COMMUNICATION_ENABLE
  readyNow();
  #endif
  
  }
