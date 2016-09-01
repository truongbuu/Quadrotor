
/*  RC variables
 *
 */
inline void initRC(){
 // pinMode(RC_PWR, OUTPUT);
 // digitalWrite(RC_PWR, HIGH);
  
 
//  PCintPort::attachInterrupt(RC_1, rcInterrupt1, CHANGE);
//  PCintPort::attachInterrupt(RC_2, rcInterrupt2, CHANGE);
  PCintPort::attachInterrupt(RC_3, rcInterrupt3, CHANGE);
//  PCintPort::attachInterrupt(RC_4, rcInterrupt4, CHANGE);
//  PCintPort::attachInterrupt(RC_5, rcInterrupt5, CHANGE);
  
}

inline void rcInterrupt3(){
  if(!interruptLock) ch3 = micros() - rcLastChange3;
  rcLastChange3 = micros();
}
inline void rcInterrupt5(){
  if(!interruptLock) ch5 = micros() - rcLastChange5;
  rcLastChange5 = micros();
}
inline void acquireLock(){
  interruptLock = true; 
}

inline void releaseLock(){
  interruptLock = false;
}

