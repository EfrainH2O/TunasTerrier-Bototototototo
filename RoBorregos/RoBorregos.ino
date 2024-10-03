  // - Dist Sensors Trigger------------------------------------------- 
  #define FrontDistSensorTrigger 2
  #define LeftDistSensorTrigger 3
  #define RightDistSensorTrigger 4
  // - Dist Sensors Echo-------------------------------------------
  #define FrontDistSensorEcho 22
  #define LeftDistSensorEcho 24
  #define RightDistSensorEcho 26

int findDistance(int triggerPin, int echoPin)
{
  pinMode(triggerPin, OUTPUT);  
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);
  int distance = pulseIn(echoPin, HIGH)/58.2;
  return distance;
}  
void setup() {
  // - InfraRedSensors
  int NInfraRedSensor;
  
  int InfraRedSensors[] = {23, 25, 27, 29, 31};
  for (int i = 0; i < NInfraRedSensor; i++){
    pinmode(InfraredSensors[i] ,INPUT);
  }
  bool InfraRedValues[NInfraRedSensor];
  // -  

}

int FrontDistance(){
  return findDistance(FrontDistSensorTrigger, FrontDistSensorEcho);
}

int LeftDistance(){
  return findDistance(LeftDistSensorTrigger, LeftDistSensorEcho);
}
int RightDistance(){
  return findDistance(RightDistSensorTrigger, RightDistSensorEcho);
}

void loop() {
  int margin = 15;
  if(RightDistance() < margin){
    // Left
  }else if(LeftDistance() < margin){
    //Right
  }else if(FrontDistance() < margin) {
    //Front
  }

  // put your main code here, to run repeatedly:

}
