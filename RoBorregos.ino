  // - Dist Sensors Trigger------------------------------------------- 
  int FrontDistSensorTrigger;
  int LeftDistSensorTrigger;
  int RightDistSensorTrigger;
  // - Dist Sensors Echo-------------------------------------------
  int FrontDistSensorEcho;
  int LeftDistSensorEcho;
  int RightDistSensorEcho;

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
  // - Dist Sensors Trigger------------------------------------------- 
  FrontDistSensorTrigger;
  LeftDistSensorTrigger;
  RightDistSensorTrigger;
  // - Dist Sensors Echo-------------------------------------------
  FrontDistSensorEcho;
  LeftDistSensorEcho;
  RightDistSensorEcho;
  // - InfraRedSensors
  int NInfraRedSensor;
  int InfraRedSensors[NInfraRedSensor];
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
  if(RightDistance() <margin){
    // Left
  }else if(LeftDistance() < margin){
    //Right
  }else if(FrontDistance() < margin) {
    //Front
  }

  // put your main code here, to run repeatedly:

}
