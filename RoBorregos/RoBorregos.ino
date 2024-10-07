  //Librerias
  // - Small Color Sensor
    #include <Wire.h>
    #include "Adafruit_TCS34725.h"
    #define RGBRedOutPut 3
    #define RGBGreenOutPut 5
    #define RGBBlueOutPut 6
    #define commonAnode true
    byte gammatable[256];
    Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
  // - Dist Sensors Trigger------------------------------------------- 
    #define FrontDistSensorTrigger 2
    #define LeftDistSensorTrigger 3
    #define RightDistSensorTrigger 4
  // - Dist Sensors Echo-------------------------------------------
    #define FrontDistSensorEcho 22
    #define LeftDistSensorEcho 24
    #define RightDistSensorEcho 26
  // - Motors
    #define RightA
    #define RightB
    #define RightPow
    #define LeftA
    #define LeftB
    #define LeftPow

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

void SetColor(){
  float red, green, blue;
  tcs.setInterrupt(false); 
  delay(60);  
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true);  

  //Funcion para redondear a colores cercanos

  analogWrite(RGBRedOutPut, gammatable[(int)red]);
  analogWrite(RGBGreenOutPut, gammatable[(int)green]);
  analogWrite(RGBBlueOutPut, gammatable[(int)blue]);
}

void Drive(int left, int right){
  //Set Motor Direction
  if(left > 0){
    digitalWrite(LeftA, HIGH);
    digitalWrite(LeftB, LOW);
  }else if(left < 0){
    digitalWrite(LeftA, LOW);
    digitalWrite(LeftB, HIGH);
  }
  else{
    digitalWrite(LeftA, LOW);
    digitalWrite(LeftB, LOW);
  }

  if(right > 0){
    digitalWrite(RightA, HIGH);
    digitalWrite(RightB, LOW);
  }else if(right < 0){
    digitalWrite(RightA, LOW);
    digitalWrite(RightB, HIGH);
  }
  else{
    digitalWrite(RightA, LOW);
    digitalWrite(RightB, LOW);
  }

  //Set Motor Velocity
  left = abs(left);
  right = abs(right);
  left = left > 1 ? 1: left;
  right = right > 1 ? 1: right;

  int leftWheel = map(left, 0, 1, 0, 255);
  int rightWheel = map(right, 0, 1, 0, 255);
  analogWrite(LeftPow, leftWheel);
  analogWrite(RightPow, RightWheel);
}


void setup() {
  // - InfraRedSensors
          int NInfraRedSensor;
          int InfraRedSensors[] = {23, 25, 27, 29, 31};
                      for (int i = 0; i < NInfraRedSensor; i++){
                        pinmode(InfraredSensors[i] ,INPUT);
                      }
          bool InfraRedValues[NInfraRedSensor];


  // -  Small Color Sensor
          pinMode(RGBRedOutPut, OUTPUT);
          pinMode(RGBGreenOutPut, OUTPUT);
          pinMode(RGBBlueOutPut, OUTPUT);

          for (int i=0; i<256; i++) {
            float x = i;
            x /= 255;
            x = pow(x, 2.5);
            x *= 255;
            gammatable[i] = 255 - x;
          }
          Serial.println(gammatable[i]);
  // - Big Color Sensor
  

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
  SetColor();
  if(RightDistance() < margin){
    Drive(-1,1);
  }else if(LeftDistance() < margin){
    Drive(1,-1);
  }else if(FrontDistance() < margin) {
    Drive(-1,-1);
  }


}
