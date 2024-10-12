    //Pre - Setup
    // - LED RGB
      #define ledRojo 41
      #define ledVerde 43
      #define ledAzul 45
    // - 4 LEDs Color Sensor
      #define S0 52
      #define S1 50
      #define S2 48
      #define S3 49
      #define SensorOut 7
      // Calibration Values
        int redMin = 0; // Red minimum value
        int redMax = 0; // Red maximum value
        int greenMin = 0; // Green minimum value
        int greenMax = 0; // Green maximum value
        int blueMin = 0; // Blue minimum value
        int blueMax = 0; // Blue maximum value
      // Variables for Color Pulse Width Measurements
        int redPW = 0;
        int greenPW = 0;
        int bluePW = 0;
      // Variables for final Color values
        int redValue;
        int greenValue;
        int blueValue;
    // - Small Color Sensor
      #include <Wire.h>
      #include "Adafruit_TCS34725.h"
      byte gammatable[256];
      Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
    // - Dist Sensors Trigger------------------------------------------- 
      #define FrontDistSensorTrigger 12
      #define LeftDistSensorTrigger 11
      #define RightDistSensorTrigger 13
    // - Dist Sensors Echo-------------------------------------------
      #define FrontDistSensorEcho 24
      #define LeftDistSensorEcho 26
      #define RightDistSensorEcho 22
    // - Motors
      #define RightA 34
      #define RightB 32
      #define RightPow 2
      #define LeftA 35
      #define LeftB 33
      #define LeftPow 3
    // - Encoders
      int pulses_per_turn = 20;
      //-Right
      #define RightEncoder 20
      volatile byte RightPulses;
      void Rcount(){RightPulses++;}
      //-Left
      #define LeftEncoder 21
      volatile byte LeftPulses;
      void Lcount(){LeftPulses++;}
    // - InfraRed
      const int NInfraRedSensor = 5;
      int InfraRedSensors[] = {23, 25, 27, 29, 31};
      bool InfraRedValues[NInfraRedSensor];
    // - Servos
      #define servoRight 10
      #define servoLeft 9

void calibraSensorRGB(){
  // Lee el ancho de pulso de ROJO
  redPW = getRedPW();
  delay(200);
  // Lee el ancho de pulso de VERDE
  greenPW = getGreenPW();
  delay(200);
  // Lee el ancho de pulso de AZUL
  bluePW = getBluePW();
  delay(200);
  // Imprimir los valores en el Serial Monitor
  Serial.print("Red PW = ");
  Serial.print(redPW);
  Serial.print(" - Green PW = ");
  Serial.print(greenPW);
  Serial.print(" - Blue PW = ");
  Serial.print(bluePW);
}

void leeSensorRGB(){

  // Lee el ancho de pulso de ROJO
  redPW = getRedPW();
  // Mapea el valor desde 0-255
  redValue = map(redPW, redMin, redMax, 255, 0);
  delay(200);
  // Lee el ancho de pulso de VERDE
  greenPW = getGreenPW();
  // Mapea el valor desde 0-255
  greenValue = map(greenPW, greenMin, greenMax, 255, 0);
  delay(200);
  // Lee el ancho de pulso de AZUL
  bluePW = getBluePW();
  // Mapea el valor desde 0-255
  blueValue = map(bluePW, blueMin, blueMax, 255, 0);
  delay(200);
  //
  ejecutaLedRGB(redValue, greenValue, blueValue);
}

int getRedPW(){
  // Ajustar el sensor a leer unicamente ROJO
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  int PW;
  // Lee la salida del ancho de pulso
  PW = pulseIn(SensorOut, LOW);
  return PW; // Regresar el valor obtenido
}

int getGreenPW(){
  // Ajustar el sensor a leer unicamente VERDE
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  int PW;
  // Lee la salida del ancho de pulso
  PW = pulseIn(SensorOut, LOW);
  return PW; // Regresar el valor obtenido
}

int getBluePW(){
  // Ajustar el sensor a leer unicamente AZUL
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  int PW;
  // Lee la salida del ancho de pulso
  PW = pulseIn(SensorOut, LOW);
  return PW; // Regresar el valor obtenido
}

void ejecutaLedRGB(int R, int G, int B){
  digitalWrite(ledRojo, R); // Pasar el valor obtenido de ROJO al led RGB
  digitalWrite(ledVerde, G); // Pasar el valor obtenido de VERDE al led RGB
  digitalWrite(ledAzul, B);  // Pasar el valor obtenido de AZUL al led RGB
  delay(1500);
}

long findDistance(int triggerPin, int echoPin)
{
  pinMode(triggerPin, OUTPUT);  
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);
  long distance = pulseIn(echoPin, HIGH, 50000)/58.2;
  return distance;
} 

void SetColor(){
  float red, green, blue;
  tcs.setInterrupt(false); 
  delay(60);  
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true);  
  
  Serial.print("R:\t"); Serial.print(int(red)); 
  Serial.print("\tG:\t"); Serial.print(int(green)); 
  Serial.print("\tB:\t"); Serial.print(int(blue));
  //Funcion para redondear a colores cercanos ()

  analogWrite(ledRojo, gammatable[(int)red]);
  analogWrite(ledVerde, gammatable[(int)green]);
  analogWrite(ledAzul, gammatable[(int)blue]);

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

  int LeftWheel = map(left, 0, 1, 0, 255);
  int RightWheel = map(right, 0, 1, 0, 255);
  analogWrite(LeftPow, LeftWheel);
  analogWrite(RightPow, RightWheel);
}

void UpdateInfraRedSensors(){
    for(int i = 0; i < NInfraRedSensor; i++){
      InfraRedValues[i] = digitalRead(InfraRedSensors[i]);
    }
}

bool DetectLine(){
  //Se puede hacer con la lista de valores, pero siento que seria un tanto peligroso
  for(int i = 0; i < NInfraRedSensor; i++){
    if(digitalRead(InfraRedSensors[i]) != LOW){
      return true;
    }
  }
  return false;
  
}

void GoFront(){
  LeftPulses = 0;
  RightPulses = 0;
  int blockDist = 15000;   //Ponemos operaciones para ponerlo en terminos de bloques
  int RightError ;
  int LeftError ;
  int kp = 0.1;
  while(LeftPulses < blockDist || RightPulses < blockDist){
    int RightError = (blockDist - RightPulses) ;
    int LeftError = (blockDist - LeftPulses);
    //El error puede ser cambiado entre calcular la distancia hacia la pared o nomas con los encoders

    if(LeftPulses < blockDist && RightPulses < blockDist){
      Drive(1,1);
    }
    else if(LeftPulses < blockDist){
      Drive(kp*RightError,0);
    }else{
      Drive(0,kp*LeftError);
    }
  }
  //Delay(5000)
}
  // - Distance sensors
        long FrontDistance(){
          return findDistance(FrontDistSensorTrigger, FrontDistSensorEcho);
        }
        long LeftDistance(){
          return findDistance(LeftDistSensorTrigger, LeftDistSensorEcho);
        }
        long RightDistance(){
          return findDistance(RightDistSensorTrigger, RightDistSensorEcho);
        }

void setup() {
  
      Serial.begin(9600);
  // - InfraRedSensors
      for (int i = 0; i < NInfraRedSensor; i++){
        pinMode(InfraRedSensors[i] ,INPUT);
      }

  // - LED RGB
      pinMode(ledRojo, OUTPUT);
      pinMode(ledVerde, OUTPUT);
      pinMode(ledAzul, OUTPUT);
      
  // - Small Color Sensor
  Serial.print("Help");
      if (tcs.begin()) {
        Serial.println("Found sensor");
      } else {
        Serial.println("No TCS34725 found ... check your connections");
        while (1); // halt!
      }
      
      Serial.print("Help?");
      for (int i=0; i<256; i++) {
        float x = i;
        x /= 255;
        x = pow(x, 2.5);
        x *= 255;
        gammatable[i] = 255;
        Serial.println(gammatable[i]);
      }
          
  // - 4 LEDs Color Sensor
      pinMode(S0, OUTPUT);
      pinMode(S1, OUTPUT);
      pinMode(S2, OUTPUT);
      pinMode(S3, OUTPUT);
      pinMode(SensorOut, INPUT);
      // Ajustar el escalado de frecuencia a 20%
      digitalWrite(S0, HIGH);
      digitalWrite(S1, LOW);
  // - Encoders
      pinMode(RightEncoder, INPUT);
      pinMode(LeftEncoder, INPUT);
      //Verifiquemos si esto afecta negativamente el rendimiento
      attachInterrupt(digitalPinToInterrupt(RightEncoder), Rcount, FALLING );
      attachInterrupt(digitalPinToInterrupt(LeftEncoder), Lcount, FALLING );
  // General Setup
}



void loop() {
  /* Funciona 
  Serial.print("Frente: "); Serial.print((int)FrontDistance());
  Serial.print("\tDerecha: "); Serial.print(RightDistance());
  Serial.print("\tLeft: "); Serial.print(LeftDistance());
  Serial.print("\n");*/

  SetColor();


}
