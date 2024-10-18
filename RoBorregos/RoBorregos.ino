//Pre - Setup
    // - LED RGB
      #define ledRojo 7
      #define ledVerde 5
      #define ledAzul 6
    // - 4 LEDs Color Sensor
      #define S0 53
      #define S1 52
      #define S2 51
      #define S3 50
      #define SensorOut 4
      // Calibration Values
        int redMin = 26; // Red minimum value
        int redMax = 206; // Red maximum value
        int greenMin = 28; // Green minimum value
        int greenMax = 276; // Green maximum value
        int blueMin = 22; // Blue minimum value
        int blueMax = 213; // Blue maximum value
      // Threshold Variable
        int thresholdValue = 25 // Error acceptance
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
      #define FrontDistSensorEcho 23
      #define LeftDistSensorEcho 25
      #define RightDistSensorEcho 27
    // - Motors
      #define RightA 32
      #define RightB 34
      #define RightPow 2 
      #define LeftA 35
      #define LeftB 33
      #define LeftPow 3
    // - Encoders
      int pulses_per_turn = 40;
      //-Right
      #define RightEncoder 19
      volatile byte RightPulses;
      void Rcount(){RightPulses++;}
      //-Left
      #define LeftEncoder 18
      volatile byte LeftPulses;
      void Lcount(){LeftPulses++;}
    // - InfraRed 
      const int NInfraRedSensor = 5;
      int InfraRedSensors[] = {22, 24, 26, 28, 30};
      bool InfraRedValues[NInfraRedSensor];
    // - Servos
      #include <Servo.h>
      Servo Rservo;
      Servo Lservo;
      #define servoRight 9
      #define servoLeft 10

// - Subsistemas
  
  void calibraSensorRGB(){
    // Variables para la Medida del Ancho de Pulso de Color
    int redPW = 0;
    int greenPW = 0;
    int bluePW = 0;
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
    Serial.println(bluePW);
  }

  void leeSensorRGB(){
    // Variables para la Medida del Ancho de Pulso de Color
    int redPW = 0;
    int greenPW = 0;
    int bluePW = 0;
    // Variables para el Valor Final del Color
    int redValue = 0;
    int greenValue = 0;
    int blueValue = 0;
    // Variables para sumar los primeros 5 valores del color
    int redValueSum = 0;
    int greenValueSum = 0;
    int blueValueSum = 0;
    // Lee el ancho de pulso de ROJO
    for (int i = 0; i < 5; i++) { 
      redPW = getRedPW();
      // Mapea el valor desde 0-255
      redValue = map(redPW, redMin, redMax, 255, 0);
      redValueSum += redValue;
      delay(200);
    }
    // Promedia la suma de los 5 valores del color ROJO
    redValue = redValueSum / 5;
    // Lee el ancho de pulso de VERDE
    for (int j = 0; j < 5; j++) {
      greenPW = getGreenPW();
      // Mapea el valor desde 0-255
      greenValue = map(greenPW, greenMin, greenMax, 255, 0);
      greenValueSum += greenValue;
      delay(200);
    }
    // Promedia la suma de los 5 valores del color VERDE
    greenValue = greenValueSum / 5;
    // Lee el ancho de pulso de AZUL
    for(int h = 0; h < 5; h++) {
      bluePW = getBluePW();
      // Mapea el valor desde 0-255
      blueValue = map(bluePW, blueMin, blueMax, 255, 0);
      blueValueSum += blueValue;
      delay(200);
      }
    // Promedia la suma de los 5 valores del color AZUL
    blueValue = blueValueSum / 5;
    // Imprimir los valores finales en el Serial Monitor
    Serial.print("Red Value = ");
    Serial.print(redValue);
    Serial.print(" - Green Value = ");
    Serial.print(greenValue);
    Serial.print(" - Blue Value = ");
    Serial.println(blueValue);
        // Mapeo de colores
    if (redValue > 200 &&  blueValue > 200 && greenValue + thresholdValue < 200){
      Serial.println("Color detectado: Morado");
      ejecutaLedRGB(128, 0, 128);
    }
    else if (redValue > 250 && greenValue + thresholdValue < 200 && blueValue + thresholdValue < 200){
      Serial.println("Color detectado: Rosa");
      ejecutaLedRGB(255, 105, 180);
    }
    else if (redValue > 200 && greenValue > 200 && blueValue + thresholdValue < 200){
      Serial.println("Color detectado: Amarillo");
      ejecutaLedRGB(255, 255, 0);
    }
    else if (redValue > 250 && greenValue + thresholdValue < 200 && blueValue + thresholdValue < 150){
      Serial.println("Color detectado: Naranja");
      ejecutaLedRGB(255, 165, 0);
    }
    else if (redValue > 250 && greenValue + thresholdValue < 150 && blueValue + thresholdValue < 150){
      Serial.println("Color detectado: Rojo");
      ejecutaLedRGB(255, 0, 0);
    }
    else if(redValue + thresholdValue < 150 && greenValue > 250 && blueValue + thresholdValue < 150){
      Serial.print("Color detectado: Verde");
      ejecutaLedRGB(0, 255, 0);
    }
    else if (redValue < thresholdValue && greenValue < thresholdValue && blueValue < thresholdValue){
      Serial.println("Color detectado: Negro");
      ejecutaLedRGB(0, 0, 255);
    }
    else {
      Serial.println("Color indeterminado");
      ejecutaLedRGB(255, 255, 255);
    }
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
    analogWrite(ledRojo, R); // Pasar el valor obtenido de ROJO al led RGB
    analogWrite(ledVerde, G); // Pasar el valor obtenido de VERDE al led RGB
    analogWrite(ledAzul, B);  // Pasar el valor obtenido de AZUL al led RGB
  }

  long findDistance(int triggerPin, int echoPin){
    pinMode(triggerPin, OUTPUT);  
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(triggerPin, LOW);
    pinMode(echoPin, INPUT);
    long distance = pulseIn(echoPin, HIGH, 50000)/58.2;
    return distance;
  } 

  bool IsBall(){
    float red, green, blue;
    tcs.setInterrupt(false);
    delay(60);  
    tcs.getRGB(&red, &green, &blue);
    tcs.setInterrupt(true);  
  /* analogWrite(ledRojo, red);
    analogWrite(ledVerde, green);
    analogWrite(ledAzul, blue);*/
    Serial.print(red); Serial.print(" "); Serial.print(green); Serial.print(" "); Serial.println(blue);
    return blue > green && red < blue;
  }
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
  void Drive(float left, float right){
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

    float LeftWheel = mapfloat(left, 0, 1, 0, 255);
    float RightWheel = mapfloat(right, 0, 1, 0, 255);
    
    //Serial.print("LWhel: ");Serial.print(left); Serial.print("\tRWehl: "); Serial.println(right);
    analogWrite(LeftPow, LeftWheel);
    analogWrite(RightPow, RightWheel);
  }

  void UpdateInfraRedSensors(){
      for(int i = 0; i < NInfraRedSensor; i++){
        InfraRedValues[i] = digitalRead(InfraRedSensors[i]);
        Serial.print(InfraRedValues[i]);
        Serial.print(" ");
      }
      Serial.print("\n");
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

  void GoFront(double dist){
    LeftPulses = 0;
    RightPulses = 0;
    int blockDist = dist/3.14/6.75*pulses_per_turn;   //Ponemos operaciones para ponerlo en terminos de bloques
    double RightError = 0;
    double LeftError = 0;
    while(LeftPulses < blockDist || RightPulses < blockDist){
      RightError = (blockDist - RightPulses) ;
      LeftError = (blockDist - LeftPulses); // map(LeftError, 0, blockDist, 0, 1);
      Serial.print("Lpulses: "); Serial.print(LeftPulses);Serial.print("\tRPulses:"); Serial.print(RightPulses);
      LeftError = LeftError/(float)(blockDist);
      RightError = RightError/(float)(blockDist);
      //El error puede ser cambiado entre calcular la distancia hacia la pared o nomas con los encoders
      RightError = RightError <0.7 && RightError > 0 ? 0.7 : RightError;
      RightError = RightError < 0 ? 0 : RightError;
      LeftError = LeftError <0.7 && LeftError > 0 ? 0.7 : LeftError;
      LeftError = LeftError < 0 ? 0 : LeftError;
      Drive(LeftError, RightError);

      Serial.print("\t\t\tLError: "); Serial.print(LeftError);Serial.print("\tRIghtError:"); Serial.println(RightError);

    }
    
    Drive(0,0);
  }

  void ActivateServos(int Limit){
    Serial.print("Adelante");
    for(int i = -10; i < Limit; i+=10){
      Lservo.write(-i);
    }
    delay(300);
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
      if (tcs.begin()) {
        Serial.println("Found sensor");
      } 
      else {
        Serial.println("No TCS34725 found ... check your connections");
        while (1);
      }
  
      for (int i=0; i<256; i++) {
        float x = i;
        x /= 255;
        x = pow(x, 2.5);
        x *= 255;
        gammatable[i] = 255;
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
  // - Servo
      //Rservo.attach(servoRight);
      Lservo.attach(servoLeft);
}



void loop() {
  /* Funciona 
  Serial.print("Frente: "); Serial.print((int)FrontDistance());
  Serial.print("\tDerecha: "); Serial.print(RightDistance());
  Serial.print("\tLeft: "); Serial.print(LeftDistance());
  Serial.print("\n");*/

/*Funciona
Serial.println(IsBall());

if(IsBall()){
   ActivateServos(200);
}else{
  ActivateServos(10);
}

*/
//GoFront(15);
//Drive(0.5,0.5);
//delay(5000);
/*digitalWrite(RightA, HIGH);
digitalWrite(RightB, LOW);
analogWrite(RightPow, 255);*/
 
/*UpdateInfraRedSensors();
delay(500); */
 
  /*
  digitalWrite(servoRight, HIGH);
  digitalWrite(servoLeft, HIGH);
  delay(5000);
    
  digitalWrite(servoRight, LOW);
  digitalWrite(servoLeft, LOW);
  delay(3000);*/
  calibraSensorRGB();
  //leeSensorRGB();
}
