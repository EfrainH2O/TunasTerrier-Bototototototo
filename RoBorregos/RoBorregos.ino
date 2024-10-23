//Pre - Setup
  #include "Pines.h"
  // - Small Color Sensor
    #include <Wire.h>
    #include "Adafruit_TCS34725.h"
    Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
  // - Encoders
    //-Right
    volatile byte RightPulses;
    void Rcount(){RightPulses++;}
    //-Left
    volatile byte LeftPulses;
    void Lcount(){LeftPulses++;}
  // - InfraRed 
    bool InfraRedValues[NInfraRedSensor];
  // - Servos
    #include <Servo.h>
    Servo Rservo;
    Servo Lservo;
  // - PID
    float p;
    float i;
    float d;
    float prevError;

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
    Serial.print(red); Serial.print(" "); Serial.print(green); Serial.print(" "); Serial.println(blue);
    return blue > green && red < blue;
  }

  float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
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
      }
  }

  bool DetectLine(){
    //Se puede hacer con la lista de valores, pero siento que seria un tanto peligroso
    for(int i = 0; i < NInfraRedSensor; i++){
      if((InfraRedSensors[i]) != LOW){
        return true;
      }
    }
    return false;
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
      RightError = RightError <0.3 && RightError > 0 ? 0.3 : RightError;
      LeftError = LeftError <0.3 && LeftError > 0 ? 0.3 : LeftError;
      RightError = RightError < 0 ? 0 : RightError;
      LeftError = LeftError < 0 ? 0 : LeftError;
      Drive(LeftError, RightError);
      Serial.print("\t\t\tLError: "); Serial.print(LeftError);Serial.print("\tRIghtError:"); Serial.println(RightError);
    }
    while(LeftPulses < RightPulses){
      Drive(0.3,0);
      Serial.print("Lpulses: "); Serial.print(LeftPulses);Serial.print("\tRPulses:"); Serial.println(RightPulses);
    }
    while(RightPulses < LeftPulses){
      Serial.print("Lpulses: "); Serial.print(LeftPulses);Serial.print("\tRPulses:"); Serial.println(RightPulses);
      Drive(0,0.3);
    }
    Drive(0,0);
  }

  void PIDLinea(){
    
    UpdateInfraRedSensors();
    int error = -2*InfraRedValues[0] - InfraRedValues[1] + InfraRedValues[3] + 2*InfraRedValues[4];
    p = error ;
    i += error;
    i = error * i < 0 ? 0 : i;
    d = error - prevError;
    prevError = error;
    float total = p + i + d;
    Drive(0.7+total,0.7-total);
  }
// Algoritmo de resolucion de Pista A
  // Secuencia para resolver Pista A con "front" como sensor main
    void movFrontSensor() {
      if (FrontDistance() > 5) { goFront(); }
      else {
        goLeft();
        goFront();
        mainSensor = 1;
      }
    }

  // Secuencia para resolver Pista A con "right" como sensor main
    void movRightSensor() {
      if (RightDistance() > 5) {
        if (FrontDistance() > 30) { catchBall(); }
        else { goFront(); }
      }
      else { 
        goRight();
        goFront();
      }
    }

  // Secuencia para resolver Pista A con "left" como main
    void movLeftSensor(){
      if (LeftDistance() > 5) {
        if (FrontDistance() > 30) { catchBall(); }
        else { goFront(); }
      }
      else {
        goLeft();
        goFront();
      }
    }

  // Secuencia para verificar y evitar la linea negra de la Pista A
    void verfNegro(){
      if (DetectLine()){
        goInverted();
        if (main == 1){
          main = 2;
        }
        else {
          main = 1;
        }
      }
    }

  // Resolucion de Pista A
    void resuelveLaberinto(int mainSensor){
      while(Ball == false){ // Fase 1: Busqueda de pelota
        verfNegro();
        if(mainSensor == 0){ movFrontSensor(); }
        else if (mainSensor == 1){ moveRightSensor(); }
        else { moveLeftSensor(); }
      }
      while(Color != "green"){
        verfNegro();
      }
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
  i = 0;
  prevError = 0;
  while(DetectLine()){
    PIDLinea();
  }
  Drive(-5,-5);
}
