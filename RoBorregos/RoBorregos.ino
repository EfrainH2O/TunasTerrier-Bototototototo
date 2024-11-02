#include <GY_61.h>

#include <GY_61.h>

// Pre SetUp
  #include "Pines.h"
  //Gyroscopio
    #include <GY_61.h>
    GY_61 gyro;
    long prevTime;
    double angle = 0;
  // Small Color Sensor
    #include <Wire.h>
    #include "Adafruit_TCS34725.h"
    Adafruit_TCS34725 tcs= Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_4X);
  // Encoders
    // Right
      volatile byte RightPulses;
      void Rcount(){ RightPulses++; }
    // Left
      volatile byte LeftPulses;
      void Lcount(){ LeftPulses++; }
  // InfraRed
    bool InfraRedValues[NInfraRedSensor];
  // Servos
    #include <Servo.h>
    Servo Lservo;
  // Pista A
    bool Ball = false;
    bool finished = false;
  // Pista B
    float p;
    float i;
    float d;
    float prevError;
  // Pista C
    int colores[7];
     float wallI = 0;
    float wallprevError = 0;
    float wallDer = 0;
    bool turned = false;
    /*
    0 = rojo
    1 = verde
    2 = amarillo
    3 = naranja
    4 = rosa
    5 = morado
    6 = azul
    */

// Subsistemas
  void calibraSensorRGB() {
    // Variables para la Medida del Ancho de Pulso de Color
    int redPW = 0;
    int greenPW = 0;
    int bluePW = 0;
    // Lee el ancho de pulso de ROJO
    redPW = getRGBPW(-1);
    delay(200);
    // Lee el ancho de pulso de VERDE
    greenPW = getRGBPW(1);
    delay(200);
    // Lee el ancho de pulso de AZUL
    bluePW = getRGBPW(0);
    delay(200);
    // Imprimir los valores en el Serial Monitor
    Serial.print("Red PW = ");
    Serial.print(redPW);
    Serial.print(" - Green PW = ");
    Serial.print(greenPW);
    Serial.print(" - Blue PW = ");
    Serial.println(bluePW);
  }

  int leeSensorRGB() {
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
    for (int i = 0; i < 5; i++) {
      // Lee el ancho de pulso de ROJO
      redPW = getRGBPW(-1);
      // Mapea el valor desde 0-255
      redValue = map(redPW, redMin, redMax, 255, 0);
      redValueSum += redValue;
      delay(200);
      // Lee el ancho de pulso de VERDE
      greenPW = getRGBPW(1);
      // Mapea el valor desde 0-255
      greenValue = map(greenPW, greenMin, greenMax, 255, 0);
      greenValueSum += greenValue;
      delay(200);
      // Lee el ancho de pulso de AZUL
      bluePW = getRGBPW(0);
      // Mapea el valor desde 0-255
      blueValue = map(bluePW, blueMin, blueMax, 255, 0);
      blueValueSum += blueValue;
      delay(200);
    }
    // Promedia la suma de los 5 valores del color ROJO
    redValue = redValueSum / 5;
    // Promedia la suma de los 5 valores del color VERDE
    greenValue = greenValueSum / 5;
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
    return DecisionTreeRGB(redValue, greenValue, blueValue);
  }

  int DecisionTreeRGB(int R, int G, int B) {
    if (B >= 221){
      if (B >= 226){
        Serial.println("Color detectado: MORADO");
        ejecutaLedRGB(255, 0, 255);
        return 5;
      } else {
        Serial.println("Color detectado: ROSA");
        ejecutaLedRGB(255, 175, 255);
        return 4;
      }
    } else {
      if (B >= 164){
        if (R <= 220){
          Serial.println("Color detectado: GREEN");
          ejecutaLedRGB(0, 255, 0);
          return 1;
        } else {
          Serial.println("Color detectado: AMARILLO");
          ejecutaLedRGB(255, 255, 0);
          return 2;
        }
      } else {
        Serial.println("Color detectado: ROJO");
        ejecutaLedRGB(255, 0, 0);
        return 0;
      }
    } 
  }

  int getRGBPW(int color) {
    if (color < 0){  // Ajustar el sensor a leer unicamente ROJO
      digitalWrite(S2, LOW);
      digitalWrite(S3, LOW);
    } else if (color > 0){ // Ajustar el sensor a leer unicamente VERDE
      digitalWrite(S2, HIGH);
      digitalWrite(S3, HIGH);
    } else {  // Ajustar el sensor a leer unicamente AZUL
      digitalWrite(S2, LOW);
      digitalWrite(S3, HIGH);
    }
    int PW;
    // Lee la salida del ancho de pulso
    PW = pulseIn(SensorOut, LOW);
    return PW;  // Regresar el valor obtenido
  }

  void ejecutaLedRGB(int R, int G, int B) {
    analogWrite(ledRojo, R);   // Pasar el valor obtenido de ROJO al led RGB
    analogWrite(ledVerde, G);  // Pasar el valor obtenido de VERDE al led RGB
    analogWrite(ledAzul, B);   // Pasar el valor obtenido de AZUL al led RGB
  }

  double findDistance(int triggerPin, int echoPin) {
    pinMode(triggerPin, OUTPUT);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(triggerPin, LOW);
    pinMode(echoPin, INPUT);
    double distance = (double)pulseIn(echoPin, HIGH, 50000) / 58.2;
    return distance;
  }

  bool IsBall() {
    float red, green, blue;
    delay(60);
    tcs.getRGB(&red, &green, &blue);
    Serial.print(red);
    Serial.print(" ");
    Serial.print(green);
    Serial.print(" ");
    Serial.println(blue);
    return blue > green && red < blue;
  }

  float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  void Drive(float left, float right) {
    //Set Motor Direction
    if (left > 0) {
      digitalWrite(LeftA, HIGH);
      digitalWrite(LeftB, LOW);
    } else if (left < 0) {
      digitalWrite(LeftA, LOW);
      digitalWrite(LeftB, HIGH);
    } else {
      digitalWrite(LeftA, LOW);
      digitalWrite(LeftB, LOW);
    }

    if (right > 0) {
      digitalWrite(RightA, HIGH);
      digitalWrite(RightB, LOW);
    } else if (right < 0) {
      digitalWrite(RightA, LOW);
      digitalWrite(RightB, HIGH);
    } else {
      digitalWrite(RightA, LOW);
      digitalWrite(RightB, LOW);
    }

    //Set Motor Velocity
    left = abs(left);
    right = abs(right);

    left = left > 1 ? 1 : left;
    right = right > 1 ? 1 : right;

    float LeftWheel = mapfloat(left, 0, 1, 0, 255);
    float RightWheel = mapfloat(right, 0, 1, 0, 255);

    //Serial.print("LWhel: ");Serial.print(left); Serial.print("\tRWehl: "); Serial.println(right);
    analogWrite(LeftPow, LeftWheel);
    analogWrite(RightPow, RightWheel);
  }

  void UpdateInfraRedSensors() {
    for (int i = 0; i < NInfraRedSensor; i++) {
      InfraRedValues[i] = digitalRead(InfraRedSensors[i]);
      Serial.print(InfraRedValues[i]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }

  bool DetectLine() {
    //Se puede hacer con la lista de valores, pero siento que seria un tanto peligroso
    for (int i = 0; i < NInfraRedSensor; i++) {
      if ((InfraRedSensors[i]) != LOW) {
        return true;
      }
    }
    return false;
  }

  void ActivateServos() {
    Lservo.attach(servoLeft);
    Lservo.write(280);
    delay(1000);
    Lservo.detach();
  }

  // Ultrasonic sensors
    double FrontDistance() {
      return findDistance(FrontDistSensorTrigger, FrontDistSensorEcho);
    }
    double LeftDistance() {
      return findDistance(LeftDistSensorTrigger, LeftDistSensorEcho);
    }
    double RightDistance() {
      return findDistance(RightDistSensorTrigger, RightDistSensorEcho);
    }

  void GoFront(double dist) {
    float distError = FrontDistance();
    float objective = distError-dist;  //Ponemos operaciones para ponerlo en terminos de bloques
    objective = objective < 0 ? 0 : objective;
    float start = distError;
    while (distError-1 > objective) {
      distError = FrontDistance();
      if(DetectLine()){Drive(0,0);return;}
      float percentage = distError;
      percentage = (-objective +distError)*0.8 / (start-objective);
      //El error puede ser cambiado entre calcular la distancia hacia la pared o nomas con los encoders
      percentage = percentage < 0.4 ? 0.4 : percentage;
     Serial.print("Start:\t");Serial.print(start);Serial.print("\tobjective:\t");Serial.print(objective);
     Serial.print("\tdistError:\t");Serial.print(distError);Serial.print("\tpercen:\t");Serial.println(percentage);

      Drive(percentage, percentage);
    }
    return;
   // Serial.print("L:\t");Serial.print(LeftPulses);Serial.print("\tR:\t");Serial.println(RightPulses);
  }

  void FollowWall(char side, float dist) {
    float distError = FrontDistance();
    float objective = distError-dist;  //Ponemos operaciones para ponerlo en terminos de bloques
    objective = objective < 0 ? distError-1 : objective;
    wallI = 0;
    wallprevError = 0;
   while(distError > objective+1){
    distError = FrontDistance();
    Serial.print("\tobjective:\t");Serial.print(objective);
     Serial.print("\tdistError:\t");Serial.print(distError);
    int dir = side == 'r' ? -1 : 1;
    float error = side == 'r' ? RightDistance() : LeftDistance();

    if(DetectLine()){Drive(0,0);return;}
    if(error > MAX_WAL_DIST+1){Drive(0,0);return;}
    if(distError < MAX_FRONT_DIST){Drive(0,0);return;}
    error = error ? error : -1;
    error = MAX_WAL_DIST/2 - error;
    wallI += error;
    wallI = error * wallI < 0 ? 0 : wallI;
    wallDer = error - wallprevError;
    wallprevError = error;
    float kin = error * wallkp + wallI * wallki + wallDer*wallkd;
    float right = 0.5 + kin *dir < 0 ? 0.2 : 0.5 + kin *dir;
    float left = 0.5 - kin *dir < 0 ? 0.2 : 0.5 - kin *dir;
    Serial.print("L:\t");Serial.print(LeftPulses);Serial.print("\tR:\t");Serial.println(RightPulses);
    Drive(right ,left );
    }

    
  }

  void Turn(char input) {
    LeftPulses = 0;
    RightPulses = 0;
    int direction = input == 'r' ? 1 : -1;
    float distance = 40;
    float RightError;
    while (RightPulses < distance) {
      Drive( direction * 0.55, -0.55 * direction);
    }
    Drive(0, 0);     
  }

  void UTurn() {
    LeftPulses = 0;
    RightPulses = 0;
    while (RightPulses < 35) {
      Drive(0.7, -0.7);
    }
    Drive(0, 0);
  }

  void ActuAngule(){
    float y;
    float x;
    for(int i = 0; i < 120; i++){
      y +=  gyro.ready();
      y = y /2;
      x += gyro.readx();
      x = x/2;
    }
    y = y >0 ? y : -y ;

    long delta = (millis() - prevTime);
    //Serial.println((double)delta/1000);
    prevTime = millis();

    if(x > 0){
      angle += sqrt(y/8.1) * (double)delta/1000;
    }else{
      angle -= sqrt( y/8.1) * (double)delta/1000;
    }
  
  }
  

// Algoritmo para Resolver Pista B
  // PID
  float prevprevError;
    void PIDLinea() {
      UpdateInfraRedSensors();
      float error = -2 * InfraRedValues[0] - 1 * InfraRedValues[1] + 1 * InfraRedValues[3] + 2 * InfraRedValues[4];
      error = error == 0 ? prevprevError : error;
      error = InfraRedValues[2] ? 0 : error;
      p = error;
      i += error;
      i = error * i < 0 ? 0 : i;
      d = error - prevError;
      prevprevError = prevError;
      prevError = error;
      float total = kp * p + ki * i + kd * d;
      Drive(0.65 + total, 0.65 - total);
    }

// Algoritmo para Resolver Pista A
  // Secuencia para capturar la pelota y ubicar al robot en su posicion previa
    void catchBall() {
      while (LeftDistance() >= MAX_WAL_DIST/2 || RightDistance() >= MAX_WAL_DIST/2) { Drive(-0.5, -0.5); }
      Drive(0,0);
      Ball = true;
      ActivateServos();
      while (LeftDistance() <= MAX_WAL_DIST/2 || RightDistance() <= MAX_WAL_DIST/2) { Drive(0.5, 0.5); }
      Drive(0,0);
    }

  // Secuencia para terminar el laberinto
    void endMaze(char sensor) {
      while (FrontDistance() > MAX_FRONT_DIST){ Drive(0.5, 0.5); }
      Drive(0,0);
      if (leeSensorRGB() == 0){
        finished = true;
      } else {
        UTurn();
        while (FrontDistance() > MAX_FRONT_DIST){ Drive(0.5, 0.5); }
        Turn(sensor);
      }
    }
  // Secuencia completa para resolver el laberinto
    void FollowBothWall(){

      GoFront(30);
      Turn('r');
      GoFront(DistBetweenBlock);
      char main_sensor = 'l';
      char wall_sensor = 'r';
      while(!finished){
        if(DetectLine()){
          UTurn();
          char temp = main_sensor;
          main_sensor = wall_sensor;
          wall_sensor = temp;
        }
        int ballDist = main_sensor == 'l' ? LeftDistance() : RightDistance();
        int wallDist = wall_sensor == 'l' ? LeftDistance() : RightDistance();
        int frontDist = FrontDistance();
        //is ball in that place ?
        if(ballDist > MAX_WAL_DIST && ballDist < 50 && !Ball ){
          //CATCH
          Turn(wall_sensor);
          catchBall();
          Turn(main_sensor);
        }
        //cant continue because of a wall?, check if can turn towards the ball
        else if(ballDist > MAX_WAL_DIST && frontDist <= MAX_FRONT_DIST){
          //turn towards the wall
          Turn(main_sensor);
          ejecutaLedRGB(0,0,255);
        }
        // did you find the exit and have a ball?
        else if(Ball && wallDist <= MAX_WAL_DIST){
          //exit
          Turn(wall_sensor);
          endMaze(wall_sensor);
          ejecutaLedRGB(0,255,255);
        }
        // can you follow the wall
        else if(wallDist < MAX_WAL_DIST){
          //follow wall
          FollowWall(wall_sensor, DistBetweenBlock);
          ejecutaLedRGB(255,0,255);
        }
        // you cant follow one, try the other
        else if(ballDist < MAX_WAL_DIST){
          //follow the other wall
          FollowWall(main_sensor, DistBetweenBlock);
          ejecutaLedRGB(255,0,0);
        }
        //neather?
        else{
          //bruh
          GoFront(DistBetweenBlock);
          ejecutaLedRGB(0,255,0);
        }
        delay(2000);
      }
    }

// Algoritmo Pista C
  void RightHandSolver(){
    int rightDistance = RightDistance();
    int leftDistance = LeftDistance();
    int frontDistance = FrontDistance();

    Serial.print(rightDistance); Serial.print(leftDistance);Serial.println(frontDistance);
    /*if(DetectLine()){
      Drive(-0.5,-0.5);
      delay(500);
      if(rightDistance > MAX_WAL_DIST ){
          Turn('r');
          delay(1000);
          GoFront(DistBetweenBlock);
      }else if(leftDistance > MAX_WAL_DIST ){
          Turn('l');
          delay(1000);
          GoFront(DistBetweenBlock);
      }else{
          UTurn();
          GoFront(DistBetweenBlock);
          turned = false;
      }
    }*/
    if(rightDistance > MAX_WAL_DIST ){
          GoFront(6);
          Turn('r');
          delay(1000);
          GoFront(DistBetweenBlock);
          turned = true;
    }else if(frontDistance > MAX_FRONT_DIST ){
          FollowWall('r', DistBetweenBlock);
          turned = false;
      }else if (leftDistance > MAX_WAL_DIST ){
          Turn('l');
          delay(1000);
          GoFront(DistBetweenBlock);
      }else{
          UTurn();
          GoFront(DistBetweenBlock);
          turned = false;
      }
      Drive(0,0);
      leeSensorRGB();
      delay(1000);
  }

void setup() {
  Serial.begin(9600);
  // InfraRedSensors
    for (int i = 0; i < NInfraRedSensor; i++) {
      pinMode(InfraRedSensors[i], INPUT);
    }

  // LED RGB
    pinMode(ledRojo, OUTPUT);
    pinMode(ledVerde, OUTPUT);
    pinMode(ledAzul, OUTPUT);

  // Small Color Sensor
    if (tcs.begin()) {
      Serial.println("Found sensor");
    } else {
      Serial.println("No TCS34725 found ... check your connections");
      while (1);
    }

  // 4 LEDs Color Sensor
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(SensorOut, INPUT);
    // Ajustar el escalado de frecuencia a 20%
      digitalWrite(S0, HIGH);
      digitalWrite(S1, LOW);
  // Encoders
    pinMode(RightEncoder, INPUT);
    pinMode(LeftEncoder, INPUT);
  // Verifiquemos si esto afecta negativamente el rendimiento
    attachInterrupt(digitalPinToInterrupt(RightEncoder), Rcount, FALLING);
    attachInterrupt(digitalPinToInterrupt(LeftEncoder), Lcount, FALLING);
  // Servo
    Lservo.attach(servoLeft);
    Lservo.write(-20);
    delay(1000);
    Lservo.detach();
  // Gyro
    gyro = GY_61(pinX, pinY, pinZ);
}

void loop() {
  //PIDLinea();
  //FollowBothWall();
  //Turn('l');
  //delay(2000);
  //FollowWall('r');
  RightHandSolver();
  //UTurn();
  //FollowBothWall();
  //leeSensorRGB();
  //catchBall();
  //Drive(1,1);
  //leeSensorRGB();
  //delay(1000);

}
