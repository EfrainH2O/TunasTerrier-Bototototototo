// Pre SetUp
  #include "Pines.h"
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
    int mainSensor = 1;
    bool Ball = false;
    char Color;
  // Pista B
    float p;
    float i;
    float d;
    float prevError;

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
    // Lee el ancho de pulso de ROJO
    for (int i = 0; i < 5; i++) {
      redPW = getRGBPW(-1);
      // Mapea el valor desde 0-255
      redValue = map(redPW, redMin, redMax, 255, 0);
      redValueSum += redValue;
      delay(200);
    }
    // Promedia la suma de los 5 valores del color ROJO
    redValue = redValueSum / 5;
    // Lee el ancho de pulso de VERDE
    for (int j = 0; j < 5; j++) {
      greenPW = getRGBPW(1);
      // Mapea el valor desde 0-255
      greenValue = map(greenPW, greenMin, greenMax, 255, 0);
      greenValueSum += greenValue;
      delay(200);
    }
    // Promedia la suma de los 5 valores del color VERDE
    greenValue = greenValueSum / 5;
    // Lee el ancho de pulso de AZUL
    for (int h = 0; h < 5; h++) {
      bluePW = getRGBPW(0);
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
    return DecisionTreeRGB(redValue, greenValue, blueValue);
  }

  int DecisionTreeRGB(int R, int G, int B) {
    if (B > 210) {
      if (R > 217) {
        if (R > 250) {
          if (B > 229) {
            Serial.println("Color detectado: ROSA");
            ejecutaLedRGB(255, 192, 203);  // Rosa
            return 3;
          } else {
            if (G > 220) {
              Serial.println("Color detectado: AMARILLO");
              ejecutaLedRGB(255, 255, 0);  // Amarillo
              return 4;
            } else {
              Serial.println("Color detectado: ROSA");
              ejecutaLedRGB(255, 198, 203);  // Rosa
              return 3;
            }
          }
        } else {
          Serial.println("Color detectado: MORADO");
          ejecutaLedRGB(255, 0, 255);  // Morado
          return 5;
        }
      } else {
        Serial.println("Color detectado: AZUL");
        ejecutaLedRGB(0, 0, 255);  // Azul
        return 2;
      }
    } else {
      if (G > 205) {
        if (G > 235) {
          if (B > 200) {
            Serial.println("Color detectado: AMARILLO");
            ejecutaLedRGB(255, 255, 0);  // Amarillo
            return 4;
          } else {
            Serial.println("Color detectado: VERDE");
            ejecutaLedRGB(0, 255, 0);  // Verde
            return 1;
          }
        } else {
          Serial.println("Color detectado: NARANJA");
          ejecutaLedRGB(255, 128, 0);  // Naranja
          return 6;
        }
      } else {
        if (B > 177) {
          if (R > 185) {
            Serial.println("Color detectado: VERDE");
            ejecutaLedRGB(0, 255, 0);  // Verde
            return 1;
          } else {
            Serial.println("Color detectado: MORADO");
            ejecutaLedRGB(255, 0, 255);  // Morado
            return 5;
          }
        } else {
          Serial.println("Color detectado: ROJO");
          ejecutaLedRGB(255, 0, 0);  // Rojo
          return 0;
        }
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

  long findDistance(int triggerPin, int echoPin) {
    pinMode(triggerPin, OUTPUT);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(triggerPin, LOW);
    pinMode(echoPin, INPUT);
    long distance = pulseIn(echoPin, HIGH, 50000) / 58.2;
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
    long FrontDistance() {
      return findDistance(FrontDistSensorTrigger, FrontDistSensorEcho);
    }
    long LeftDistance() {
      return findDistance(LeftDistSensorTrigger, LeftDistSensorEcho);
    }
    long RightDistance() {
      return findDistance(RightDistSensorTrigger, RightDistSensorEcho);
    }

  void GoFront(double dist) {
    LeftPulses = 0;
    RightPulses = 0;
    int blockDist = dist / 3.14 / 6.75 * (float)pulses_per_turn;  //Ponemos operaciones para ponerlo en terminos de bloques
    double RightError = 0;
    while (LeftPulses < blockDist || RightPulses < blockDist) {
      RightError = (blockDist - RightPulses);
      RightError = RightError / (float)(blockDist);
      //El error puede ser cambiado entre calcular la distancia hacia la pared o nomas con los encoders
      RightError = RightError < 0.4 && RightError > 0 ? 0.4 : RightError;
      Serial.print("L:\t");Serial.print(LeftPulses);Serial.print("\tR:\t");Serial.println(RightPulses);
      Drive(RightError, RightError);
    }
    Drive(0, 0);
    Serial.print("L:\t");Serial.print(LeftPulses);Serial.print("\tR:\t");Serial.println(RightPulses);
  }

  void FollowWall(char side, int dist) {
    LeftPulses = 0;
    RightPulses = 0;
    int blockDist = dist / 3.14 / 6.75 * (float)pulses_per_turn;
    float wallI = 0;
    float wallprevError = 0;
    float wallDer = 0;
    int dir = side == 'r' ? -1 : 1;
    float RightError;
    while (LeftPulses < blockDist || RightPulses < blockDist) {
      float error = side == 'r' ? RightDistance() : LeftDistance();
      error = error > 14 ? 14 : error;
      error = error ? error : -1;
      error = 7 - error;
      wallI += error;
      wallI = error * wallI < 0 ? 0 : wallI;
      wallDer = error - wallprevError;
      float kin = error * wallkp + wallI * wallki + wallDer*wallkd;
      RightError = (blockDist - RightPulses);
      RightError = RightError / (float)(blockDist);
      //El error puede ser cambiado entre calcular la distancia hacia la pared o nomas con los encoders
      RightError = RightError < 0.4 && RightError > 0 ? 0.4 : RightError;
      //Serial.print("L:\t");Serial.print(LeftPulses);Serial.print("\tR:\t");Serial.println(RightPulses);
      Drive(RightError + kin *dir*RightError, RightError - kin *dir* RightError);
    }
    Drive(0, 0);
    
  }

  void Turn(char input) {
    LeftPulses = 0;
    RightPulses = 0;
    int direction = input == 'r' ? 1 : -1;
    float distance = 16;
    float RightError;
    while (RightPulses < distance) {
      RightError = (distance - RightPulses);
      RightError = RightError / distance;
      //El error puede ser cambiado entre calcular la distancia hacia la pared o nomas con los encoders
      RightError = RightError < 0.5 && RightError > 0 ? 0.5 : RightError;
      Drive( direction * RightError, -RightError * direction);
    }
    Drive(0, 0);     
  }

  void UTurn() {
    LeftPulses = 0;
    RightPulses = 0;
    while (RightPulses < 41) {
      Drive(0.7, -0.7);
    }
    Drive(0, 0);
  }

// Algoritmo para Resolver Pista B
  // PID
    void PIDLinea() {
      UpdateInfraRedSensors();
      float error = -2 * InfraRedValues[0] - 1 * InfraRedValues[1] + 1 * InfraRedValues[3] + 2 * InfraRedValues[4];
      error = error == 0 ? prevError : error;
      error = InfraRedValues[2] ? 0 : error;
      p = error;
      i += error;
      i = error * i < 0 ? 0 : i;
      d = error - prevError;
      prevError = error;
      float total = kp * p + ki * i + kd * d;
      Drive(0.8 + total, 0.8 - total);
    }

// Algoritmo para Resolver Pista A
  // Secuencia para resolver Pista A con "front" como sensor main
    void movFrontSensor() {
      if (FrontDistance() > 5) {
        GoFront(25);
      } else {
        Turn('l');
        GoFront(25);
        mainSensor = 1;
      }
    }

  // Secuencia para resolver Pista A con "right" como sensor main
    void movRightSensor() {
      if (RightDistance() > 5) {
        if (FrontDistance() > 30) {
          Turn('l');
          if (Ball == false) {
            catchBall();
          } else {
            endMaze();
          }
        } else {
          Turn('r');
          GoFront(25);
        }
      } else {
        GoFront(25);
      }
    }

  // Secuencia para resolver Pista A con "left" como sensor main
    void movLeftSensor() {
      if (LeftDistance() > 5) {
        if (FrontDistance() > 30) {
          Turn('r');
          if (Ball == false) {
            catchBall();
          } else {
            endMaze();
          }
        } else {
          Turn('l');
          GoFront(25);
        }
      } else {
        GoFront(25);
      }
    }

  // Secuencia para verificar y evitar la linea negra de la Pista A
    void verfNegro() {
      UpdateInfraRedSensors();
      if (DetectLine()) {
        UTurn();
        mainSensor = (mainSensor == 1) ? 1 : 2;
      }
    }

  // Secuencia para capturar la pelota y ubicar al robot en su posicion previa
    void catchBall() {
      while (IsBall() == false) { Drive(-1, -1); }
      Ball = true;
      ActivateServos();
      GoFront(25);
      if (mainSensor == 1) {
        Turn('r');
        mainSensor = 2;
      } else {
        Turn('l');
        mainSensor = 1;
      }
    }

  // Secuencia para terminar el laberinto
    void endMaze() {
      Drive(-1, -1);
      if (leeSensorRGB() == 0) {
        Color = 'r';
      } else {
        GoFront(25);
        Color = 'g';
      }
    }

  // Resolucion de Pista A
    void resuelveLaberinto() {
      while (Ball == false) {  // Fase 1: Busqueda de pelota
        verfNegro();
        if (mainSensor == 0) {
          movFrontSensor();
        } else if (mainSensor == 1) {
          movRightSensor();
        } else {
          movLeftSensor();
        }
      }
      while (Color != 'r') {  // Fase 2: Resolver el laberinto
        verfNegro();
        if (mainSensor == 1) {
          movRightSensor();
        } else {
          movLeftSensor();
        }
      }
      while (true) { Drive(0, 0); }
    }

// Algoritmo Pista C
  void RightHandSolver(){
    int rightDistance = RightDistance();
    int leftDistance = LeftDistance();
    int frontDistance = FrontDistance();
      if(rightDistance < MAX_WAL_DIST && frontDistance > MAX_FRONT_DIST){
          FollowWall('r', DistBetweenBlock);
      }
      else if(rightDistance > MAX_WAL_DIST){
          Turn('r');
          GoFront(DistBetweenBlock);
      }else if (leftDistance > MAX_WAL_DIST){
          Turn('l');
          GoFront(DistBetweenBlock);
      }else{
          UTurn();
      }
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
}

void loop() {
  //resuelveLaberinto();
  FollowWall('r', 50);

}
